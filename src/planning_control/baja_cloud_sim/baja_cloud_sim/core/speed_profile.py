"""Curvature-aware speed profile planner（曲率感知速度剖面规划器）.

这个模块干一件事: 给定一条路径上每一点的曲率 κ(s) 和对应的弧长 s,
算出每一点的"建议限速" v(s). 下游(纵向控制器 / path_follower)就照着这个
v(s) 去跑 —— 直道快、弯道慢、进弯前提前减速、出弯后平滑加速.

为什么需要它?
------------
纯跟踪 / LQR 只管"往哪拐", 不管"该跑多快". 如果全程一个速度, 弯道里离心力
a_lat = v²·κ 会爆表, 转向饱和 → 出界 / 蛇形振荡. 所以必须预先告诉车:
"前面 18m 有个弯, 现在就该把速度从 4.0 降到 2.0 了".

物理核心公式(全程就这一条):
    a_lat = v² · κ   →   v_max = sqrt(a_lat_max / κ)
即: 给定允许的最大侧向加速度 a_lat_max, 曲率 κ 越大, 允许的速度越小.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence

from .config import SpeedProfileConfig
from .geometry import clamp


def plan_speed_profile(
    curvatures: Sequence[float],
    arc_lengths: Sequence[float],
    cfg: Optional[SpeedProfileConfig] = None,
) -> List[float]:
    """计算路径上每一点的建议速度 v(s).

    Args:
        curvatures:  路径上每点的曲率 κ (1/m). 直线 κ≈0, 弯越大 κ 越大.
        arc_lengths: 每点沿路径的累计弧长 s (m), 必须单调递增.
        cfg:         调参集合(SpeedProfileConfig), 含 max_speed / max_lateral_accel /
                     max_accel / max_decel / curvature_lookahead_m / pre_decel_lookahead_m
                     / pre_decel_max / min_speed 等.

    Returns:
        v: 长度与 curvatures 相同的列表, 每点建议速度 (m/s).
           已 clamp 到 [min_speed, max_speed].
    """
    if cfg is None:
        cfg = SpeedProfileConfig()
    N = len(curvatures)
    if N == 0:
        return []
    import numpy as np
    # 初始化: 先假设全程都能跑最高速 max_speed, 后面逐步用各种约束往下压.
    v = np.full(N, cfg.max_speed, dtype=np.float64)

    # ─────────────────────────────────────────────────────────────────────
    # 第 1 步: 曲率约束 —— 弯道里速度不能超过 sqrt(a_lat_max / κ)
    # ─────────────────────────────────────────────────────────────────────
    # 曲率前瞻+后视: 当前点速度受 [i-look, i+look] 区间内最大曲率约束.
    # 前瞻(i+look): 提前减速进弯(车还没到弯, 但前方有弯就压速).
    # 后视(i-look): 延后加速出弯(车还在弯里/刚出弯, 后方弯道曲率仍约束速度,
    #   只有车离开弯道 look 距离后才允许速度升高). look 足够小(3m)不影响远处直道.
    # 目的: 防止"车还在弯里(转向δ_ff满弯)就因前方直道κ=0提前加速"导致离心力飙升+LQR跟不上→蛇形.
    look_m = max(0.0, getattr(cfg, "curvature_lookahead_m", 0.0))
    if look_m > 0.0:
        # 预算每个点的前方/后方 lookahead 索引界.
        # 因为 arc_lengths 单调, 用双指针(j_hi / j_lo)滑动窗口, O(N) 而非 O(N²).
        fwd_idx = [0] * N
        bwd_idx = [0] * N
        j_hi = 0
        j_lo = 0
        for i in range(N):
            target_hi = arc_lengths[i] + look_m
            while j_hi < N - 1 and arc_lengths[j_hi] < target_hi:
                j_hi += 1
            fwd_idx[i] = j_hi
            target_lo = arc_lengths[i] - look_m
            while j_lo < i and arc_lengths[j_lo] < target_lo:
                j_lo += 1
            bwd_idx[i] = j_lo
        for i in range(N):
            # 取 [bwd_idx[i], fwd_idx[i]] 区间内最大 |κ| 作为本点的约束曲率.
            k = 0.0
            for jj in range(bwd_idx[i], fwd_idx[i] + 1):
                ak = abs(curvatures[jj])
                if ak > k:
                    k = ak
            k = max(k, 1e-4)  # 防止 κ=0 时除零; 1e-4 趋近直线, v 接近 max_speed
            v[i] = min(v[i], math.sqrt(cfg.max_lateral_accel / k))
    else:
        # look_m=0: 只看本点曲率, 不做前后视窗口.
        for i in range(N):
            k = max(abs(curvatures[i]), 1e-4)
            v[i] = min(v[i], math.sqrt(cfg.max_lateral_accel / k))

    # ─────────────────────────────────────────────────────────────────────
    # 第 2 步: 前向 pass —— 加速度约束(只能慢慢加速)
    # ─────────────────────────────────────────────────────────────────────
    # Speed can only increase so fast along the path (limited by max_accel).
    # 运动学: v[i]² = v[i-1]² + 2·a·Δs  →  v[i] ≤ sqrt(v[i-1]² + 2·a·Δs)
    # 即: 即使前方是直道, 从慢点加速到快点也需要距离, 不能瞬间提速.
    # 注意: 这是"加速度限制", 不是"强制减速". 它只把 v[i] 往下调到满足加速距离,
    # 不会主动把直道速度拉到 0(旧版错误地用 max_decel 做前向 pass, 导致直道速度塌缩).
    for i in range(1, N):
        ds = arc_lengths[i] - arc_lengths[i - 1]
        if ds <= 0.0:
            continue
        v_limit = math.sqrt(max(0.0, v[i - 1] ** 2 + 2.0 * cfg.max_accel * ds))
        v[i] = min(v[i], v_limit)

    # ─────────────────────────────────────────────────────────────────────
    # 第 3 步: 后向 pass —— 减速约束(弯前必须提前刹下来)
    # ─────────────────────────────────────────────────────────────────────
    # 从后往前扫: 下游(前方)的低速点会"反向"把减速距离传播回来.
    # 运动学(减速): v[i]² = v[i+1]² + 2·|decel|·Δs
    #   →  v[i] ≤ sqrt(v[i+1]² + 2·|decel|·Δs)
    # 含义: 要到达前方 i+1 处的低速 v[i+1], 在距离 Δs 内必须用不超过 |decel| 的减速度
    # 提前降下来. 这样 v(s) 在弯前就形成一段平滑下降的"刹车坡", 而不是到弯口才急刹.
    # 典型减速距离约 1~2m(对一般的速度落差), 所以单纯 backward pass 的刹车坡偏短.
    for i in range(N - 2, -1, -1):
        ds = arc_lengths[i + 1] - arc_lengths[i]
        if ds <= 0.0:
            continue
        v_limit = math.sqrt(max(0.0, v[i + 1] ** 2 + 2.0 * (-cfg.max_decel) * ds))
        v[i] = min(v[i], v_limit)

    # ─────────────────────────────────────────────────────────────────────
    # 第 4 步: 前向预减速 pass —— 把"刹车坡"拉得更长更柔(核心防饱和手段)
    # ─────────────────────────────────────────────────────────────────────
    # 机理: 从前往后扫, 对任意点 i, 若前方 lookahead 窗口内有更低的 v[j],
    # 则要求 v[i] 满足 v[i]² ≥ v[j]² + 2·pre_decel_max·(s[j]-s[i]),
    # 即"从 i 到 j 这段距离内, 以不超过 pre_decel_max 的减速度平滑降下来".
    # 为什么需要它: backward pass 只在弯前约 3m 内急刹, 进弯速度常降不到位 →
    #   进弯时速度还偏高 → 需要的转向角超过 EPS 齿条转速 → 转向饱和 → 出界.
    # pre_decel_max ≤ max_decel 保证这只是 backward pass 结果的"更平滑前移",
    #   不会放松任何物理约束(只是把刹车坡从 3m 拉长到 pre_decel_lookahead_m 米).
    # v2.7: pre_decel_lookahead_m 8→18, pre_decel_max 1.5→1.0, 进弯更早更柔.
    pre_look = max(0.0, getattr(cfg, "pre_decel_lookahead_m", 0.0))
    pre_a = getattr(cfg, "pre_decel_max", cfg.max_decel)
    if pre_look > 0.0 and pre_a > 0.0:
        j_hi = 0
        for i in range(N):
            target = arc_lengths[i] + pre_look
            while j_hi < N - 1 and arc_lengths[j_hi] < target:
                j_hi += 1
            for jj in range(i + 1, j_hi + 1):
                ds = arc_lengths[jj] - arc_lengths[i]
                if ds <= 0.0:
                    continue
                # 从 i 到 jj 允许以 pre_a 减速到达 v[jj]:
                #   v[i] ≥ sqrt(v[jj]² + 2·pre_a·ds)
                v_limit = math.sqrt(max(0.0, v[jj] ** 2 + 2.0 * pre_a * ds))
                if v_limit < v[i]:
                    v[i] = v_limit

    # ─────────────────────────────────────────────────────────────────────
    # 收尾: clamp 到 [min_speed, max_speed]
    # 保证任何点速度都不低于 min_speed(避免趴窝) 且不超 max_speed.
    # ─────────────────────────────────────────────────────────────────────
    return [float(clamp(vi, cfg.min_speed, cfg.max_speed)) for vi in v]
