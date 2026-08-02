"""Baja P&C core library — re-export all public symbols.

Sub-modules carry the implementation; this __init__.py preserves
backward compatibility so every existing ``from .core import X``
in the node layer continues to work unchanged.
"""

from .config import (  # noqa: F401
    ControllerConfig,
    LQRConfig,
    PlannedTrajectory,
    PlannerConfig,
    PlanResult,
    SpeedProfileConfig,
)
from .controller import (  # noqa: F401
    LQRController,
    build_lqr_matrices,
    compute_feedforward,
    compute_lqr_control,
    compute_lqr_steering,
    estimate_lqr_state,
    legacy_path_control,
    smooth_velocity,
)
from .collision import (  # noqa: F401
    point_to_oriented_box_clearance,
    polyline_distance,
    segment_is_safe,
)
from .geometry import (  # noqa: F401
    _WORLD_YAW_OFFSET,
    base_to_world,
    clamp,
    distance,
    frenet_to_world,
    gps_to_local,
    nav_to_world_yaw,
    nearest_index,
    quaternion_to_rpy,
    quaternion_to_yaw,
    rpy_to_quaternion,
    signed_lateral,
    world_to_base,
    world_to_nav_yaw,
    wrap_angle,
    yaw_to_quaternion,
    Point,
)
from .planner import (  # noqa: F401
    plan_frenet_path,
)
from .speed_profile import (  # noqa: F401
    plan_speed_profile,
)
from .terrain import (  # noqa: F401
    terrain_height,
)
from .track import (  # noqa: F401
    generate_boundaries,
    generate_centerline,
    generate_obstacles,
    smooth_centerline_c2,
)
