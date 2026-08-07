"""Terrain elevation — hills, bumps, composite profile."""

from __future__ import annotations

import math


def _smooth_hill(s_coord: float, start: float, length: float, height: float) -> float:
    if s_coord < start or s_coord > start + length:
        return 0.0
    phase = math.pi * (s_coord - start) / length
    return height * math.sin(phase) ** 2


def _circular_speed_bump(s_coord: float, center: float, width: float,
                         height: float) -> float:
    half_width = width * 0.5
    offset = abs(s_coord - center)
    if offset > half_width:
        return 0.0
    radius = (half_width * half_width + height * height) / (2.0 * height)
    baseline = radius - height
    return math.sqrt(max(0.0, radius * radius - offset * offset)) - baseline


def terrain_height(s_coord: float) -> float:
    return (
        _smooth_hill(s_coord, start=16.0, length=14.0, height=0.55)
        + _smooth_hill(s_coord, start=76.0, length=13.0, height=0.45)
        + _circular_speed_bump(s_coord, center=39.0, width=1.20, height=0.08)
        + _circular_speed_bump(s_coord, center=92.0, width=1.00, height=0.07)
    )
