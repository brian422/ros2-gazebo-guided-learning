"""Pure mathematical helpers for educational waypoint control."""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class WaypointError:
    """Geometric error from the rover pose to one planar target."""

    distance: float
    desired_heading: float
    heading_error: float


def normalize_angle(angle: float) -> float:
    """Wrap a finite angle to the interval [-pi, pi]."""
    if not math.isfinite(angle):
        raise ValueError('angle must be finite')
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a finite, nonzero quaternion."""
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion components must be finite')

    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1.0e-12:
        raise ValueError('quaternion norm must be greater than zero')

    x_n, y_n, z_n, w_n = (value / norm for value in values)
    sin_yaw = 2.0 * (w_n * z_n + x_n * y_n)
    cos_yaw = 1.0 - 2.0 * (y_n * y_n + z_n * z_n)
    return math.atan2(sin_yaw, cos_yaw)


def calculate_waypoint_error(
    x: float,
    y: float,
    yaw: float,
    target_x: float,
    target_y: float,
) -> WaypointError:
    """Calculate distance, desired heading, and normalized heading error."""
    values = (x, y, yaw, target_x, target_y)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('pose and waypoint values must be finite')

    delta_x = target_x - x
    delta_y = target_y - y
    desired_heading = math.atan2(delta_y, delta_x)
    return WaypointError(
        distance=math.hypot(delta_x, delta_y),
        desired_heading=desired_heading,
        heading_error=normalize_angle(desired_heading - yaw),
    )


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a finite value between finite ordered limits."""
    if not all(math.isfinite(item) for item in (value, lower, upper)):
        raise ValueError('clamp inputs must be finite')
    if lower > upper:
        raise ValueError('lower limit must not exceed upper limit')
    return min(max(value, lower), upper)


def saturate_velocity(
    linear_velocity: float,
    angular_velocity: float,
    max_linear_speed: float,
    max_angular_speed: float,
) -> Tuple[float, float]:
    """Clamp a velocity pair to symmetric positive speed limits."""
    if max_linear_speed <= 0.0 or max_angular_speed <= 0.0:
        raise ValueError('velocity limits must be greater than zero')
    linear = clamp(
        linear_velocity,
        -max_linear_speed,
        max_linear_speed,
    )
    angular = clamp(
        angular_velocity,
        -max_angular_speed,
        max_angular_speed,
    )
    return linear, angular
