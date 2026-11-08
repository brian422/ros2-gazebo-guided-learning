"""Unit tests for reusable waypoint-control mathematics."""

import math

import pytest

from hazard_inspection_control.control_math import (
    calculate_waypoint_error,
    normalize_angle,
    quaternion_to_yaw,
    saturate_velocity,
)


@pytest.mark.parametrize(
    ('input_angle', 'expected'),
    [
        (0.0, 0.0),
        (2.0 * math.pi, 0.0),
        (math.pi + 0.1, -math.pi + 0.1),
        (-math.pi - 0.1, math.pi - 0.1),
    ],
)
def test_angle_normalization(
    input_angle: float,
    expected: float,
) -> None:
    """Angles must wrap to an equivalent value in [-pi, pi]."""
    assert normalize_angle(input_angle) == pytest.approx(expected)


def test_waypoint_distance_and_heading() -> None:
    """A 3-4-5 triangle gives a known distance and desired heading."""
    error = calculate_waypoint_error(
        x=1.0,
        y=2.0,
        yaw=0.0,
        target_x=4.0,
        target_y=6.0,
    )

    assert error.distance == pytest.approx(5.0)
    assert error.desired_heading == pytest.approx(math.atan2(4.0, 3.0))
    assert error.heading_error == pytest.approx(math.atan2(4.0, 3.0))


def test_waypoint_heading_error_is_normalized() -> None:
    """The controller must choose the short rotation across the pi boundary."""
    error = calculate_waypoint_error(
        x=0.0,
        y=0.0,
        yaw=-math.pi + 0.05,
        target_x=-1.0,
        target_y=0.0,
    )

    assert abs(error.heading_error) < 0.1


def test_velocity_saturation() -> None:
    """Requested commands must be clamped to the configured speed limits."""
    linear, angular = saturate_velocity(
        linear_velocity=2.5,
        angular_velocity=-4.0,
        max_linear_speed=0.35,
        max_angular_speed=1.2,
    )

    assert linear == pytest.approx(0.35)
    assert angular == pytest.approx(-1.2)


@pytest.mark.parametrize(
    ('x', 'y', 'z', 'w'),
    [
        (0.0, 0.0, math.nan, 1.0),
        (0.0, 0.0, 0.0, 0.0),
    ],
)
def test_invalid_quaternion_is_rejected(
    x: float,
    y: float,
    z: float,
    w: float,
) -> None:
    """Invalid orientation data must not enter the feedback calculation."""
    with pytest.raises(ValueError):
        quaternion_to_yaw(x, y, z, w)


def test_invalid_waypoint_data_is_rejected() -> None:
    """NaN pose data must fail explicitly."""
    with pytest.raises(ValueError):
        calculate_waypoint_error(
            x=math.nan,
            y=0.0,
            yaw=0.0,
            target_x=1.0,
            target_y=1.0,
        )
