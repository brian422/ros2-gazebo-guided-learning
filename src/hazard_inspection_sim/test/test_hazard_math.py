"""Unit tests for the deterministic hazard-model mathematics."""

import math

import pytest

from hazard_inspection_sim.hazard_math import (
    calculate_hazard,
    is_valid_position,
)


def test_hazard_is_maximum_at_zone_center() -> None:
    """The center value must equal baseline plus peak without noise."""
    value = calculate_hazard(
        x=2.0,
        y=3.0,
        zone_x=2.0,
        zone_y=3.0,
        baseline=0.2,
        peak=10.0,
        spread=1.5,
    )

    assert value == pytest.approx(10.2)


def test_hazard_approaches_baseline_far_from_zone() -> None:
    """A distant measurement should contain almost no peak contribution."""
    value = calculate_hazard(
        x=100.0,
        y=100.0,
        zone_x=0.0,
        zone_y=0.0,
        baseline=0.2,
        peak=10.0,
        spread=1.5,
    )

    assert value == pytest.approx(0.2)


def test_noise_is_added_and_result_is_never_negative() -> None:
    """Bounded sensor noise may lower a reading but not below zero."""
    noisy_value = calculate_hazard(
        x=0.0,
        y=0.0,
        zone_x=10.0,
        zone_y=10.0,
        baseline=0.2,
        peak=1.0,
        spread=1.0,
        noise_offset=-0.5,
    )

    assert noisy_value == 0.0


@pytest.mark.parametrize(
    ('x', 'y', 'expected'),
    [
        (0.0, 1.0, True),
        (math.nan, 1.0, False),
        (0.0, math.inf, False),
    ],
)
def test_position_validation(x: float, y: float, expected: bool) -> None:
    """Non-finite odometry coordinates must be rejected."""
    assert is_valid_position(x, y) is expected


@pytest.mark.parametrize(
    ('spread', 'peak'),
    [
        (0.0, 1.0),
        (-1.0, 1.0),
        (1.0, -1.0),
        (math.nan, 1.0),
    ],
)
def test_invalid_model_parameters_raise(
    spread: float,
    peak: float,
) -> None:
    """Invalid configuration must fail clearly instead of hiding bad data."""
    with pytest.raises(ValueError):
        calculate_hazard(
            x=0.0,
            y=0.0,
            zone_x=1.0,
            zone_y=1.0,
            baseline=0.2,
            peak=peak,
            spread=spread,
        )
