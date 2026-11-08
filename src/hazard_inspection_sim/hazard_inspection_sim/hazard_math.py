"""Pure mathematical helpers for the simulated environmental hazard."""

import math


def calculate_hazard(
    x: float,
    y: float,
    zone_x: float,
    zone_y: float,
    baseline: float,
    peak: float,
    spread: float,
    noise_offset: float = 0.0,
) -> float:
    """Return a nonnegative Gaussian-shaped hazard value.

    The deterministic part is:

        baseline + peak * exp(-distance_squared / spread)

    ``spread`` therefore has units of square metres. ``noise_offset`` is
    supplied by the ROS node so this function remains deterministic and easy
    to unit test.

    Raises:
        ValueError: If an input is non-finite or a model parameter is invalid.
    """
    values = (
        x,
        y,
        zone_x,
        zone_y,
        baseline,
        peak,
        spread,
        noise_offset,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('hazard calculation inputs must be finite')
    if baseline < 0.0:
        raise ValueError('baseline must be nonnegative')
    if peak < 0.0:
        raise ValueError('peak must be nonnegative')
    if spread <= 0.0:
        raise ValueError('spread must be greater than zero')

    distance_squared = (x - zone_x) ** 2 + (y - zone_y) ** 2
    deterministic_value = baseline + peak * math.exp(
        -distance_squared / spread
    )

    # Environmental concentrations cannot be negative in this simplified model.
    return max(0.0, deterministic_value + noise_offset)


def is_valid_position(x: float, y: float) -> bool:
    """Return True only when both planar coordinates are finite."""
    return math.isfinite(x) and math.isfinite(y)
