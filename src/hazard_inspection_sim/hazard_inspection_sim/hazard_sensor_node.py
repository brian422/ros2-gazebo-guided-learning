"""Simulate a position-dependent environmental hazard measurement."""

import math
import random
from typing import Dict, List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import Float64

from hazard_inspection_sim.hazard_math import (
    calculate_hazard,
    is_valid_position,
)


ODOM_TOPIC = '/odom'
HAZARD_TOPIC = '/hazard/level'
WARNING_THROTTLE_SECONDS = 5.0


class HazardSensorNode(Node):
    """Publish a reproducible mathematical hazard value from rover odometry."""

    def __init__(self) -> None:
        """Initialize parameters, interfaces, timer, and random generator."""
        super().__init__('hazard_sensor_node')

        self._declare_parameters()
        self._validate_initial_parameters()
        self.add_on_set_parameters_callback(
            self._validate_parameter_updates
        )

        random_seed = int(self.get_parameter('random_seed').value)
        publish_frequency = float(
            self.get_parameter('publish_frequency').value
        )
        self._random = random.Random(random_seed)

        self._latest_position: Optional[Tuple[float, float]] = None
        self._last_odom_receipt: Optional[Time] = None
        self._last_warning_times: Dict[str, Time] = {}

        self._hazard_publisher = self.create_publisher(
            Float64,
            HAZARD_TOPIC,
            10,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self._odom_callback,
            10,
        )
        self._publish_timer = self.create_timer(
            1.0 / publish_frequency,
            self._publish_hazard,
        )

        self.get_logger().info(
            'Hazard sensor ready: odom=%s, output=%s, seed=%d, rate=%.2f Hz'
            % (
                ODOM_TOPIC,
                HAZARD_TOPIC,
                random_seed,
                publish_frequency,
            )
        )

    def _declare_parameters(self) -> None:
        """Declare all supported configuration with descriptions."""
        self.declare_parameter(
            'baseline',
            0.20,
            ParameterDescriptor(
                description='Hazard level far from the hazard zone.'
            ),
        )
        self.declare_parameter(
            'peak',
            10.0,
            ParameterDescriptor(
                description='Hazard increase at the centre of the zone.'
            ),
        )
        self.declare_parameter(
            'hazard_zone_x',
            3.50,
            ParameterDescriptor(
                description='Hazard-zone centre x coordinate in odom, metres.'
            ),
        )
        self.declare_parameter(
            'hazard_zone_y',
            1.30,
            ParameterDescriptor(
                description='Hazard-zone centre y coordinate in odom, metres.'
            ),
        )
        self.declare_parameter(
            'spread',
            1.20,
            ParameterDescriptor(
                description='Gaussian spread parameter in square metres.'
            ),
        )
        self.declare_parameter(
            'noise_amplitude',
            0.05,
            ParameterDescriptor(
                description='Uniform noise bound added to each measurement.'
            ),
        )
        self.declare_parameter(
            'random_seed',
            42,
            ParameterDescriptor(
                description='Read-only seed for reproducible random noise.',
                read_only=True,
            ),
        )
        self.declare_parameter(
            'publish_frequency',
            10.0,
            ParameterDescriptor(
                description='Read-only hazard publication rate in hertz.',
                read_only=True,
            ),
        )
        self.declare_parameter(
            'odom_timeout',
            1.0,
            ParameterDescriptor(
                description='Maximum odometry age before publishing NaN.'
            ),
        )

    def _validate_initial_parameters(self) -> None:
        """Raise a clear error when startup parameters are invalid."""
        random_seed = self.get_parameter('random_seed').value
        if isinstance(random_seed, bool) or not isinstance(random_seed, int):
            raise ValueError('random_seed must be an integer')

        values = {
            name: self.get_parameter(name).value
            for name in (
                'baseline',
                'peak',
                'hazard_zone_x',
                'hazard_zone_y',
                'spread',
                'noise_amplitude',
                'publish_frequency',
                'odom_timeout',
            )
        }

        for name, value in values.items():
            if not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f'{name} must be a finite number')

        if values['baseline'] < 0.0:
            raise ValueError('baseline must be nonnegative')
        if values['peak'] < 0.0:
            raise ValueError('peak must be nonnegative')
        if values['spread'] <= 0.0:
            raise ValueError('spread must be greater than zero')
        if values['noise_amplitude'] < 0.0:
            raise ValueError('noise_amplitude must be nonnegative')
        if values['publish_frequency'] <= 0.0:
            raise ValueError('publish_frequency must be greater than zero')
        if values['odom_timeout'] <= 0.0:
            raise ValueError('odom_timeout must be greater than zero')

    def _validate_parameter_updates(
        self,
        parameters: List[Parameter],
    ) -> SetParametersResult:
        """Validate proposed dynamic updates without causing side effects."""
        candidate_values = {
            parameter.name: parameter.value
            for parameter in parameters
        }

        finite_names = {
            'baseline',
            'peak',
            'hazard_zone_x',
            'hazard_zone_y',
            'spread',
            'noise_amplitude',
            'odom_timeout',
        }
        for name in finite_names.intersection(candidate_values):
            value = candidate_values[name]
            if not isinstance(value, (float, int)) or not math.isfinite(value):
                return SetParametersResult(
                    successful=False,
                    reason=f'{name} must be a finite number',
                )

        for name in ('baseline', 'peak', 'noise_amplitude'):
            if name in candidate_values and candidate_values[name] < 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f'{name} must be nonnegative',
                )

        for name in ('spread', 'odom_timeout'):
            if name in candidate_values and candidate_values[name] <= 0.0:
                return SetParametersResult(
                    successful=False,
                    reason=f'{name} must be greater than zero',
                )

        return SetParametersResult(successful=True)

    def _odom_callback(self, message: Odometry) -> None:
        """Store the latest valid planar position and its receipt time."""
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        self._last_odom_receipt = self.get_clock().now()

        if not is_valid_position(x, y):
            self._latest_position = None
            self._warn_throttled(
                'invalid_odom',
                'Received invalid odometry position; publishing NaN hazard.',
            )
            return

        self._latest_position = (x, y)

    def _publish_hazard(self) -> None:
        """Publish a valid hazard value or NaN for missing/stale odometry."""
        if self._latest_position is None or self._last_odom_receipt is None:
            self._publish_invalid_measurement('odometry is unavailable')
            return

        now = self.get_clock().now()
        odom_age = max(
            0.0,
            (now - self._last_odom_receipt).nanoseconds / 1.0e9,
        )
        odom_timeout = float(self.get_parameter('odom_timeout').value)
        if odom_age > odom_timeout:
            self._publish_invalid_measurement(
                'odometry is stale (age %.2f s, limit %.2f s)'
                % (odom_age, odom_timeout)
            )
            return

        x, y = self._latest_position
        noise_amplitude = float(
            self.get_parameter('noise_amplitude').value
        )
        noise_offset = self._random.uniform(
            -noise_amplitude,
            noise_amplitude,
        )

        hazard_value = calculate_hazard(
            x=x,
            y=y,
            zone_x=float(self.get_parameter('hazard_zone_x').value),
            zone_y=float(self.get_parameter('hazard_zone_y').value),
            baseline=float(self.get_parameter('baseline').value),
            peak=float(self.get_parameter('peak').value),
            spread=float(self.get_parameter('spread').value),
            noise_offset=noise_offset,
        )

        message = Float64()
        message.data = hazard_value
        self._hazard_publisher.publish(message)

    def _publish_invalid_measurement(self, reason: str) -> None:
        """Publish NaN so downstream safety logic can detect invalid data."""
        message = Float64()
        message.data = math.nan
        self._hazard_publisher.publish(message)
        self._warn_throttled(
            'invalid_measurement',
            f'Hazard measurement invalid because {reason}.',
        )

    def _warn_throttled(self, key: str, message: str) -> None:
        """Log a warning at most once per configured throttle interval."""
        now = self.get_clock().now()
        previous = self._last_warning_times.get(key)
        if previous is not None:
            elapsed = max(0.0, (now - previous).nanoseconds / 1.0e9)
            if elapsed < WARNING_THROTTLE_SECONDS:
                return

        self._last_warning_times[key] = now
        self.get_logger().warning(message)


def main(args: Optional[List[str]] = None) -> None:
    """Run the hazard sensor node until shutdown."""
    rclpy.init(args=args)
    node: Optional[HazardSensorNode] = None
    try:
        node = HazardSensorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
