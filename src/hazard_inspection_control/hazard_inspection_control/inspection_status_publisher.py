"""Publish a simple inspection status for the Module 1 ROS 2 lesson."""

from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


STATUS_TOPIC = '/training/inspection_status'
DEFAULT_STATION_NAME = 'training_station'
DEFAULT_PUBLISH_PERIOD_S = 1.0
PUBLISHER_QUEUE_DEPTH = 10


class InspectionStatusPublisher(Node):
    """Publish a numbered training status message at a validated interval."""

    def __init__(self) -> None:
        """Initialize parameters, publisher, and periodic timer."""
        super().__init__('inspection_status_publisher')

        self.declare_parameter('station_name', DEFAULT_STATION_NAME)
        self.declare_parameter(
            'publish_period_s',
            DEFAULT_PUBLISH_PERIOD_S,
        )

        self._station_name, publish_period_s = (
            self._read_and_validate_parameters()
        )
        self._sequence = 0
        self._publisher = self.create_publisher(
            String,
            STATUS_TOPIC,
            PUBLISHER_QUEUE_DEPTH,
        )
        self._timer = self.create_timer(
            publish_period_s,
            self._publish_status,
        )

        self.get_logger().info(
            f'Publishing {STATUS_TOPIC} every {publish_period_s:.2f} s '
            f'for station "{self._station_name}"'
        )

    def _read_and_validate_parameters(self) -> tuple[str, float]:
        """Return validated startup parameters or raise a clear error."""
        station_name_value = self.get_parameter('station_name').value
        publish_period_value = self.get_parameter('publish_period_s').value

        if not isinstance(station_name_value, str):
            raise TypeError('station_name must be a string')

        station_name = station_name_value.strip()
        if not station_name:
            raise ValueError('station_name must not be empty')

        if isinstance(publish_period_value, bool) or not isinstance(
            publish_period_value,
            (int, float),
        ):
            raise TypeError('publish_period_s must be a number')

        publish_period_s = float(publish_period_value)
        if publish_period_s <= 0.0:
            raise ValueError('publish_period_s must be greater than zero')

        return station_name, publish_period_s

    def _publish_status(self) -> None:
        """Create and publish the next status message."""
        self._sequence += 1

        message = String()
        message.data = (
            f'READY | station={self._station_name} '
            f'| sequence={self._sequence}'
        )
        self._publisher.publish(message)
        self.get_logger().info(f'Published: "{message.data}"')


def main(args: Optional[List[str]] = None) -> None:
    """Run the inspection status publisher until interrupted."""
    rclpy.init(args=args)
    node: Optional[InspectionStatusPublisher] = None

    try:
        node = InspectionStatusPublisher()
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

