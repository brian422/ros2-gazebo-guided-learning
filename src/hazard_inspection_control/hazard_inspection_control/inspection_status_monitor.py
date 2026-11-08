"""Receive and report inspection status messages for the Module 1 lesson."""

from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


STATUS_TOPIC = '/training/inspection_status'
SUBSCRIPTION_QUEUE_DEPTH = 10


class InspectionStatusMonitor(Node):
    """Count and log each training inspection status message received."""

    def __init__(self) -> None:
        """Initialize the subscriber and receipt counter."""
        super().__init__('inspection_status_monitor')

        self._received_count = 0
        self._subscription = self.create_subscription(
            String,
            STATUS_TOPIC,
            self._status_callback,
            SUBSCRIPTION_QUEUE_DEPTH,
        )

        self.get_logger().info(
            f'Waiting for messages on {STATUS_TOPIC}'
        )

    def _status_callback(self, message: String) -> None:
        """Log one received status message and its receipt number."""
        self._received_count += 1
        self.get_logger().info(
            f'Received #{self._received_count}: "{message.data}"'
        )


def main(args: Optional[List[str]] = None) -> None:
    """Run the inspection status monitor until interrupted."""
    rclpy.init(args=args)
    node: Optional[InspectionStatusMonitor] = None

    try:
        node = InspectionStatusMonitor()
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

