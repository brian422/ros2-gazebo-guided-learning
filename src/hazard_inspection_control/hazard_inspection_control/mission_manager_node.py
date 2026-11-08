"""Expose start, reset, and abort mission services."""

from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


COMMAND_TOPIC = '/mission/command'


class MissionManagerNode(Node):
    """Translate operator service calls into mission command events."""

    def __init__(self) -> None:
        """Create command publisher and three standard Trigger services."""
        super().__init__('mission_manager_node')

        self._command_publisher = self.create_publisher(
            String,
            COMMAND_TOPIC,
            10,
        )
        self._start_service = self.create_service(
            Trigger,
            '/mission/start',
            self._start_callback,
        )
        self._reset_service = self.create_service(
            Trigger,
            '/mission/reset',
            self._reset_callback,
        )
        self._abort_service = self.create_service(
            Trigger,
            '/mission/abort',
            self._abort_callback,
        )

        self.get_logger().info(
            'Mission services ready: /mission/start, /mission/reset, '
            '/mission/abort'
        )

    def _start_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Publish a START command event."""
        return self._publish_command('START', response)

    def _reset_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Publish a RESET command event."""
        return self._publish_command('RESET', response)

    def _abort_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Publish an ABORT command event."""
        return self._publish_command('ABORT', response)

    def _publish_command(
        self,
        command: str,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Publish one command if a controller subscriber is discovered."""
        if self._command_publisher.get_subscription_count() == 0:
            response.success = False
            response.message = (
                'No mission controller is subscribed to /mission/command.'
            )
            self.get_logger().warning(response.message)
            return response

        message = String()
        message.data = command
        self._command_publisher.publish(message)

        response.success = True
        response.message = (
            f'{command} request published; inspect /mission/state '
            'for the controller decision.'
        )
        self.get_logger().info(response.message)
        return response


def main(args: Optional[List[str]] = None) -> None:
    """Run the mission manager until shutdown."""
    rclpy.init(args=args)
    node: Optional[MissionManagerNode] = None
    try:
        node = MissionManagerNode()
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
