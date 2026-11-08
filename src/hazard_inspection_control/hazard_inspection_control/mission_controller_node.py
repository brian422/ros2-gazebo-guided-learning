"""Run the educational waypoint mission and hazard retreat behavior."""

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from hazard_inspection_interfaces.msg import MissionState
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker

from hazard_inspection_control.control_math import (
    calculate_waypoint_error,
    quaternion_to_yaw,
    saturate_velocity,
)
from hazard_inspection_control.mission_state_machine import (
    MissionStateCode,
    MissionStateMachine,
    TransitionResult,
)


ODOM_TOPIC = '/odom'
HAZARD_TOPIC = '/hazard/level'
COMMAND_TOPIC = '/mission/command'
STATE_TOPIC = '/mission/state'
COMMAND_VELOCITY_TOPIC = '/cmd_vel'
PATH_TOPIC = '/mission/path'
STATUS_MARKER_TOPIC = '/mission/status_marker'
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'

Pose2D = Tuple[float, float, float]
Point2D = Tuple[float, float]


class MissionControllerNode(Node):
    """Follow a known route with simple proportional feedback and retreat.

    This controller is intentionally educational. It has no map search,
    obstacle avoidance, localization correction, Nav2, or safety certification.
    """

    def __init__(self) -> None:
        """Create parameters, ROS interfaces, state, and control timers."""
        super().__init__('mission_controller_node')

        self._declare_parameters()
        self._validate_initial_parameters()
        self._waypoints, self._waypoint_types = self._load_route()
        self.add_on_set_parameters_callback(
            self._validate_parameter_updates
        )

        self._state_machine = MissionStateMachine()
        self._latest_pose: Optional[Pose2D] = None
        self._last_odom_receipt: Optional[Time] = None
        self._latest_hazard: Optional[float] = None
        self._last_hazard_receipt: Optional[Time] = None
        self._inspection_started_at: Optional[Time] = None
        self._hazard_detected_at: Optional[Time] = None
        self._mission_start_pose: Optional[Point2D] = None
        self._last_safe_pose: Optional[Point2D] = None

        retained_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._velocity_publisher = self.create_publisher(
            Twist,
            COMMAND_VELOCITY_TOPIC,
            10,
        )
        self._state_publisher = self.create_publisher(
            MissionState,
            STATE_TOPIC,
            retained_qos,
        )
        self._path_publisher = self.create_publisher(
            Path,
            PATH_TOPIC,
            retained_qos,
        )
        self._marker_publisher = self.create_publisher(
            Marker,
            STATUS_MARKER_TOPIC,
            retained_qos,
        )

        self._odom_subscription = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self._odom_callback,
            10,
        )
        self._hazard_subscription = self.create_subscription(
            Float64,
            HAZARD_TOPIC,
            self._hazard_callback,
            10,
        )
        self._command_subscription = self.create_subscription(
            String,
            COMMAND_TOPIC,
            self._command_callback,
            10,
        )

        control_frequency = float(
            self.get_parameter('control_frequency').value
        )
        status_frequency = float(
            self.get_parameter('status_publish_frequency').value
        )
        self._control_timer = self.create_timer(
            1.0 / control_frequency,
            self._control_step,
        )
        self._status_timer = self.create_timer(
            1.0 / status_frequency,
            self._publish_status_bundle,
        )

        self._publish_zero_velocity()
        self._publish_status_bundle()
        if bool(self.get_parameter('enable_motion').value):
            self.get_logger().info(
                'Educational waypoint controller ready with %d route points.'
                % len(self._waypoints)
            )
        else:
            self.get_logger().info(
                'Mission controller state scaffold ready; motion disabled.'
            )

    def _declare_parameters(self) -> None:
        """Declare route, control, timing, and safety configuration."""
        route_descriptor = ParameterDescriptor(
            description='Read-only route definition in the odom frame.',
            read_only=True,
        )
        self.declare_parameter(
            'waypoints_x',
            [-1.60, 2.40, 3.50],
            route_descriptor,
        )
        self.declare_parameter(
            'waypoints_y',
            [-1.50, -1.50, 1.30],
            route_descriptor,
        )
        self.declare_parameter(
            'waypoint_types',
            ['inspection', 'transit', 'inspection'],
            route_descriptor,
        )
        self.declare_parameter(
            'enable_motion',
            True,
            ParameterDescriptor(
                description=(
                    'Read-only switch preserving the nonmoving Module 4 lab.'
                ),
                read_only=True,
            ),
        )

        descriptions = {
            'max_linear_speed': 'Maximum forward speed in metres per second.',
            'max_angular_speed': 'Maximum yaw rate in radians per second.',
            'retreat_linear_speed': 'Forward speed limit during retreat.',
            'linear_gain': 'Proportional gain from distance to speed.',
            'angular_gain': (
                'Proportional gain from heading error to yaw rate.'
            ),
            'position_tolerance': 'Distance for declaring waypoint arrival.',
            'heading_alignment_tolerance': (
                'Heading error below which forward motion is allowed.'
            ),
            'inspection_duration': 'Stopped inspection time in seconds.',
            'hazard_threshold': (
                'Hazard value that triggers an immediate stop.'
            ),
            'hazard_timeout': 'Maximum hazard-data age during a mission.',
            'odom_timeout': 'Maximum odometry age during a mission.',
            'hazard_stop_duration': (
                'Visible stopped time before retreat begins.'
            ),
        }
        defaults = {
            'max_linear_speed': 0.35,
            'max_angular_speed': 1.20,
            'retreat_linear_speed': 0.25,
            'linear_gain': 0.80,
            'angular_gain': 2.00,
            'position_tolerance': 0.12,
            'heading_alignment_tolerance': 0.35,
            'inspection_duration': 2.00,
            'hazard_threshold': 6.00,
            'hazard_timeout': 1.00,
            'odom_timeout': 0.50,
            'hazard_stop_duration': 0.50,
        }
        for name, default in defaults.items():
            self.declare_parameter(
                name,
                default,
                ParameterDescriptor(description=descriptions[name]),
            )

        self.declare_parameter(
            'control_frequency',
            20.0,
            ParameterDescriptor(
                description='Read-only controller update rate in hertz.',
                read_only=True,
            ),
        )
        self.declare_parameter(
            'status_publish_frequency',
            2.0,
            ParameterDescriptor(
                description='Read-only status update rate in hertz.',
                read_only=True,
            ),
        )

    def _validate_initial_parameters(self) -> None:
        """Raise a clear startup error for invalid configuration."""
        enable_motion = self.get_parameter('enable_motion').value
        if not isinstance(enable_motion, bool):
            raise ValueError('enable_motion must be a Boolean')

        positive_names = {
            'max_linear_speed',
            'max_angular_speed',
            'retreat_linear_speed',
            'linear_gain',
            'angular_gain',
            'position_tolerance',
            'heading_alignment_tolerance',
            'inspection_duration',
            'hazard_threshold',
            'hazard_timeout',
            'odom_timeout',
            'control_frequency',
            'status_publish_frequency',
        }
        for name in positive_names:
            value = self.get_parameter(name).value
            if not self._is_finite_number(value) or value <= 0.0:
                raise ValueError(f'{name} must be a finite positive number')

        stop_duration = self.get_parameter('hazard_stop_duration').value
        if not self._is_finite_number(stop_duration):
            raise ValueError('hazard_stop_duration must be finite')
        if stop_duration < 0.0:
            raise ValueError('hazard_stop_duration must be nonnegative')

        heading_limit = float(
            self.get_parameter('heading_alignment_tolerance').value
        )
        if heading_limit > math.pi:
            raise ValueError(
                'heading_alignment_tolerance must not exceed pi'
            )

        self._load_route()

    def _load_route(self) -> Tuple[List[Point2D], List[str]]:
        """Read and validate the fixed route parameter arrays."""
        x_values = list(self.get_parameter('waypoints_x').value)
        y_values = list(self.get_parameter('waypoints_y').value)
        type_values = list(self.get_parameter('waypoint_types').value)

        if not x_values:
            raise ValueError('at least one waypoint is required')
        if not (len(x_values) == len(y_values) == len(type_values)):
            raise ValueError(
                'waypoint parameter arrays must have equal length'
            )
        if not all(
            self._is_finite_number(value)
            for value in x_values + y_values
        ):
            raise ValueError('all waypoint coordinates must be finite')

        normalized_types = [str(value).lower() for value in type_values]
        allowed_types = {'inspection', 'transit'}
        if not all(value in allowed_types for value in normalized_types):
            raise ValueError(
                'waypoint types must be inspection or transit'
            )

        waypoints = [
            (float(x_value), float(y_value))
            for x_value, y_value in zip(x_values, y_values)
        ]
        return waypoints, normalized_types

    @staticmethod
    def _is_finite_number(value: object) -> bool:
        """Return True for a finite real number but not a Boolean."""
        return (
            not isinstance(value, bool)
            and isinstance(value, (float, int))
            and math.isfinite(value)
        )

    def _validate_parameter_updates(
        self,
        parameters: List[Parameter],
    ) -> SetParametersResult:
        """Reject unsafe dynamic values without applying side effects."""
        positive_names = {
            'max_linear_speed',
            'max_angular_speed',
            'retreat_linear_speed',
            'linear_gain',
            'angular_gain',
            'position_tolerance',
            'heading_alignment_tolerance',
            'inspection_duration',
            'hazard_threshold',
            'hazard_timeout',
            'odom_timeout',
        }
        for parameter in parameters:
            if parameter.name in positive_names:
                value = parameter.value
                if not self._is_finite_number(value) or value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            f'{parameter.name} must be a finite '
                            'positive number'
                        ),
                    )
                if (
                    parameter.name == 'heading_alignment_tolerance'
                    and value > math.pi
                ):
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            'heading_alignment_tolerance must not exceed pi'
                        ),
                    )
            elif parameter.name == 'hazard_stop_duration':
                value = parameter.value
                if not self._is_finite_number(value) or value < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason=(
                            'hazard_stop_duration must be finite and '
                            'nonnegative'
                        ),
                    )
        return SetParametersResult(successful=True)

    def _odom_callback(self, message: Odometry) -> None:
        """Store a finite planar pose or abort an active mission."""
        self._last_odom_receipt = self.get_clock().now()
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        try:
            yaw = quaternion_to_yaw(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            )
            x_value = float(position.x)
            y_value = float(position.y)
            if not all(math.isfinite(value) for value in (x_value, y_value)):
                raise ValueError('odometry position must be finite')
        except ValueError as error:
            self._latest_pose = None
            self._abort_active_mission(
                f'Mission aborted because odometry is invalid: {error}'
            )
            return

        self._latest_pose = (x_value, y_value, yaw)

    def _hazard_callback(self, message: Float64) -> None:
        """Store hazard data and stop immediately at the threshold."""
        self._last_hazard_receipt = self.get_clock().now()
        hazard_value = float(message.data)
        if not math.isfinite(hazard_value) or hazard_value < 0.0:
            self._latest_hazard = None
            self._abort_active_mission(
                'Mission aborted because hazard data is invalid.'
            )
            return

        self._latest_hazard = hazard_value
        threshold = float(self.get_parameter('hazard_threshold').value)
        if (
            hazard_value >= threshold
            and self._state_machine.state
            in {
                MissionStateCode.MOVING_TO_WAYPOINT,
                MissionStateCode.INSPECTING,
            }
        ):
            self._publish_zero_velocity()
            self._hazard_detected_at = self.get_clock().now()
            result = self._state_machine.report_hazard(hazard_value)
            self._handle_result(result)

    def _command_callback(self, message: String) -> None:
        """Apply START, RESET, or ABORT from the mission manager."""
        command = message.data.strip().upper()
        if command == 'START':
            self._handle_start_command()
        elif command == 'RESET':
            self._publish_zero_velocity()
            self._clear_mission_runtime()
            self._handle_result(self._state_machine.request_reset())
        elif command == 'ABORT':
            self._publish_zero_velocity()
            self._handle_result(self._state_machine.request_abort())
        else:
            self.get_logger().warning(
                f'Ignoring unknown mission command: {message.data!r}'
            )

    def _handle_start_command(self) -> None:
        """Accept START only when pose and hazard data are safe."""
        if not self._odom_is_fresh() or not self._hazard_is_fresh():
            reason = (
                'START rejected because fresh odometry and hazard data '
                'are required.'
            )
            self.get_logger().warning(reason)
            self._publish_state(detail_override=reason)
            return

        threshold = float(self.get_parameter('hazard_threshold').value)
        if self._latest_hazard is None or self._latest_hazard >= threshold:
            reason = 'START rejected because the current hazard is unsafe.'
            self.get_logger().warning(reason)
            self._publish_state(detail_override=reason)
            return

        if self._latest_pose is None:
            return

        result = self._state_machine.request_start()
        if result.accepted:
            x_value, y_value, _yaw = self._latest_pose
            self._mission_start_pose = (x_value, y_value)
            self._last_safe_pose = self._mission_start_pose
            self._inspection_started_at = None
            self._hazard_detected_at = None
        self._handle_result(result)
        self._publish_path()

    def _control_step(self) -> None:
        """Run one state-dependent feedback and safety update."""
        state = self._state_machine.state
        if not self._state_machine.is_active:
            self._publish_zero_velocity()
            return

        if not self._odom_is_fresh():
            self._abort_active_mission(
                'Mission aborted because odometry is missing or stale.'
            )
            return
        if not self._hazard_is_fresh():
            self._abort_active_mission(
                'Mission aborted because hazard data is missing or stale.'
            )
            return

        if not bool(self.get_parameter('enable_motion').value):
            self._publish_zero_velocity()
            return

        if state == MissionStateCode.MOVING_TO_WAYPOINT:
            self._drive_to_active_waypoint()
        elif state == MissionStateCode.INSPECTING:
            self._update_inspection()
        elif state == MissionStateCode.HAZARD_DETECTED:
            self._update_hazard_stop()
        elif state == MissionStateCode.RETREATING:
            self._drive_to_safe_pose()
        else:
            self._publish_zero_velocity()

    def _drive_to_active_waypoint(self) -> None:
        """Command the current route point or process its arrival."""
        waypoint_index = self._state_machine.waypoint_index
        if waypoint_index >= len(self._waypoints):
            self._abort_active_mission(
                'Mission aborted because waypoint index is out of range.'
            )
            return

        target = self._waypoints[waypoint_index]
        if self._target_reached(target):
            self._publish_zero_velocity()
            self._last_safe_pose = target
            more_waypoints = waypoint_index < len(self._waypoints) - 1
            if self._waypoint_types[waypoint_index] == 'inspection':
                self._inspection_started_at = self.get_clock().now()
                result = self._state_machine.report_waypoint_reached()
            else:
                result = (
                    self._state_machine.report_transit_waypoint_reached(
                        more_waypoints
                    )
                )
            self._handle_result(result)
            return

        max_speed = float(self.get_parameter('max_linear_speed').value)
        self._drive_toward(target, max_speed)

    def _update_inspection(self) -> None:
        """Remain stopped until the configured inspection time elapses."""
        self._publish_zero_velocity()
        if self._inspection_started_at is None:
            self._inspection_started_at = self.get_clock().now()

        elapsed = self._elapsed_seconds(self._inspection_started_at)
        duration = float(self.get_parameter('inspection_duration').value)
        if elapsed < duration:
            return

        waypoint_index = self._state_machine.waypoint_index
        if waypoint_index < len(self._waypoints):
            self._last_safe_pose = self._waypoints[waypoint_index]
        more_waypoints = waypoint_index < len(self._waypoints) - 1
        self._inspection_started_at = None
        result = self._state_machine.report_inspection_complete(
            more_waypoints
        )
        self._handle_result(result)

    def _update_hazard_stop(self) -> None:
        """Hold zero velocity briefly, then enter RETREATING."""
        self._publish_zero_velocity()
        if self._hazard_detected_at is None:
            self._hazard_detected_at = self.get_clock().now()

        stop_duration = float(
            self.get_parameter('hazard_stop_duration').value
        )
        if self._elapsed_seconds(self._hazard_detected_at) < stop_duration:
            return
        if self._last_safe_pose is None:
            self._abort_active_mission(
                'Mission aborted because no safe retreat point exists.'
            )
            return

        result = self._state_machine.begin_retreat()
        self._handle_result(result)

    def _drive_to_safe_pose(self) -> None:
        """Turn and drive toward the last known safe route point."""
        if self._last_safe_pose is None:
            self._abort_active_mission(
                'Mission aborted because the retreat target is missing.'
            )
            return
        if self._target_reached(self._last_safe_pose):
            self._publish_zero_velocity()
            result = self._state_machine.report_retreat_complete()
            self._handle_result(result)
            return

        retreat_speed = float(
            self.get_parameter('retreat_linear_speed').value
        )
        self._drive_toward(self._last_safe_pose, retreat_speed)

    def _drive_toward(
        self,
        target: Point2D,
        linear_speed_limit: float,
    ) -> None:
        """Publish proportional rotate-then-translate velocity control."""
        if self._latest_pose is None:
            self._publish_zero_velocity()
            return

        x_value, y_value, yaw = self._latest_pose
        error = calculate_waypoint_error(
            x_value,
            y_value,
            yaw,
            target[0],
            target[1],
        )
        linear_gain = float(self.get_parameter('linear_gain').value)
        angular_gain = float(self.get_parameter('angular_gain').value)
        heading_tolerance = float(
            self.get_parameter('heading_alignment_tolerance').value
        )

        requested_linear = linear_gain * error.distance
        if abs(error.heading_error) > heading_tolerance:
            requested_linear = 0.0
        requested_angular = angular_gain * error.heading_error

        max_angular = float(
            self.get_parameter('max_angular_speed').value
        )
        linear, angular = saturate_velocity(
            requested_linear,
            requested_angular,
            linear_speed_limit,
            max_angular,
        )
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._velocity_publisher.publish(command)

    def _target_reached(self, target: Point2D) -> bool:
        """Return True when the current position is within tolerance."""
        if self._latest_pose is None:
            return False
        x_value, y_value, yaw = self._latest_pose
        error = calculate_waypoint_error(
            x_value,
            y_value,
            yaw,
            target[0],
            target[1],
        )
        tolerance = float(self.get_parameter('position_tolerance').value)
        return error.distance <= tolerance

    def _odom_is_fresh(self) -> bool:
        """Return True for a recent valid odometry pose."""
        if self._latest_pose is None or self._last_odom_receipt is None:
            return False
        timeout = float(self.get_parameter('odom_timeout').value)
        return self._elapsed_seconds(self._last_odom_receipt) <= timeout

    def _hazard_is_fresh(self) -> bool:
        """Return True for a recent finite nonnegative hazard value."""
        if self._latest_hazard is None:
            return False
        if self._last_hazard_receipt is None:
            return False
        timeout = float(self.get_parameter('hazard_timeout').value)
        return self._elapsed_seconds(self._last_hazard_receipt) <= timeout

    def _elapsed_seconds(self, start_time: Time) -> float:
        """Return nonnegative elapsed node-clock time."""
        elapsed = (self.get_clock().now() - start_time).nanoseconds / 1.0e9
        return max(0.0, elapsed)

    def _abort_active_mission(self, reason: str) -> None:
        """Publish zero and move an active mission to ABORTED."""
        self._publish_zero_velocity()
        if not self._state_machine.is_active:
            return
        result = self._state_machine.request_abort(reason)
        self._handle_result(result)

    def _clear_mission_runtime(self) -> None:
        """Clear timers and safe points when RESET is requested."""
        self._inspection_started_at = None
        self._hazard_detected_at = None
        self._mission_start_pose = None
        self._last_safe_pose = None

    def _handle_result(self, result: TransitionResult) -> None:
        """Log and publish one accepted or rejected state event."""
        if result.accepted:
            self.get_logger().info(
                '%s -> %s: %s'
                % (
                    result.previous_state.name,
                    result.current_state.name,
                    result.reason,
                )
            )
        else:
            self.get_logger().warning(
                'State event rejected in %s: %s'
                % (result.current_state.name, result.reason)
            )
        self._publish_status_bundle(detail_override=result.reason)

    def _publish_zero_velocity(self) -> None:
        """Publish an explicit zero command."""
        self._velocity_publisher.publish(Twist())

    def _publish_status_bundle(
        self,
        detail_override: Optional[str] = None,
    ) -> None:
        """Publish mission state, planned path, and RViz text marker."""
        self._publish_state(detail_override)
        self._publish_path()
        self._publish_status_marker()

    def _publish_state(self, detail_override: Optional[str] = None) -> None:
        """Publish the strongly typed mission state."""
        message = MissionState()
        message.stamp = self.get_clock().now().to_msg()
        message.state = int(self._state_machine.state)
        message.state_name = self._state_machine.state.name
        message.waypoint_index = self._state_machine.waypoint_index
        message.detail = (
            detail_override
            if detail_override is not None
            else self._state_machine.detail
        )
        self._state_publisher.publish(message)

    def _publish_path(self) -> None:
        """Publish the planned route for RViz2 visualization."""
        now_message = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = now_message
        path.header.frame_id = ODOM_FRAME

        points: List[Point2D] = []
        if self._mission_start_pose is not None:
            points.append(self._mission_start_pose)
        points.extend(self._waypoints)

        for x_value, y_value in points:
            pose = PoseStamped()
            pose.header.stamp = now_message
            pose.header.frame_id = ODOM_FRAME
            pose.pose.position.x = x_value
            pose.pose.position.y = y_value
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_publisher.publish(path)

    def _publish_status_marker(self) -> None:
        """Publish readable mission state and hazard text above the rover."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = BASE_FRAME
        marker.ns = 'mission_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 0.75
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.16
        marker.color.a = 1.0

        red, green, blue = self._state_color()
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue

        hazard_text = (
            'invalid'
            if self._latest_hazard is None
            else f'{self._latest_hazard:.2f}'
        )
        marker.text = (
            f'{self._state_machine.state.name}\n'
            f'Hazard: {hazard_text}'
        )
        self._marker_publisher.publish(marker)

    def _state_color(self) -> Tuple[float, float, float]:
        """Return a simple RGB status color for the RViz marker."""
        colors: Dict[MissionStateCode, Tuple[float, float, float]] = {
            MissionStateCode.IDLE: (0.75, 0.75, 0.75),
            MissionStateCode.MOVING_TO_WAYPOINT: (0.10, 0.75, 1.00),
            MissionStateCode.INSPECTING: (0.25, 1.00, 0.25),
            MissionStateCode.HAZARD_DETECTED: (1.00, 0.10, 0.05),
            MissionStateCode.RETREATING: (1.00, 0.60, 0.05),
            MissionStateCode.MISSION_COMPLETE: (0.20, 1.00, 0.75),
            MissionStateCode.ABORTED: (1.00, 0.20, 0.60),
        }
        return colors[self._state_machine.state]

    def stop(self) -> None:
        """Issue a final zero command before shutdown."""
        self._publish_zero_velocity()


def main(args: Optional[List[str]] = None) -> None:
    """Run the mission controller until shutdown."""
    rclpy.init(args=args)
    node: Optional[MissionControllerNode] = None
    try:
        node = MissionControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
