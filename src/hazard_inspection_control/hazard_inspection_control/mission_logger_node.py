"""Record synchronized snapshots of mission topics to a CSV file."""

import csv
from datetime import datetime, timezone
import math
from pathlib import Path
from typing import IO, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from hazard_inspection_interfaces.msg import MissionState
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float64

from hazard_inspection_control.control_math import quaternion_to_yaw


CSV_COLUMNS = (
    'time',
    'x',
    'y',
    'yaw',
    'linear_velocity',
    'angular_velocity',
    'hazard_level',
    'mission_state',
)


class MissionLoggerNode(Node):
    """Sample the latest mission messages and write bounded-rate CSV rows."""

    def __init__(self) -> None:
        """Declare configuration, open the CSV, and create ROS interfaces."""
        super().__init__('mission_logger_node')

        self.declare_parameter(
            'output_directory',
            '~/.ros/hazard_inspection_rover/logs',
            ParameterDescriptor(
                description=(
                    'Read-only directory for timestamped mission CSV files.'
                ),
                read_only=True,
            ),
        )
        self.declare_parameter(
            'file_prefix',
            'mission',
            ParameterDescriptor(
                description='Read-only prefix before the UTC timestamp.',
                read_only=True,
            ),
        )
        self.declare_parameter(
            'logging_frequency',
            10.0,
            ParameterDescriptor(
                description='Read-only CSV sampling rate in hertz.',
                read_only=True,
            ),
        )
        self.declare_parameter(
            'flush_every_rows',
            10,
            ParameterDescriptor(
                description='Rows written between explicit file flushes.'
            ),
        )
        self._validate_parameters()
        self.add_on_set_parameters_callback(
            self._validate_parameter_updates
        )

        nan_value = math.nan
        self._pose: Tuple[float, float, float] = (
            nan_value,
            nan_value,
            nan_value,
        )
        self._velocity: Tuple[float, float] = (0.0, 0.0)
        self._hazard_level = nan_value
        self._mission_state = 'UNKNOWN'
        self._rows_since_flush = 0
        self._file: Optional[IO[str]] = None
        self._writer: Optional[csv.DictWriter] = None
        self._output_path: Optional[Path] = None

        self._open_output_file()

        retained_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10,
        )
        self._hazard_subscription = self.create_subscription(
            Float64,
            '/hazard/level',
            self._hazard_callback,
            10,
        )
        self._state_subscription = self.create_subscription(
            MissionState,
            '/mission/state',
            self._state_callback,
            retained_qos,
        )
        self._velocity_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._velocity_callback,
            10,
        )

        logging_frequency = float(
            self.get_parameter('logging_frequency').value
        )
        self._logging_timer = self.create_timer(
            1.0 / logging_frequency,
            self._write_row,
        )

    def _validate_parameters(self) -> None:
        """Raise clear startup errors for invalid logger configuration."""
        output_directory = self.get_parameter('output_directory').value
        file_prefix = self.get_parameter('file_prefix').value
        frequency = self.get_parameter('logging_frequency').value
        flush_rows = self.get_parameter('flush_every_rows').value

        if not isinstance(output_directory, str) or not output_directory:
            raise ValueError('output_directory must be a nonempty string')
        if not isinstance(file_prefix, str) or not file_prefix:
            raise ValueError('file_prefix must be a nonempty string')
        if '/' in file_prefix or '\\' in file_prefix:
            raise ValueError('file_prefix must not contain path separators')
        if (
            isinstance(frequency, bool)
            or not isinstance(frequency, (float, int))
            or not math.isfinite(frequency)
            or frequency <= 0.0
        ):
            raise ValueError('logging_frequency must be finite and positive')
        if (
            isinstance(flush_rows, bool)
            or not isinstance(flush_rows, int)
            or flush_rows <= 0
        ):
            raise ValueError('flush_every_rows must be a positive integer')

    def _validate_parameter_updates(
        self,
        parameters: List[Parameter],
    ) -> SetParametersResult:
        """Validate the one dynamically adjustable logger parameter."""
        for parameter in parameters:
            if parameter.name != 'flush_every_rows':
                continue
            value = parameter.value
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or value <= 0
            ):
                return SetParametersResult(
                    successful=False,
                    reason='flush_every_rows must be a positive integer',
                )
        return SetParametersResult(successful=True)

    def _open_output_file(self) -> None:
        """Create the output directory and timestamped CSV file."""
        directory_text = str(
            self.get_parameter('output_directory').value
        )
        file_prefix = str(self.get_parameter('file_prefix').value)
        output_directory = Path(directory_text).expanduser()
        timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        output_path = output_directory / f'{file_prefix}_{timestamp}.csv'

        try:
            output_directory.mkdir(parents=True, exist_ok=True)
            self._file = output_path.open(
                mode='w',
                encoding='utf-8',
                newline='',
            )
            self._writer = csv.DictWriter(
                self._file,
                fieldnames=CSV_COLUMNS,
            )
            self._writer.writeheader()
            self._file.flush()
        except OSError as error:
            self._file = None
            self._writer = None
            self.get_logger().error(
                f'CSV logging disabled; cannot create {output_path}: {error}'
            )
            return

        self._output_path = output_path
        self.get_logger().info(f'Mission CSV log: {output_path}')

    def _odom_callback(self, message: Odometry) -> None:
        """Store x, y, and yaw from the most recent odometry message."""
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        try:
            x_value = float(position.x)
            y_value = float(position.y)
            if not all(math.isfinite(value) for value in (x_value, y_value)):
                raise ValueError('odometry position must be finite')
            yaw = quaternion_to_yaw(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            )
        except ValueError:
            self._pose = (math.nan, math.nan, math.nan)
            return
        self._pose = (x_value, y_value, yaw)

    def _hazard_callback(self, message: Float64) -> None:
        """Store the latest hazard value, including NaN if supplied."""
        self._hazard_level = float(message.data)

    def _state_callback(self, message: MissionState) -> None:
        """Store the latest human-readable mission state."""
        self._mission_state = message.state_name

    def _velocity_callback(self, message: Twist) -> None:
        """Store the latest commanded planar velocity."""
        self._velocity = (
            float(message.linear.x),
            float(message.angular.z),
        )

    def _write_row(self) -> None:
        """Write one snapshot row if file creation succeeded."""
        if self._writer is None or self._file is None:
            return

        x_value, y_value, yaw = self._pose
        linear, angular = self._velocity
        time_seconds = self.get_clock().now().nanoseconds / 1.0e9
        row = {
            'time': f'{time_seconds:.6f}',
            'x': self._format_float(x_value),
            'y': self._format_float(y_value),
            'yaw': self._format_float(yaw),
            'linear_velocity': self._format_float(linear),
            'angular_velocity': self._format_float(angular),
            'hazard_level': self._format_float(self._hazard_level),
            'mission_state': self._mission_state,
        }

        try:
            self._writer.writerow(row)
            self._rows_since_flush += 1
            flush_limit = int(
                self.get_parameter('flush_every_rows').value
            )
            if self._rows_since_flush >= flush_limit:
                self._file.flush()
                self._rows_since_flush = 0
        except OSError as error:
            self.get_logger().error(
                f'CSV write failed; logging stopped: {error}'
            )
            self.close()

    @staticmethod
    def _format_float(value: float) -> str:
        """Format finite values consistently and invalid values as nan."""
        return f'{value:.6f}' if math.isfinite(value) else 'nan'

    def close(self) -> None:
        """Flush and close the output file safely and idempotently."""
        if self._file is None:
            return
        try:
            self._file.flush()
            self._file.close()
        except OSError as error:
            self.get_logger().error(f'Error while closing CSV file: {error}')
        finally:
            self._file = None
            self._writer = None


def main(args: Optional[List[str]] = None) -> None:
    """Run the mission logger until shutdown."""
    rclpy.init(args=args)
    node: Optional[MissionLoggerNode] = None
    try:
        node = MissionLoggerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
