"""Launch the two ROS 2 communication nodes used in Module 1."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


PACKAGE_NAME = 'hazard_inspection_control'


def generate_launch_description() -> LaunchDescription:
    """Create the Module 1 publisher-and-monitor launch description."""
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    parameter_file = package_share / 'config' / 'module_01.yaml'

    status_publisher = Node(
        package=PACKAGE_NAME,
        executable='inspection_status_publisher',
        name='inspection_status_publisher',
        output='screen',
        parameters=[str(parameter_file)],
    )

    status_monitor = Node(
        package=PACKAGE_NAME,
        executable='inspection_status_monitor',
        name='inspection_status_monitor',
        output='screen',
    )

    return LaunchDescription([
        status_publisher,
        status_monitor,
    ])

