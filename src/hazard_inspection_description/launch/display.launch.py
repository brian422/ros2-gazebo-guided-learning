"""Display the Module 2 rover model and TF tree in RViz2."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = 'hazard_inspection_description'


def generate_launch_description() -> LaunchDescription:
    """Create the robot-description visualization launch description."""
    package_share = Path(get_package_share_directory(PACKAGE_NAME))
    default_model = package_share / 'urdf' / 'hazard_rover.urdf.xacro'
    default_rviz_config = package_share / 'rviz' / 'inspection.rviz'

    model_argument = DeclareLaunchArgument(
        'model',
        default_value=str(default_model),
        description='Absolute path to the rover Xacro model.',
    )
    use_gui_argument = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Start joint_state_publisher_gui when true.',
    )
    rviz_argument = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 when true.',
    )
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time. Keep false in Module 2.',
    )

    model = LaunchConfiguration('model')
    use_gui = LaunchConfiguration('use_gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_content = ParameterValue(
        Command(['xacro "', model, '"']),
        value_type=str,
    )
    robot_description = {
        'robot_description': robot_description_content,
        'use_sim_time': use_sim_time,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui),
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_gui),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(default_rviz_config)],
        condition=IfCondition(rviz),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        model_argument,
        use_gui_argument,
        rviz_argument,
        use_sim_time_argument,
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz_node,
    ])
