"""Launch the Module 3 Gazebo Harmonic simulation and RViz2 view."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DESCRIPTION_PACKAGE = 'hazard_inspection_description'
SIM_PACKAGE = 'hazard_inspection_sim'
WORLD_NAME = 'inspection_world'
ROBOT_NAME = 'hazard_inspection_rover'


def generate_launch_description() -> LaunchDescription:
    """Create all launch actions needed for the Module 3 simulation."""
    description_share = Path(
        get_package_share_directory(DESCRIPTION_PACKAGE)
    )
    sim_share = Path(get_package_share_directory(SIM_PACKAGE))
    ros_gz_sim_share = Path(get_package_share_directory('ros_gz_sim'))

    default_model = (
        description_share / 'urdf' / 'hazard_rover.urdf.xacro'
    )
    default_world = sim_share / 'worlds' / 'inspection_world.sdf'
    bridge_config = sim_share / 'config' / 'bridge.yaml'
    rviz_config = sim_share / 'rviz' / 'simulation.rviz'
    gz_launch_file = ros_gz_sim_share / 'launch' / 'gz_sim.launch.py'

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    gui_argument = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start the Gazebo graphical client.',
    )
    rviz_argument = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 with the simulation configuration.',
    )
    model_argument = DeclareLaunchArgument(
        'model',
        default_value=str(default_model),
        description='Absolute path to the rover Xacro model.',
    )
    world_argument = DeclareLaunchArgument(
        'world',
        default_value=str(default_world),
        description='Absolute path to a Gazebo Harmonic world file.',
    )
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo /clock instead of wall-clock time.',
    )

    # Quotation around the path protects workspaces whose directory has spaces.
    robot_description = ParameterValue(
        Command(['xacro "', model, '" simulation:=true']),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # The GUI version starts both server and client.
    gazebo_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_launch_file)),
        launch_arguments={
            'gz_args': ['-r -v 3 "', world, '"'],
        }.items(),
        condition=IfCondition(gui),
    )

    # The -s flag starts only the server for lighter or display-free testing.
    gazebo_server_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_launch_file)),
        launch_arguments={
            'gz_args': ['-s -r -v 3 "', world, '"'],
        }.items(),
        condition=UnlessCondition(gui),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config}',
        ],
    )

    # A short delay gives Gazebo time to create the world's /create service.
    spawn_rover = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_hazard_inspection_rover',
                output='screen',
                arguments=[
                    '-world', WORLD_NAME,
                    '-name', ROBOT_NAME,
                    '-topic', 'robot_description',
                    '-x', '-4.0',
                    '-y', '-2.5',
                    '-z', '0.03',
                    '-Y', '0.0',
                ],
            ),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config)],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        gui_argument,
        rviz_argument,
        model_argument,
        world_argument,
        use_sim_time_argument,
        robot_state_publisher,
        gazebo_with_gui,
        gazebo_server_only,
        bridge,
        spawn_rover,
        rviz_node,
    ])
