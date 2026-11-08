"""Launch the complete Module 5 hazard inspection demonstration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


BRINGUP_PACKAGE = 'hazard_inspection_bringup'
CONTROL_PACKAGE = 'hazard_inspection_control'
SIM_PACKAGE = 'hazard_inspection_sim'


def generate_launch_description() -> LaunchDescription:
    """Create simulation, sensing, control, services, logging, and RViz."""
    bringup_share = Path(
        get_package_share_directory(BRINGUP_PACKAGE)
    )
    sim_share = Path(get_package_share_directory(SIM_PACKAGE))

    simulation_launch = sim_share / 'launch' / 'simulation.launch.py'
    hazard_parameters = sim_share / 'config' / 'hazard_sensor.yaml'
    inspection_parameters = (
        bringup_share / 'config' / 'inspection_params.yaml'
    )

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_directory = LaunchConfiguration('log_directory')

    gui_argument = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start the Gazebo graphical client.',
    )
    rviz_argument = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 with the integrated display.',
    )
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time for ROS nodes.',
    )
    log_directory_argument = DeclareLaunchArgument(
        'log_directory',
        default_value='~/.ros/hazard_inspection_rover/logs',
        description='Directory for timestamped mission CSV output.',
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(simulation_launch)),
        launch_arguments={
            'gui': gui,
            'rviz': rviz,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    hazard_sensor = Node(
        package=SIM_PACKAGE,
        executable='hazard_sensor_node',
        name='hazard_sensor_node',
        output='screen',
        parameters=[
            str(hazard_parameters),
            {'use_sim_time': use_sim_time},
        ],
    )
    mission_controller = Node(
        package=CONTROL_PACKAGE,
        executable='mission_controller_node',
        name='mission_controller_node',
        output='screen',
        parameters=[
            str(inspection_parameters),
            {'use_sim_time': use_sim_time},
        ],
    )
    mission_manager = Node(
        package=CONTROL_PACKAGE,
        executable='mission_manager_node',
        name='mission_manager_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    mission_logger = Node(
        package=CONTROL_PACKAGE,
        executable='mission_logger_node',
        name='mission_logger_node',
        output='screen',
        parameters=[
            str(inspection_parameters),
            {
                'use_sim_time': use_sim_time,
                'output_directory': log_directory,
            },
        ],
    )

    return LaunchDescription([
        gui_argument,
        rviz_argument,
        use_sim_time_argument,
        log_directory_argument,
        simulation,
        hazard_sensor,
        mission_controller,
        mission_manager,
        mission_logger,
    ])
