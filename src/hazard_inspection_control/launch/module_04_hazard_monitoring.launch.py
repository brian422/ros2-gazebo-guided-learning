"""Launch Module 3 simulation plus Module 4 sensing and mission state."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CONTROL_PACKAGE = 'hazard_inspection_control'
SIM_PACKAGE = 'hazard_inspection_sim'


def generate_launch_description() -> LaunchDescription:
    """Create the complete Module 4 learning-system launch."""
    control_share = Path(get_package_share_directory(CONTROL_PACKAGE))
    sim_share = Path(get_package_share_directory(SIM_PACKAGE))

    simulation_launch = sim_share / 'launch' / 'simulation.launch.py'
    hazard_parameters = sim_share / 'config' / 'hazard_sensor.yaml'
    controller_parameters = control_share / 'config' / 'module_04.yaml'

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    gui_argument = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start the Gazebo graphical client.',
    )
    rviz_argument = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 through the Module 3 simulation launch.',
    )
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time for all Module 4 nodes.',
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
            str(controller_parameters),
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

    return LaunchDescription([
        gui_argument,
        rviz_argument,
        use_sim_time_argument,
        simulation,
        hazard_sensor,
        mission_controller,
        mission_manager,
    ])
