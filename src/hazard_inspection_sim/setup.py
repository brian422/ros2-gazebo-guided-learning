"""Installation description for the hazard_inspection_sim package."""

from glob import glob

from setuptools import find_packages, setup

package_name = 'hazard_inspection_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/rviz', glob('rviz/*.rviz')),
        (f'share/{package_name}/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hazard Inspection Rover Maintainer',
    maintainer_email='maintainer@users.noreply.github.com',
    description=(
        'Gazebo Harmonic world and ROS-Gazebo integration for the '
        'hazard inspection rover.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hazard_sensor_node = '
            'hazard_inspection_sim.hazard_sensor_node:main',
        ],
    },
)
