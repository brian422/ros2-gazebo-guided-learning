"""Python packaging configuration for hazard_inspection_control."""

from glob import glob
import os

from setuptools import find_packages, setup


PACKAGE_NAME = 'hazard_inspection_control'


setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME],
        ),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (
            os.path.join('share', PACKAGE_NAME, 'config'),
            glob('config/*.yaml'),
        ),
        (
            os.path.join('share', PACKAGE_NAME, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hazard Inspection Rover Maintainer',
    maintainer_email='maintainer@users.noreply.github.com',
    description=(
        'Control and introductory communication nodes for the hazard '
        'inspection rover.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspection_status_monitor = '
            'hazard_inspection_control.inspection_status_monitor:main',
            'inspection_status_publisher = '
            'hazard_inspection_control.inspection_status_publisher:main',
            'mission_controller_node = '
            'hazard_inspection_control.mission_controller_node:main',
            'mission_manager_node = '
            'hazard_inspection_control.mission_manager_node:main',
            'mission_logger_node = '
            'hazard_inspection_control.mission_logger_node:main',
        ],
    },
)
