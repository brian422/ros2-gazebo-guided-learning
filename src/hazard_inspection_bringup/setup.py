"""Installation configuration for hazard_inspection_bringup."""

from glob import glob
import os

from setuptools import find_packages, setup


PACKAGE_NAME = 'hazard_inspection_bringup'


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
    description='Main launch and configuration for the inspection demo.',
    license='MIT',
)
