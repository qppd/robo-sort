from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robosort_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboSort',
    maintainer_email='quezon.province.pd@gmail.com',
    description='RoboSort Control - DC Motor and LiDAR-based autonomous navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = robosort_control.motor_controller:main',
            'obstacle_avoidance = robosort_control.obstacle_avoidance:main',
            'tf_broadcaster = robosort_control.tf_broadcaster:main',
            'lidar_tf_publisher = robosort_control.lidar_tf_publisher:main',
        ],
    },
)
