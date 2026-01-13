from setuptools import find_packages, setup

package_name = 'robosort_sensors'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/lidar.launch.py',
            'launch/nav2_slam.launch.py',
            'launch/nav2_visualization.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
            'config/slam_params.yaml',
            'config/slam_nav2.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboSort',
    maintainer_email='quezon.province.pd@gmail.com',
    description='RoboSort Sensor Package - LiDAR LD06 integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_processor = robosort_sensors.lidar_processor:main',
            'object_localizer = robosort_sensors.object_localizer:main',
            'fake_odom = robosort_sensors.fake_odom:main',
        ],
    },
)
