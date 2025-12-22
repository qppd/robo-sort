from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robosort_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robosort',
    maintainer_email='quezon.province.pd@gmail.com',
    description='RoboSort Vision Package - AI-powered waste segregation system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = robosort_vision.yolo_detector:main',
            'arduino_serial = robosort_vision.arduino_serial:main',
            'waste_segregation_controller = robosort_vision.waste_segregation_controller:main',
        ],
    },
)
