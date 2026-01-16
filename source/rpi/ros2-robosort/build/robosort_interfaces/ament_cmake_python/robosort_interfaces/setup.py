from setuptools import find_packages
from setuptools import setup

setup(
    name='robosort_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('robosort_interfaces', 'robosort_interfaces.*')),
)
