from setuptools import find_packages
from setuptools import setup

setup(
    name='mavros_msgs',
    version='2.6.0',
    packages=find_packages(
        include=('mavros_msgs', 'mavros_msgs.*')),
)
