from setuptools import find_packages
from setuptools import setup

setup(
    name='me416_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('me416_msgs', 'me416_msgs.*')),
)
