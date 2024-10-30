import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'me416_lab'
package_path = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[('share/ament_index/resource_index/packages',
                 ['resource/' + package_name]),
                (package_path, ['package.xml']),
                (package_path, glob('launch/*launch.[pxy][yma]*'))],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='tron@bu.edu',
    description='Scaffold files for ME416',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = me416_lab.listener:main',
            'talker = me416_lab.talker:main',
            'repeater = me416_lab.repeater:main',
            'listener_accumulator = me416_lab.listener_accumulator:main',
            'talker_interesting = me416_lab.talker_interesting:main',
            'motorspeeds_talker = me416_lab.motorspeeds_talker:main',
            'motor_command = me416_lab.motor_command:main',
            'zigzag_op = me416_lab.zigzag_op:main',
            'key_terminate = me416_lab.key_terminate:main',
            'key_op = me416_lab.key_op:main',
            'scripted_op = me416_lab.scripted_op:main',
            'image_flipper = me416_lab.image_flipper:main',
            'image_segment = me416_lab.image_segment:main',
            'encoders_publisher = me416_lab.encoders_publisher:main',
            'odometry_encoders = me416_lab.odometry_encoders:main',
            'signal_generator = me416_lab.signal_generator:main',
            'controller_test = me416_lab.controller_test:main',
            'controller_line = me416_lab.controller_line:main',
        ],
    },
)
