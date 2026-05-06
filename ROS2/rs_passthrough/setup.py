from setuptools import setup
import os
from glob import glob

package_name = 'rs_passthrough'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer_email='support@reachrobotics.com',
    description='Passthrough node for Reach Robotics Products.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_passthrough = rs_passthrough.serial_passthrough:main',
            'udp_passthrough = rs_passthrough.udp_passthrough:main',
            'joint_telemetry = rs_passthrough.joint_telemetry:main',
            'master_arm_bridge = rs_passthrough.master_arm_bridge:main',
            'get_end_effector_pose = rs_passthrough.get_end_effector_pose:main',
            'move_ab = rs_passthrough.move_ab:main',
        ],
    },
)
