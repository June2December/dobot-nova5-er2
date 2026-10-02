import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dobot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Large YOLO weights stay in src/.../models and are resolved at runtime.
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='june',
    maintainer_email='june@todo.todo',
    description='RealSense D405 RGB-D cup 6D pose estimation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cup_pose_node = dobot_perception.cup_pose_node:main',
        ],
    },
)
