import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dobot_er2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='june',
    maintainer_email='june@todo.todo',
    description='Gemini Robotics ER 2 ROS 2 bridge for Dobot Nova 5',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'er2_bridge_node = dobot_er2.er2_bridge_node:main',
            'er2_skill_node = dobot_er2.er2_skill_node:main',
        ],
    },
)
