from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    perception = os.path.join(
        get_package_share_directory('dobot_perception'), 'launch', 'cup_pose.launch.py')
    instruction = LaunchConfiguration('instruction').perform(context)
    cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception),
        launch_arguments={
            'device': LaunchConfiguration('device').perform(context),
            'show_window': LaunchConfiguration('show_window').perform(context),
            'rviz': 'false',
        }.items(),
    )
    bridge = Node(
        package='dobot_er2',
        executable='er2_bridge_node',
        name='er2_bridge_node',
        output='screen',
        parameters=[{
            'instruction': instruction,
            'model': LaunchConfiguration('model').perform(context),
        }],
    )
    return [cam, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('show_window', default_value='true'),
        DeclareLaunchArgument('model', default_value='gemini-robotics-er-2-preview'),
        DeclareLaunchArgument(
            'instruction',
            default_value='Pick up the cup and place it 8cm to the right on the same table.'),
        OpaqueFunction(function=launch_setup),
    ])
