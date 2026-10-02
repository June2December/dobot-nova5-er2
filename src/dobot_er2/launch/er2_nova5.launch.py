"""D405 + ER 2 + Nova 5 skill node.

Bringup must already be running in another terminal:
  ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py
Camera TF is the same eyeballed mount as the perception demo. Tune after calibration.
"""
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
    cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception),
        launch_arguments={
            'device': LaunchConfiguration('device').perform(context),
            'show_window': LaunchConfiguration('show_window').perform(context),
            'rviz': 'false',
        }.items(),
    )
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_tf',
        arguments=[
            '--x', LaunchConfiguration('cam_x').perform(context),
            '--y', LaunchConfiguration('cam_y').perform(context),
            '--z', LaunchConfiguration('cam_z').perform(context),
            '--roll', LaunchConfiguration('cam_roll').perform(context),
            '--pitch', LaunchConfiguration('cam_pitch').perform(context),
            '--yaw', LaunchConfiguration('cam_yaw').perform(context),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_color_optical_frame',
        ],
    )
    bridge = Node(
        package='dobot_er2',
        executable='er2_bridge_node',
        name='er2_bridge_node',
        output='screen',
        parameters=[{
            'instruction': LaunchConfiguration('instruction').perform(context),
            'model': LaunchConfiguration('model').perform(context),
        }],
    )
    skill = Node(
        package='dobot_er2',
        executable='er2_skill_node',
        name='er2_skill_node',
        output='screen',
        parameters=[{
            'execute_robot': ParameterValue(
                LaunchConfiguration('execute_robot'), value_type=bool),
            'speed_ratio': ParameterValue(
                LaunchConfiguration('speed_ratio'), value_type=int),
            'tool_do_index': ParameterValue(
                LaunchConfiguration('tool_do_index'), value_type=int),
            'gripper_mode': LaunchConfiguration('gripper_mode').perform(context),
        }],
    )
    return [cam, camera_tf, bridge, skill]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('show_window', default_value='true'),
        DeclareLaunchArgument('model', default_value='gemini-robotics-er-2-preview'),
        DeclareLaunchArgument(
            'instruction',
            default_value='Pick up the cup and place it 8cm to the right on the same table.'),
        DeclareLaunchArgument(
            'execute_robot', default_value='false',
            description='true 일 때만 Nova 5가 움직입니다. 기본은 dry-run.'),
        DeclareLaunchArgument('speed_ratio', default_value='10'),
        DeclareLaunchArgument(
            'gripper_mode', default_value='manual',
            description='manual: 사람이 그리퍼 버튼 후 /er2/continue. tool_do: ToolDOExecute'),
        DeclareLaunchArgument('tool_do_index', default_value='1'),
        DeclareLaunchArgument('cam_x', default_value='0.50'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.75'),
        DeclareLaunchArgument('cam_roll', default_value='-2.50'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_yaw', default_value='-1.5708'),
        OpaqueFunction(function=launch_setup),
    ])
