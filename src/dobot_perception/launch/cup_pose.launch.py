from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('dobot_perception')
    rviz_cfg = os.path.join(share, 'rviz', 'cup_pose.rviz')
    classes = [s.strip() for s in LaunchConfiguration('classes').perform(context).split(',') if s.strip()]

    nodes = [
        Node(
            package='dobot_perception',
            executable='cup_pose_node',
            name='cup_pose_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device': LaunchConfiguration('device'),
                'show_window': ParameterValue(
                    LaunchConfiguration('show_window'), value_type=bool),
                'conf': ParameterValue(LaunchConfiguration('conf'), value_type=float),
                'target_classes': classes,
                'model': LaunchConfiguration('model'),
                'orient_mode': LaunchConfiguration('orient_mode'),
            }],
        )
    ]
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('show_window', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('conf', default_value='0.65'),
        DeclareLaunchArgument(
            'model',
            default_value='yolo26x.pt',
            description='Ultralytics weights name or absolute path'),
        DeclareLaunchArgument(
            'orient_mode',
            default_value='pca_max',
            description='pca_max | pca_min | view | plane | identity'),
        DeclareLaunchArgument(
            'classes',
            default_value='cup,bottle,wine glass,bowl',
            description='COCO class names to treat as the target object'),
        OpaqueFunction(function=launch_setup),
    ])
