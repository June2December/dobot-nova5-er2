"""CR16 MoveIt fake sim + D405 cup pose with bbox image in RViz.

Temporary camera mount TF (front table, looking down). Tune xyz/rpy after
you fix the real mount calibration.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('dobot_perception')
    rviz_cfg = os.path.join(share, 'rviz', 'moveit_cup_pose.rviz')
    classes = [
        s.strip()
        for s in LaunchConfiguration('classes').perform(context).split(',')
        if s.strip()
    ]

    moveit_config = MoveItConfigsBuilder(
        'cr16_robot', package_name='cr16_moveit').to_moveit_configs()

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('cr16_moveit'), 'launch', 'demo.launch.py'
            ])
        ]),
        launch_arguments={'use_rviz': 'false'}.items(),
    )

    # Temporary: camera on robot looking at front table.
    # optical frame: X right, Y down, Z forward (viewing direction).
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

    perception = Node(
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

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return [moveit, camera_tf, perception, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='cuda:0'),
        DeclareLaunchArgument('show_window', default_value='false'),
        DeclareLaunchArgument('conf', default_value='0.35'),
        DeclareLaunchArgument(
            'model',
            default_value='yolo26x.pt',
            description='Ultralytics weights name or absolute path'),
        DeclareLaunchArgument(
            'orient_mode',
            default_value='pca_max',
            description='pca_max | pca_min | view | plane | identity'),
        DeclareLaunchArgument(
            'classes', default_value='cup,bottle,wine glass,bowl'),
        # Eyeballed from RViz: old (0.30,0,0.55) sat on the base mesh.
        # Push forward/up so the optical frame clears the pedestal and looks
        # down at the front table (~0.7–1.0 m along +X).
        DeclareLaunchArgument('cam_x', default_value='0.50'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.75'),
        DeclareLaunchArgument('cam_roll', default_value='-2.50'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_yaw', default_value='-1.5708'),
        OpaqueFunction(function=launch_setup),
    ])
