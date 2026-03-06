from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    show_annotated_feed = LaunchConfiguration("show_annotated_feed")
    profile_config = PathJoinSubstitution(
        [FindPackageShare('tracking_pkg'), 'config', 'loop_mover_profiles.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "show_annotated_feed",
            default_value="true",
            description="Start rqt_image_view for /annotated_hand_image.",
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='socket_world_to_base_tf',
            output='screen',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--yaw', '3.141592653589793',
                '--pitch', '0.0',
                '--roll', '0.0',
                '--frame-id', 'world',
                '--child-frame-id', 'base',
            ]
        ),
        Node(
            package='tracking_pkg',
            executable='camera_publisher.py',
            name='camera_publisher',
            output='screen'
        ),
        Node(
            package='tracking_pkg',
            executable='frame_publisher.py',
            name='frame_publisher',
            output='screen'
        ),
        Node(
            package='tracking_pkg',
            executable='hand_tracker.py',
            name='hand_tracker',
            output='screen'
        ),
        Node(
            package='tracking_pkg',
            executable='gripper_opener_with_zeroer.py',
            name='gripper_opener_with_zeroer',
            output='screen',
            parameters=[profile_config]
        ),
        Node(
            package='tracking_pkg',
            executable='reclaim_controller.py',
            name='reclaim_controller',
            output='screen',
            parameters=[profile_config]
        ),
        Node(
            package='tracking_pkg',
            executable='handover_sound_publisher.py',
            name='handover_sound_publisher',
            output='screen'
        ),
        Node(
            package='tracking_pkg',
            executable='annotated_image_viewer.py',
            name='annotated_hand_image_view',
            output='log',
            parameters=[{'topic_name': '/annotated_hand_image'}],
            condition=IfCondition(show_annotated_feed)
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='socket_mover.py',
                    name='socket_mover',
                    output='screen',
                    parameters=[profile_config]
                )
            ]
        ),
    ])
