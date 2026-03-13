from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    profile_config = PathJoinSubstitution(
        [FindPackageShare('tracking_pkg'), 'config', 'loop_mover_profiles.yaml']
    )

    return LaunchDescription([
        Node(
            package='tracking_pkg',
            executable='camera_publisher.py', 
            name='camera_publisher',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_tf',
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
            package='tf2_ros',
            executable='static_transform_publisher',
            name='board_to_base_tf',
            output='screen',
            arguments=[
                '--x', '0.05',
                '--y', '-0.3',
                '--z', '0.0',
                '--yaw', '0.0',
                '--pitch', '0.0',
                '--roll', '0.0',
                '--frame-id', 'base',
                '--child-frame-id', 'aruco_board_frame',
            ]
        ),
        Node(
            package='tracking_pkg',
            executable='frame_publisher.py',
            name='frame_publisher',
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
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='hand_tracker.py', 
                    name='hand_tracker',
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=1.5,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='gesture_pose_publisher.py', 
                    name='gesture_pose_marker',
                    output='screen'
                )
            ]
        ),
        # Verzögerter Start der Node um 5 Sekunden
        TimerAction(
            period=5.0,  # Verzögerung in Sekunden
            actions=[
                Node(
                package='tracking_pkg',
                executable='loop_mover', 
                name='loop_mover',
                output='screen',
                parameters=[profile_config]
                )
            ]
        ),
        # Verzögerter Start der Node um 5 Sekunden
        TimerAction(
            period=5.0,  # Verzögerung in Sekunden
            actions=[
                Node(
            package='tracking_pkg',
            executable='mir_publisher.py', 
            name='mir_publisher',
            output='screen'
                )
            ]
        )
    ])
