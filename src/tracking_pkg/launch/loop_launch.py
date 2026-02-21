from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
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
            executable='gesture_pose_publisher.py', 
            name='gesture_pose_marker',
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
            output='screen'
        ),
        # Verzögerter Start der Node um 5 Sekunden
        TimerAction(
            period=5.0,  # Verzögerung in Sekunden
            actions=[
                Node(
                package='tracking_pkg',
                executable='loop_mover', 
                name='loop_mover',
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
            executable='mir_publisher.py', 
            name='mir_publisher',
            output='screen'
                )
            ]
        )
    ])
