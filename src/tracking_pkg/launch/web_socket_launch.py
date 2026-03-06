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
        # Verzögerter Start der socket_mover-Node um 5 Sekunden
        TimerAction(
            period=0.5,  # Verzögerung in Sekunden
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='socket_mover.py',
                    name='socket_mover',
                    output='screen'
                )
            ]
        ),
        Node(
            package='tracking_pkg',
            executable='box_publisher.py', 
            name='box_publisher',
            output='screen'
        )
    ])