"""
Joint-State Jogger MoveIt/RViz Launch
=====================================
Starts the MoveIt and RViz infrastructure needed by the interactive
joint_state_jogger_node. Run the jogger itself in a second terminal so stdin
works reliably:

  ros2 run tracking_pkg joint_state_jogger_node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    tracking_rviz = LaunchConfiguration("tracking_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "false",
            "launch_servo": launch_servo,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    "xacro ",
                    PathJoinSubstitution(
                        [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
                    ),
                    " ur_type:=",
                    ur_type,
                    " name:=ur",
                    " prefix:=",
                ]
            ),
            value_type=str,
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(
                [
                    "xacro ",
                    PathJoinSubstitution(
                        [FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]
                    ),
                    " ur_type:=",
                    ur_type,
                    " name:=ur",
                ]
            ),
            value_type=str,
        )
    }
    robot_description_kinematics_path = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_joint_state_jogger",
        output="log",
        condition=IfCondition(tracking_rviz),
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("tracking_pkg"), "rviz", "view_robot_tracking.rviz"]
            ),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics_path,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("tracking_rviz", default_value="true"),
        DeclareLaunchArgument("launch_servo", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        SetEnvironmentVariable("LC_NUMERIC", "en_US.UTF-8"),
        ur_moveit_launch,
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_tray_camera_tf",
            output="screen",
            arguments=[
                "--x", "-0.075", "--y", "0.349", "--z", "0.4325",
                "--qx", "0.0", "--qy", "1.0", "--qz", "0.0", "--qw", "0.0",
                "--frame-id", "world", "--child-frame-id", "tray_camera_color_optical_frame",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_aruco_marker_105_tf",
            output="screen",
            arguments=[
                "--x", "0.0", "--y", "-0.44", "--z", "-0.0855",
                "--qx", "0.7071067811865475", "--qy", "0.0",
                "--qz", "0.0", "--qw", "0.7071067811865476",
                "--frame-id", "world", "--child-frame-id", "aruco_marker_105_frame",
            ],
        ),
        TimerAction(
            period=2.0,
            actions=[rviz_node],
        ),
    ])
