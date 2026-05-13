"""
Lightweight MoveIt Pick-Test Launch
===================================
Starts MoveIt, the official scene-camera RealSense node, marker-105 camera
localization, hand tracking, the static instrument-camera TF, the on-demand
world-model builder, the gripper bridge, and optional RViz.

Run the interactive picker separately in a terminal with stdin:
  ros2 run tracking_pkg tool_pick_test_node
"""

import os

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

    profile_config = PathJoinSubstitution(
        [FindPackageShare("tracking_pkg"), "config", "loop_mover_profiles.yaml"]
    )
    rs_launch_file = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )
    scene_cam_serial = os.environ.get("SCENE_CAM_SERIAL", "239222300719")
    tray_cam_serial = os.environ.get("TRAY_CAM_SERIAL", "239222302690")
    obb_model_path = os.environ.get(
        "OBB_MODEL_PATH",
        "/home/mir/robotic_scrub_nurse_ws/ros_unrelated_scripts/first_obb_test.pt",
    )

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
        name="rviz2_pick_test",
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_file),
            launch_arguments={
                "camera_name": "scene_camera",
                "camera_namespace": "",
                "serial_no": f"'{scene_cam_serial}'",
                "enable_color": "true",
                "enable_depth": "true",
                "rgb_camera.color_profile": "640,480,30",
                "depth_module.depth_profile": "640,480,30",
                "align_depth.enable": "true",
                "spatial_filter.enable": "true",
                "temporal_filter.enable": "true",
                "hole_filling_filter.enable": "true",
                "decimation_filter.enable": "true",
                "enable_sync": "true",
                # The ArUco manager connects the detected camera pose into
                # the world tree through aruco_marker_105_frame.
                "publish_tf": "false",
            }.items(),
        ),
        Node(
            package="tracking_pkg",
            executable="aruco_marker_manager.py",
            name="aruco_marker_manager",
            output="screen",
            parameters=[{
                "publish_marker_static_tfs": False,
            }],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="tracking_pkg",
                    executable="world_model_builder.py",
                    name="world_model_builder",
                    output="screen",
                    parameters=[{
                        "model_path": obb_model_path,
                        "camera_mode": "direct",
                        "realsense_serial": tray_cam_serial,
                        "color_width": 1280,
                        "color_height": 720,
                        "color_fps": 30,
                        "warmup_frames": 5,
                        "tray_camera_frame": "tray_camera_color_optical_frame",
                        "world_frame": "world",
                        "conf_threshold": 0.35,
                        "imgsz": 1024,
                        "device": "cpu",
                        "handle_class_name": "handle",
                        "grasp_offset_fraction": 1.0 / 10.0,
                        "fixed_tool_plane_z_m": 0.04,
                        "depth_search_radius_px": 5,
                        "depth_min_m": 0.35,
                        "depth_max_m": 0.45,
                        "max_image_age_sec": 1.0,
                    }],
                ),
            ],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="tracking_pkg",
                    executable="hand_tracker.py",
                    name="hand_tracker",
                    output="screen",
                    parameters=[{
                        "camera_frame": "scene_camera_color_optical_frame",
                        "world_frame": "world",
                        "max_num_hands": 2,
                        "publish_rate_hz": 15.0,
                        "annotated_image_max_hz": 12.0,
                    }],
                    remappings=[
                        ("color_image", "/scene_camera/color/image_raw"),
                        ("depth_image", "/scene_camera/aligned_depth_to_color/image_raw"),
                        ("camera_info", "/scene_camera/color/camera_info"),
                    ],
                ),
            ],
        ),
        Node(
            package="tracking_pkg",
            executable="gripper_opener_with_zeroer.py",
            name="gripper_opener_with_zeroer",
            output="screen",
            parameters=[profile_config],
        ),
        Node(
            package="tracking_pkg",
            executable="mir_publisher.py",
            name="mir_publisher",
            output="screen",
        ),
        Node(
            package="tracking_pkg",
            executable="tray_camera_volume_publisher.py",
            name="tray_camera_volume_publisher",
            output="screen",
            parameters=[{
                "frame_id": "tray_camera_color_optical_frame",
                "collision_topic": "/collision_object",
                "object_id": "tray_camera_volume",
                "width_m": 0.05,
                "height_m": 0.05,
                "length_m": 0.60,
                "start_offset_m": -0.10,
                "publish_hz": 2.0,
            }],
        ),
        TimerAction(
            period=2.0,
            actions=[rviz_node],
        ),
    ])
