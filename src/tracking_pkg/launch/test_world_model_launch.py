"""
Minimal Test Launch for World Model Builder
============================================
Starts the world_model_builder node in STREAMING mode: a realsense2_camera
node publishes /tray_camera/* topics, the builder subscribes and pulls a
fresh frame only when /build_world_model is called.

The tray camera is localized via ArUco marker 110 detected by
aruco_marker_manager; the resulting aruco_marker_110_frame ->
tray_camera_color_optical_frame TF chains together with the static
world -> aruco_marker_110_frame so service results land in world frame.

No MoveIt and no execution. The official realsense2_camera node starts the
reclaim tray camera for marker-105 localization and hand tracking.
The MiR base and tray-camera volume are also published as collision objects
for quick Planning Scene / RViz checks when a consumer is available.

Usage:
  # The reclaim tray camera defaults to serial 239222300719. Override if needed:
  RECLAIM_TRAY_CAM_SERIAL=239222300719 ros2 launch tracking_pkg test_world_model_launch.py

  # The tray camera defaults to serial 239222302690. Override if needed:
  TRAY_CAM_SERIAL=239222302690 ros2 launch tracking_pkg test_world_model_launch.py

  # Optionally override the OBB model path:
  OBB_MODEL_PATH=/path/to/first_obb_test.pt ros2 launch tracking_pkg test_world_model_launch.py

In a second terminal:
  ros2 service call /build_world_model tracking_msgs/srv/BuildWorldModel

In a third terminal:
  ros2 run rqt_image_view rqt_image_view /world_model/annotated_image
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rs_launch_file = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    )
    reclaim_tray_cam_serial = os.environ.get('RECLAIM_TRAY_CAM_SERIAL', '239222300719')
    tray_cam_serial = os.environ.get('TRAY_CAM_SERIAL', '239222302690')

    obb_model_path = os.environ.get(
        'OBB_MODEL_PATH',
        '/home/mir/robotic_scrub_nurse_ws/ros_unrelated_scripts/first_obb_test.pt',
    )

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_aruco_marker_110_tf',
            output='screen',
            arguments=[
                '--x', '-0.375', '--y', '0.0', '--z', '-0.01',
                '--qx', '0.0', '--qy', '0.0', '--qz', '1.0', '--qw', '0.0',
                '--frame-id', 'world', '--child-frame-id', 'aruco_marker_110_frame',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_aruco_marker_105_tf',
            output='screen',
            arguments=[
                '--x', '0.0', '--y', '-0.44', '--z', '-0.0855',
                '--qx', '0.7071067811865475', '--qy', '0.0',
                '--qz', '0.0', '--qw', '0.7071067811865476',
                '--frame-id', 'world', '--child-frame-id', 'aruco_marker_105_frame',
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_file),
            launch_arguments={
                'camera_name': 'reclaim_tray_camera',
                'camera_namespace': '',
                'serial_no': f"'{reclaim_tray_cam_serial}'",
                'enable_color': 'true',
                'enable_depth': 'true',
                'rgb_camera.color_profile': '640,480,30',
                'depth_module.depth_profile': '640,480,30',
                'align_depth.enable': 'true',
                'spatial_filter.enable': 'true',
                'temporal_filter.enable': 'true',
                'hole_filling_filter.enable': 'true',
                'decimation_filter.enable': 'true',
                'enable_sync': 'true',
                # The ArUco manager connects the detected camera pose into
                # the world tree through aruco_marker_105_frame.
                'publish_tf': 'false',
            }.items(),
        ),
        # Tray Camera (realsense2_camera) — publishes /tray_camera/* topics.
        # Consumed by aruco_marker_manager (marker 110 detection) and
        # world_model_builder (camera_mode='streaming', on-demand).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_file),
            launch_arguments={
                'camera_name': 'tray_camera',
                'camera_namespace': '',
                'serial_no': f"'{tray_cam_serial}'",
                'enable_color': 'true',
                'enable_depth': 'true',
                'rgb_camera.color_profile': '1280,720,30',
                'depth_module.depth_profile': '1280,720,30',
                'align_depth.enable': 'true',
                'spatial_filter.enable': 'true',
                'temporal_filter.enable': 'true',
                'hole_filling_filter.enable': 'true',
                'decimation_filter.enable': 'false',
                'enable_sync': 'true',
                # The ArUco manager connects the detected camera pose into
                # the world tree through aruco_marker_110_frame.
                'publish_tf': 'false',
            }.items(),
        ),
        Node(
            package='tracking_pkg',
            executable='aruco_marker_manager.py',
            name='aruco_marker_manager',
            output='screen',
            parameters=[{
                'publish_marker_static_tfs': False,
            }],
        ),
        Node(
            package='tracking_pkg',
            executable='world_model_builder.py',
            name='world_model_builder',
            output='screen',
            parameters=[{
                'model_path': obb_model_path,
                # Streaming mode: builder subscribes to /tray_camera/*
                'camera_mode': 'streaming',
                'tray_camera_namespace': '/tray_camera',
                'tray_camera_frame': 'tray_camera_color_optical_frame',
                'world_frame': 'world',
                'conf_threshold': 0.35,
                'imgsz': 1024,
                # GPU by default on the Jetson; override with OBB_DEVICE=cpu
                'device': os.environ.get('OBB_DEVICE', 'cuda:0'),
                'handle_class_name': 'handle',
                'grasp_offset_fraction': 1.0 / 8.0,
                'fixed_tool_plane_z_m': 0.04,
                'depth_min_m': 0.25,
                'depth_max_m': 0.55,
                'max_image_age_sec': 5.0,
            }],
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='hand_tracker.py',
                    name='hand_tracker',
                    output='screen',
                    parameters=[{
                        'camera_frame': 'reclaim_tray_camera_color_optical_frame',
                        'world_frame': 'world',
                        'max_num_hands': 2,
                        'publish_rate_hz': 15.0,
                        'annotated_image_max_hz': 12.0,
                    }],
                    remappings=[
                        ('color_image', '/reclaim_tray_camera/color/image_raw'),
                        ('depth_image', '/reclaim_tray_camera/aligned_depth_to_color/image_raw'),
                        ('camera_info', '/reclaim_tray_camera/color/camera_info'),
                    ],
                ),
            ],
        ),
        Node(
            package='tracking_pkg',
            executable='mir_publisher.py',
            name='mir_publisher',
            output='screen',
        ),
        Node(
            package='tracking_pkg',
            executable='instrument_tray_collision_publisher.py',
            name='instrument_tray_collision_publisher',
            output='screen',
            parameters=[{
                'publish_hz': 2.0,
            }],
        ),
        Node(
            package='tracking_pkg',
            executable='reclaim_tray_collision_publisher.py',
            name='reclaim_tray_collision_publisher',
            output='screen',
            parameters=[{
                'publish_hz': 2.0,
            }],
        ),
    ])
