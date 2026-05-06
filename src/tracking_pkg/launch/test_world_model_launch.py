"""
Minimal Test Launch for World Model Builder
============================================
Starts the world_model_builder node in DIRECT mode for the tray camera:
the node opens its own pyrealsense2 pipeline for tray frames. Frames are pulled
from the open tray pipeline only when /build_world_model is called.

This keeps idle CPU low - good for the NUC.

No MoveIt and no execution. The official realsense2_camera node starts the
scene camera for marker-105 localization and hand tracking. A static
world -> tray_camera_color_optical_frame TF is started so the service can return
world-frame grasp coordinates during instrument-camera pick tests.
The MiR base and tray-camera volume are also published as collision objects
for quick Planning Scene / RViz checks when a consumer is available.

Usage:
  # The scene camera defaults to serial 239222300719. Override if needed:
  SCENE_CAM_SERIAL=239222300719 ros2 launch tracking_pkg test_world_model_launch.py

  # The tray camera defaults to serial 239222302690. Override if needed:
  TRAY_CAM_SERIAL=239222302690 ros2 launch tracking_pkg test_world_model_launch.py

  # Optionally override the OBB model path:
  OBB_MODEL_PATH=/path/to/first_obb_test.pt ros2 launch tracking_pkg test_world_model_launch.py

In a second terminal:
  ros2 service call /build_world_model tracking_pkg/srv/BuildWorldModel

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
    scene_cam_serial = os.environ.get('SCENE_CAM_SERIAL', '239222300719')
    tray_cam_serial = os.environ.get('TRAY_CAM_SERIAL', '239222302690')

    obb_model_path = os.environ.get(
        'OBB_MODEL_PATH',
        '/home/mir/robotic_scrub_nurse_ws/ros_unrelated_scripts/first_obb_test.pt',
    )

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_tray_camera_tf',
            output='screen',
            arguments=[
                '--x', '-0.075', '--y', '0.349', '--z', '0.4325',
                '--qx', '0.0', '--qy', '1.0', '--qz', '0.0', '--qw', '0.0',
                '--frame-id', 'world', '--child-frame-id', 'tray_camera_color_optical_frame',
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
                'camera_name': 'scene_camera',
                'camera_namespace': '',
                'serial_no': f"'{scene_cam_serial}'",
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
                # Direct mode: builder opens pyrealsense2 pipeline itself
                'camera_mode': 'direct',
                'realsense_serial': tray_cam_serial,
                'color_width': 1280,
                'color_height': 720,
                'color_fps': 30,
                'warmup_frames': 5,
                # Matches the static TF above; no fixed ArUco marker is
                # required for this pick-test path.
                'tray_camera_frame': 'tray_camera_color_optical_frame',
                'world_frame': 'world',
                'conf_threshold': 0.35,
                'imgsz': 1024,
                'device': 'cpu',
                'handle_class_name': 'handle',
                'grasp_offset_fraction': 1.0 / 8.0,
                'fixed_tool_plane_z_m': 0.05,
                # streaming-mode-only safeguard, ignored in direct mode
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
                        'camera_frame': 'scene_camera_color_optical_frame',
                        'world_frame': 'world',
                        'max_num_hands': 2,
                        'publish_rate_hz': 15.0,
                        'annotated_image_max_hz': 12.0,
                    }],
                    remappings=[
                        ('color_image', '/scene_camera/color/image_raw'),
                        ('depth_image', '/scene_camera/aligned_depth_to_color/image_raw'),
                        ('camera_info', '/scene_camera/color/camera_info'),
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
            executable='tray_camera_volume_publisher.py',
            name='tray_camera_volume_publisher',
            output='screen',
            parameters=[{
                'frame_id': 'tray_camera_color_optical_frame',
                'collision_topic': '/collision_object',
                'object_id': 'tray_camera_volume',
                'width_m': 0.05,
                'height_m': 0.05,
                'length_m': 0.60,
                'start_offset_m': 0.10,
                'publish_hz': 2.0,
            }],
        ),
    ])
