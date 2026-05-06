"""
Minimal Test Launch for World Model Builder
============================================
Starts ONLY the world_model_builder node in DIRECT mode:
the node opens its own pyrealsense2 pipeline (no realsense2_camera
process, no continuous ROS image publishing). Frames are pulled from
the open pipeline only when /build_world_model is called.

This keeps idle CPU low - good for the NUC.

No MoveIt, no scene camera, no ArUco manager, no execution. A static
world -> tray_camera_color_optical_frame TF is started so the service can
return world-frame grasp coordinates during instrument-camera pick tests.

Usage:
  # If only one RealSense is connected, leave TRAY_CAM_SERIAL unset
  # (the builder will pick the first available device).
  ros2 launch tracking_pkg test_world_model_launch.py

  # Or pin to a specific camera by serial:
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
from launch_ros.actions import Node


def generate_launch_description():
    tray_cam_serial = os.environ.get('TRAY_CAM_SERIAL', '')

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
                'grasp_offset_fraction': 1.0 / 10.0,
                # streaming-mode-only safeguard, ignored in direct mode
                'max_image_age_sec': 5.0,
            }],
        ),
    ])
