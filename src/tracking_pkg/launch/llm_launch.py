"""
LLM-based Robotic Scrub Nurse Launch File
==========================================
New architecture with natural language interaction, OBB-based tool detection,
grasp reasoning, world model, and LLM-driven execution.

Cameras use the official realsense2_camera ROS2 package.
Set SCENE_CAM_SERIAL / TRAY_CAM_SERIAL env vars or edit serial_no below.

Startup order:
  1. Static TFs (world→base)
  2. Scene camera (realsense2_camera, side: hand tracking + marker)
  3. Tray camera  (realsense2_camera, top-down: instrument detection) [when available]
  4. ArUco marker manager (config-driven static + detection TFs)
  5. Hand tracker (continuous, no gestures)
  6. Tray perception (OBB model) [when model available]
  7. Grasp reasoning + World model
  8. Gripper + execution nodes
  9. ASR + LLM
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    profile_config = PathJoinSubstitution(
        [FindPackageShare('tracking_pkg'), 'config', 'loop_mover_profiles.yaml']
    )
    rs_launch_file = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    )

    # ── Camera serial numbers ─────────────────────────────────
    # Set via environment variable or hardcode here.
    scene_cam_serial = os.environ.get('SCENE_CAM_SERIAL', '')
    tray_cam_serial = os.environ.get('TRAY_CAM_SERIAL', '')

    return LaunchDescription([

        # ── Layer 0: Static TFs ───────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_tf',
            output='screen',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--yaw', '3.141592653589793', '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 'world', '--child-frame-id', 'base',
            ]
        ),

        # ── Layer 1: Scene Camera (realsense2_camera) ─────────────
        # Publishes to /scene_camera/color/image_raw,
        #              /scene_camera/aligned_depth_to_color/image_raw,
        #              /scene_camera/color/camera_info, etc.
        # publish_tf is disabled: aruco_marker_manager wires camera frames
        # into the world tree on first marker detection.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_file),
            launch_arguments={
                'camera_name': 'scene_camera',
                'camera_namespace': 'scene_camera',
                'serial_no': scene_cam_serial,
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
                'publish_tf': 'false',
            }.items(),
        ),

        # ── Layer 1b: Tray Camera (second RealSense) ─────────────
        # TODO: Enable when second camera is physically set up.
        #       Set TRAY_CAM_SERIAL env var to the camera's serial.
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(rs_launch_file),
        #     launch_arguments={
        #         'camera_name': 'tray_camera',
        #         'camera_namespace': 'tray_camera',
        #         'serial_no': tray_cam_serial,
        #         'enable_color': 'true',
        #         'enable_depth': 'true',
        #         'rgb_camera.color_profile': '640,480,30',
        #         'depth_module.depth_profile': '640,480,30',
        #         'align_depth.enable': 'true',
        #         'spatial_filter.enable': 'true',
        #         'temporal_filter.enable': 'true',
        #         'hole_filling_filter.enable': 'true',
        #         'enable_sync': 'true',
        #         'publish_tf': 'false',
        #     }.items(),
        # ),

        # ── Layer 2: ArUco Marker Manager ─────────────────────────
        # Loads config/aruco_markers.yaml and handles:
        #   - Static TFs (marker frames + tool_holder_frame)
        #   - Camera-pose detection (one-shot static TFs per marker)
        Node(
            package='tracking_pkg',
            executable='aruco_marker_manager.py',
            name='aruco_marker_manager',
            output='screen',
        ),

        # ── Layer 3: Perception ───────────────────────────────────
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='hand_tracker.py',
                    name='hand_tracker',
                    output='screen',
                    parameters=[{
                        'max_num_hands': 2,
                        'publish_rate_hz': 15.0,
                        'annotated_image_max_hz': 12.0,
                    }],
                    # Remap to match realsense2_camera topic names
                    remappings=[
                        ('color_image', '/scene_camera/color/image_raw'),
                        ('depth_image', '/scene_camera/aligned_depth_to_color/image_raw'),
                        ('camera_info', '/scene_camera/color/camera_info'),
                    ],
                ),
            ]
        ),
        # TODO: Enable when OBB model is trained
        # TimerAction(
        #     period=3.0,
        #     actions=[
        #         Node(
        #             package='tracking_pkg',
        #             executable='tray_perception_node.py',
        #             name='tray_perception',
        #             output='screen',
        #             parameters=[{
        #                 'model_path': '/path/to/obb_model.pt',
        #                 'confidence_threshold': 0.5,
        #                 'inference_rate_hz': 10.0,
        #             }],
        #             remappings=[
        #                 ('tray_camera/color_image', '/tray_camera/color/image_raw'),
        #                 ('tray_camera/depth_image', '/tray_camera/aligned_depth_to_color/image_raw'),
        #                 ('tray_camera/camera_info', '/tray_camera/color/camera_info'),
        #             ],
        #         ),
        #     ]
        # ),

        # ── Layer 4: Reasoning ────────────────────────────────────
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='grasp_reasoning_node.py',
                    name='grasp_reasoning',
                    output='screen',
                ),
                Node(
                    package='tracking_pkg',
                    executable='world_model_node.py',
                    name='world_model',
                    output='screen',
                ),
            ]
        ),

        # ── Layer 5: Execution ────────────────────────────────────
        Node(
            package='tracking_pkg',
            executable='grasp_approach_pose_service.py',
            name='grasp_approach_pose_service',
            output='screen',
        ),
        Node(
            package='tracking_pkg',
            executable='gripper_opener_with_zeroer.py',
            name='gripper_opener_with_zeroer',
            output='screen',
            parameters=[profile_config],
        ),
        Node(
            package='tracking_pkg',
            executable='reclaim_controller.py',
            name='reclaim_controller',
            output='screen',
            parameters=[profile_config],
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='loop_mover',
                    name='loop_mover',
                    output='screen',
                    parameters=[profile_config],
                ),
            ]
        ),

        # ── Layer 6: Interfaces ───────────────────────────────────
        Node(
            package='tracking_pkg',
            executable='handover_sound_publisher.py',
            name='handover_sound_publisher',
            output='screen',
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='asr_node.py',
                    name='asr_node',
                    output='screen',
                    parameters=[{
                        'whisper_model': 'base',
                        'language': 'de',
                        'silence_threshold_seconds': 0.8,
                    }],
                ),
            ]
        ),

        # ── Layer 7: LLM ─────────────────────────────────────────
        # TODO: Enable when OpenAI API key is configured
        # TimerAction(
        #     period=4.0,
        #     actions=[
        #         Node(
        #             package='tracking_pkg',
        #             executable='llm_reasoning_node.py',
        #             name='llm_reasoning',
        #             output='screen',
        #             parameters=[{
        #                 'model_name': 'gpt-4o',
        #             }],
        #         ),
        #     ]
        # ),
    ])
