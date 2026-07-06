"""
Jetson Launch File — Perception & AI Layer
==========================================
Runs on the NVIDIA Jetson Orin Nano (Humble, ARM64, CUDA / JetPack).
Handles cameras, detection, hand tracking, grasp reasoning, world model, ASR and LLM.

Robot control (UR driver, MoveIt, skill execution) runs separately on the NUC via
nuc_launch.py. Both machines must share the same ROS_DOMAIN_ID and use CycloneDDS.

Env vars:
  SCENE_CAM_SERIAL   Serial of the scene camera  (default: 239222300719)
  TRAY_CAM_SERIAL    Serial of the tray camera   (default: 239222302690)
  OBB_MODEL_PATH     Path to the YOLO OBB model
  OBB_DEVICE         YOLO inference device       (default: cuda:0)
  OPENAI_API_KEY     OpenAI API key for LLM
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    scene_cam_serial = os.environ.get('SCENE_CAM_SERIAL', '239222300719')
    tray_cam_serial  = os.environ.get('TRAY_CAM_SERIAL',  '239222302690')

    ur_type       = LaunchConfiguration('ur_type')

    rs_launch_file = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    )

    # RViz runs on the NUC (which has the robot model + planning scene natively).
    # The Jetson is headless — it must NOT generate a robot_description here: that
    # would pull rsn_ur.urdf.xacro / the UR ros2_control xacro (not needed for
    # perception, and version-fragile), and any failure aborts the whole launch.

    return LaunchDescription([

        DeclareLaunchArgument('ur_type',       default_value='ur3e'),
        SetEnvironmentVariable('LC_NUMERIC', 'en_US.UTF-8'),

        # ── Static TFs (ArUco marker world-poses) ─────────────────
        # world→base is published by the NUC.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_aruco_marker_110_tf',
            output='screen',
            arguments=[
                '--x', '-0.375', '--y', '0.0', '--z', '-0.01',
                '--qx', '0.0', '--qy', '0.0', '--qz', '1.0', '--qw', '0.0',
                '--frame-id', 'world', '--child-frame-id', 'aruco_marker_110_frame',
            ]
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
            ]
        ),

        # ── Scene Camera ───────────────────────────────────────────
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
                'spatial_filter.enable': 'false',
                'temporal_filter.enable': 'false',
                'hole_filling_filter.enable': 'false',
                'decimation_filter.enable': 'false',
                'enable_sync': 'true',
                'publish_tf': 'false',
            }.items(),
        ),

        # ── Tray Camera (delayed 5 s to let scene camera claim USB first) ─
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'sleep 5 && exec env TRAY_CAM_SERIAL={tray_cam_serial} '
                f'ros2 launch tracking_pkg tray_camera_launch.py',
            ],
            output='screen',
        ),

        # ── ArUco Marker Manager ───────────────────────────────────
        Node(
            package='tracking_pkg',
            executable='aruco_marker_manager.py',
            name='aruco_marker_manager',
            output='screen',
            parameters=[{'publish_marker_static_tfs': False}],
        ),

        # ── Hand Tracker (MediaPipe, GPU) ──────────────────────────
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
                        # 1 hand ~halves MediaPipe CPU cost (runs on CPU/XNNPACK, not GPU).
                        'max_num_hands': 1,
                        # Use more of the ~25 Hz scene feed (capped by camera / MediaPipe rate).
                        'publish_rate_hz': 30.0,
                        'annotated_image_max_hz': 15.0,
                    }],
                    remappings=[
                        ('color_image', '/scene_camera/color/image_raw'),
                        ('depth_image', '/scene_camera/aligned_depth_to_color/image_raw'),
                        ('camera_info',  '/scene_camera/color/camera_info'),
                    ],
                ),
            ]
        ),

        # ── Tool Detection (YOLO OBB, GPU) ────────────────────────
        # Startup is staggered (5/9/13 s) so the heavy model loads (2x YOLO, Whisper,
        # MediaPipe) don't all hit the Orin's CPU/GPU at once — avoids the boot spike.
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='tool_detection_node.py',
                    name='tool_detection_node',
                    output='screen',
                    parameters=[{
                        'model_path': os.environ.get(
                            'OBB_MODEL_PATH',
                            os.path.join(os.path.expanduser('~'),
                                         'robotic_scrub_nurse_ws',
                                         'ros_unrelated_scripts', 'first_obb_test.pt'),
                        ),
                        'tray_camera_namespace': '/tray_camera',
                        'tray_camera_frame':     'tray_camera_color_optical_frame',
                        'world_frame':           'world',
                        'conf_threshold':        0.35,
                        'imgsz':                 1024,
                        'device':                os.environ.get('OBB_DEVICE', 'cuda:0'),
                        'handle_class_name':     'handle',
                        # 2 Hz keeps the continuous world model fresh for get_world_model
                        # (< tracker max_age 3 s so tool IDs stay stable) at a fraction of the
                        # GPU/CPU cost — the tray is mostly static, so 2 Hz is plenty.
                        'inference_rate_hz':     2.0,
                        'fixed_tool_plane_z_m':  0.04,
                        'publish_annotated_image': True,
                    }],
                ),
            ]
        ),

        # ── Grasp Geometry ────────────────────────────────────────
        Node(
            package='tracking_pkg',
            executable='grasp_geometry_node.py',
            name='grasp_geometry_node',
            output='screen',
            parameters=[{
                'world_frame': 'world',
                'grasp_offset_m': 0.035,
                'detections_topic': '/detected_tools_obb',
                'candidates_topic': '/tool_grasp_candidates',
            }],
        ),

        # ── Tool Semantics ────────────────────────────────────────
        Node(
            package='tracking_pkg',
            executable='tool_semantics_node.py',
            name='tool_semantics_node',
            output='screen',
            parameters=[{
                'input_topic':  '/tool_grasp_candidates',
                'output_topic': '/enriched_tool_grasp_candidates',
            }],
        ),

        # ── Reclaim Tray perception (2nd chain on the SCENE camera) ─
        # The scene camera doubles as the reclaim-tray camera (localized via marker
        # 105). Detection + grasp + semantics analogous to the instrument tray, on
        # distinct /reclaim_* topics, at a lower rate (reclaim = intermediate storage).
        # Runs alongside hand_tracker on the same camera. NOT wired into the world
        # model / execution yet (later phase).
        # TODO: set fixed_tool_plane_z_m to the measured reclaim tray surface height.
        TimerAction(
            period=11.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='tool_detection_node.py',
                    name='reclaim_tool_detection_node',
                    output='screen',
                    parameters=[{
                        'model_path': os.environ.get(
                            'OBB_MODEL_PATH',
                            os.path.join(os.path.expanduser('~'),
                                         'robotic_scrub_nurse_ws',
                                         'ros_unrelated_scripts', 'first_obb_test.pt'),
                        ),
                        'tray_camera_namespace': '/scene_camera',
                        'tray_camera_frame':     'scene_camera_color_optical_frame',
                        'world_frame':           'world',
                        'conf_threshold':        0.35,
                        'imgsz':                 1024,
                        'device':                os.environ.get('OBB_DEVICE', 'cuda:0'),
                        'handle_class_name':     'handle',
                        # Slow "stream" — one frame every 5 s. Reclaim = intermediate
                        # storage (not time-critical); keeps the RViz annotation updating
                        # at negligible load. NOTE: if the reclaim is later wired into a
                        # tracker/world model, set that tracker's max_age > 5 s.
                        'inference_rate_hz':     0.2,
                        'fixed_tool_plane_z_m':  0.04,
                        'publish_annotated_image': True,
                        'detections_topic':      '/reclaim_tools_obb',
                        'annotated_topic':       '/reclaim_detection/annotated_image',
                    }],
                ),
            ]
        ),
        Node(
            package='tracking_pkg',
            executable='grasp_geometry_node.py',
            name='reclaim_grasp_geometry_node',
            output='screen',
            parameters=[{
                'world_frame':      'world',
                'grasp_offset_m':   0.035,
                'detections_topic': '/reclaim_tools_obb',
                'candidates_topic': '/reclaim_grasp_candidates',
            }],
        ),
        Node(
            package='tracking_pkg',
            executable='tool_semantics_node.py',
            name='reclaim_tool_semantics_node',
            output='screen',
            parameters=[{
                'input_topic':  '/reclaim_grasp_candidates',
                'output_topic': '/reclaim_enriched_grasp_candidates',
            }],
        ),

        # ── World Model ───────────────────────────────────────────
        Node(
            package='tracking_pkg',
            executable='world_model_node.py',
            name='world_model_node',
            output='screen',
            parameters=[{
                'track_distance_threshold_m': 0.05,
                'track_max_age_sec':          3.0,
                'candidates_topic':           '/enriched_tool_grasp_candidates',
                'hand_state_topic':           '/hand_state',
                'world_frame':               'world',
                'hand_confidence_threshold':  0.3,
            }],
        ),

        # ── World Model Builder — REMOVED from the production launch ──
        # world_model_builder (/build_world_model) is only called by the manual
        # test node tool_pick_test_node.cpp (see tool_pick_test_launch.py); the LLM
        # uses world_model_node's /get_world_model instead. In streaming mode it
        # converts the tray image at camera rate (cv_bridge) and cost ~63% CPU +
        # ~1.6 GB here for nothing. Run tool_pick_test_launch.py if you need it.

        # ── Collision Publishers — MOVED to the NUC (nuc_launch.py) ──
        # They publish static geometry to move_group (which runs on the NUC) with
        # latched (TRANSIENT_LOCAL) QoS, so they belong next to move_group: keeps
        # them off the Jetson CPU and saves a DDS hop. The instrument-tray one
        # needs world→tray_camera_color_optical_frame, which still arrives on the
        # NUC from this node's aruco_marker_manager over DDS.

        # ── Sound ─────────────────────────────────────────────────
        Node(
            package='tracking_pkg',
            executable='handover_sound_publisher.py',
            name='handover_sound_publisher',
            output='screen',
        ),

        # ── ASR (Whisper — CPU on the Jetson, ctranslate2 has no CUDA build) ──
        # Loaded last (13 s) and thread-capped so it doesn't peg all 6 cores /
        # starve sshd during startup.
        TimerAction(
            period=13.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='asr_node.py',
                    name='asr_node',
                    output='screen',
                    parameters=[{
                        'whisper_model':               'base.en',
                        'language':                    'en',
                        'silence_threshold_seconds':   0.5,
                        'cpu_threads':                 3,
                        # Samson Q2U USB mic via direct ALSA (PulseAudio exposes no
                        # capture source for it, so the default device is silent).
                        'audio_device':                'Samson',
                    }],
                ),
            ]
        ),

        # ── LLM Orchestrator ──────────────────────────────────────
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='llm_orchestrator_node.py',
                    name='llm_orchestrator_node',
                    output='screen',
                    parameters=[{
                        'model_name':        'gpt-4o-mini',
                        'max_tool_turns':    8,
                        'action_timeout_sec': 120.0,
                    }],
                ),
            ]
        ),
    ])
