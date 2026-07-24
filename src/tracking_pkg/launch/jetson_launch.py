"""
Launch the perception and AI layer on the Jetson Orin Nano.

Runs on the NVIDIA Jetson Orin Nano (Humble, ARM64, CUDA / JetPack).
Handles cameras, detection, hand tracking, grasp reasoning, world model, ASR and LLM.

Robot control (UR driver, MoveIt, skill execution) runs separately on the NUC via
nuc_launch.py. Both machines must share the same ROS_DOMAIN_ID and use CycloneDDS.

Env vars:
  RECLAIM_TRAY_CAM_SERIAL   Serial of the reclaim tray camera  (default: 239222300719)
  TRAY_CAM_SERIAL    Serial of the tray camera   (default: 239222302690)
  INSTRUMENT_TRAY_MODEL_PATH  Instrument-tray YOLO OBB model
  RECLAIM_TRAY_MODEL_PATH     Reclaim-tray YOLO OBB model
  OBB_DEVICE                  YOLO inference device (default: cuda:0)
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
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _default_model_path(filename):
    """Find model weights in either supported workspace directory layout."""
    home = os.path.expanduser('~')
    candidates = [
        os.path.join(home, 'robotic_scrub_nurse', 'ros_unrelated_scripts', filename),
        os.path.join(home, 'robotic_scrub_nurse_ws', 'ros_unrelated_scripts', filename),
    ]
    return next((path for path in candidates if os.path.isfile(path)), candidates[0])


def generate_launch_description():

    reclaim_tray_cam_serial = os.environ.get('RECLAIM_TRAY_CAM_SERIAL', '239222300719')
    tray_cam_serial = os.environ.get('TRAY_CAM_SERIAL', '239222302690')

    instrument_tray_model_path = os.environ.get(
        'INSTRUMENT_TRAY_MODEL_PATH',
        _default_model_path('instrument_tray_detector.pt'),
    )
    reclaim_tray_model_path = os.environ.get(
        'RECLAIM_TRAY_MODEL_PATH',
        _default_model_path('reclaim_tray_detector.pt'),
    )

    rs_launch_file = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    )
    wake_word_model_path = PathJoinSubstitution(
        [FindPackageShare('tracking_pkg'), 'models', 'robot.onnx']
    )

    # ASR parameters shared by both wake-word modes. The acoustic node adds the
    # openWakeWord model (needs models/robot.onnx); the text-gate node runs without
    # it so the pipeline is testable before that model is trained — see the
    # 'use_wake_word' launch argument below.
    asr_params_common = {
        # tiny.en: ~1 s instead of base.en's ~2.7 s on CPU. The deterministic
        # NLU (fuzzy verbs/tools) plus the initial_prompt vocabulary bias absorb
        # its rougher raw accuracy. Revert to 'base.en' if mishearings get worse.
        'whisper_model':                'tiny.en',
        'language':                     'en',
        'silence_threshold_seconds':    0.35,
        'max_speech_seconds':           8.0,
        'cpu_threads':                  3,
        # PortAudio name substrings paired with native capture rates. Exactly one
        # is expected to be connected.
        'audio_device_candidates':      ['Samson', 'USB Composite Device'],
        'audio_device_candidate_rates': [16000, 48000],
        'wake_words':                   ['robot'],
    }

    # RViz runs on the NUC (which has the robot model + planning scene natively).
    # The Jetson is headless — it must NOT generate a robot_description here: that
    # would pull rsn_ur.urdf.xacro / the UR ros2_control xacro (not needed for
    # perception, and version-fragile), and any failure aborts the whole launch.

    return LaunchDescription([

        DeclareLaunchArgument('ur_type',       default_value='ur3e'),
        # Acoustic openWakeWord gate (needs models/robot.onnx). Set false to test
        # without the trained model: Whisper text-gate — say "robot", pause, then
        # the command. asr_node transcribes everything and matches "robot" as text.
        DeclareLaunchArgument(
            'use_wake_word', default_value='true',
            description='true: acoustic openWakeWord gate (needs '
                        'models/robot.onnx). false: Whisper text-gate for '
                        'testing without the model (say "robot", then command).'),
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

        # ── Reclaim Tray Camera ───────────────────────────────────────────
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
                'spatial_filter.enable': 'false',
                'temporal_filter.enable': 'false',
                'hole_filling_filter.enable': 'false',
                'decimation_filter.enable': 'false',
                'enable_sync': 'true',
                'publish_tf': 'false',
            }.items(),
        ),

        # ── Tray Camera (delayed 5 s to let reclaim tray camera claim USB first) ─
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
                        'camera_frame': 'reclaim_tray_camera_color_optical_frame',
                        'world_frame': 'world',
                        # 1 hand ~halves MediaPipe CPU cost (runs on CPU/XNNPACK, not GPU).
                        'max_num_hands': 1,
                        # Use more of the ~25 Hz camera feed (capped by camera / MediaPipe rate).
                        'publish_rate_hz': 30.0,
                        'annotated_image_max_hz': 15.0,
                    }],
                    remappings=[
                        ('color_image', '/reclaim_tray_camera/color/image_raw'),
                        ('depth_image', '/reclaim_tray_camera/aligned_depth_to_color/image_raw'),
                        ('camera_info',  '/reclaim_tray_camera/color/camera_info'),
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
                        'model_path': instrument_tray_model_path,
                        'tray_camera_namespace': '/tray_camera',
                        'tray_camera_frame':     'tray_camera_color_optical_frame',
                        'world_frame':           'world',
                        'conf_threshold':        0.35,
                        'imgsz':                 1024,
                        'device':                os.environ.get('OBB_DEVICE', 'cuda:0'),
                        'handle_class_name':     'handle',
                        # 4 Hz keeps the continuous world model fresh for get_world_model
                        # (<< tracker max_age 3 s so tool IDs stay stable). Raised from 2 Hz
                        # after the aruco unsubscribe + collision-pub move freed Orin CPU.
                        'inference_rate_hz':     4.0,
                        # Measured with tcp_probe.py + closed gripper: the tray
                        # surface is at +0.044, not the +0.040 assumed before.
                        # This plane is what every detected pixel is ray-cast onto,
                        # so it sets the grasp point's x/y as well as its z — it is
                        # NOT just a height knob. Grasp DEPTH is held constant by
                        # z_offset in nuc_launch.py: plane + z_offset = TCP.
                        'fixed_tool_plane_z_m':  0.044,
                        'location':              'instrument_tray',
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
                # Half the gripper's OUTER width across the tool axis (measured:
                # 65 mm outer edge to outer edge). These two points, not just the
                # grasp point, are what would hit a profile bar during the descent.
                'finger_half_span_m': 0.0325,
                # Step size when searching along the tool axis for a spot over the
                # tray opening.
                'slide_step_m':      0.002,
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

        # ── Reclaim Tray perception (2nd chain on reclaim_tray_camera) ─
        # reclaim_tray_camera also runs hand_tracker + marker-105 localization (above).
        # This adds reclaim-tray detection + grasp + semantics, analogous to the instrument
        # tray, on distinct /reclaim_* topics, at a lower rate (reclaim = intermediate
        # storage). Feeds world_model_node's reclaim_candidates_topic.
        TimerAction(
            period=11.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='tool_detection_node.py',
                    name='reclaim_tool_detection_node',
                    output='screen',
                    parameters=[{
                        'model_path': reclaim_tray_model_path,
                        'tray_camera_namespace': '/reclaim_tray_camera',
                        'tray_camera_frame':     'reclaim_tray_camera_color_optical_frame',
                        'world_frame':           'world',
                        'conf_threshold':        0.35,
                        # Reclaim tray camera streams native 640x480 (vs 1280x720 on the tray),
                        # so imgsz=640 matches the native resolution — no upscale to 1024,
                        # which buys no real detail and just wastes Orin compute. Re-validate
                        # (or match this imgsz in training) once the reclaim perspective is
                        # fine-tuned into the OBB model.
                        'imgsz':                 640,
                        'device':                os.environ.get('OBB_DEVICE', 'cuda:0'),
                        'handle_class_name':     'handle',
                        # Match the instrument-tray detector so both annotated tool feeds
                        # update consistently in RViz.
                        'inference_rate_hz':     4.0,
                        # Reclaim tray surface, measured with tcp_probe.py + closed
                        # gripper: -0.137 (the earlier -0.145 sat 8 mm too low).
                        # This matters more here than on the instrument tray: the
                        # reclaim camera looks at the tray from the SIDE, so a plane
                        # that is too low makes the ray overshoot and walks the grasp
                        # point LATERALLY away from the camera, not just downward.
                        # Grasp DEPTH is held constant by reclaim_z_offset in
                        # nuc_launch.py: plane + reclaim_z_offset = TCP.
                        'fixed_tool_plane_z_m': -0.137,
                        'location':              'reclaim_tray',
                        'publish_annotated_image': True,
                        'annotation_line_width_px': 1,
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
                # Half the gripper's OUTER width across the tool axis (measured:
                # 65 mm outer edge to outer edge). These two points, not just the
                # grasp point, are what would hit a profile bar during the descent.
                'finger_half_span_m': 0.0325,
                # Step size when searching along the tool axis for a spot over the
                # tray opening.
                'slide_step_m':      0.002,
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
                'reclaim_candidates_topic':   '/reclaim_enriched_grasp_candidates',
                # The reclaim tray changes slowly and the surgeon reaches over it
                # constantly; a 3 s eviction would drop tools on every occlusion.
                'reclaim_track_max_age_sec':  10.0,
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

        # ── Sound: intentionally NOT started — the Jetson has no speakers.
        # (handover_sound_publisher still exists for hosts that do; see
        # llm_launch.py. The HRI display carries the surgeon-facing feedback.)

        # ── ASR (openWakeWord + Whisper — CPU on the Jetson) ─────────────
        # Loaded last (13 s) and thread-capped so it doesn't peg all 6 cores /
        # starve sshd during startup. The node automatically selects the one
        # connected supported microphone and uses its native capture rate.
        # openWakeWord filters idle audio before Whisper; the spoken interaction
        # is deliberately two-stage: "Robot", a short pause, then one command.
        #
        # Two mutually-exclusive nodes gated by use_wake_word (only one launches):
        # acoustic (default, needs models/robot.onnx, fail-closed) vs. Whisper
        # text-gate for testing before that model exists.
        TimerAction(
            period=13.0,
            actions=[
                # Acoustic openWakeWord gate — the production path. Fails closed
                # if the enabled detector or its model cannot be loaded; there is
                # no Whisper fallback.
                Node(
                    package='tracking_pkg',
                    executable='asr_node.py',
                    name='asr_node',
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('use_wake_word')),
                    parameters=[{
                        **asr_params_common,
                        'audio_wake_enabled': True,
                        'audio_wake_model_path': ParameterValue(
                            wake_word_model_path, value_type=str),
                        'audio_wake_threshold': 0.5,
                        # Whisper verifies only the exact isolated wake token.
                        'wake_require_separate_command': True,
                    }],
                ),
                # Whisper text-gate — no ONNX needed. Whisper transcribes every
                # segment and matches "robot" as text (two-stage: say "robot",
                # pause, then the command). For testing before robot.onnx exists.
                Node(
                    package='tracking_pkg',
                    executable='asr_node.py',
                    name='asr_node',
                    output='screen',
                    condition=UnlessCondition(
                        LaunchConfiguration('use_wake_word')),
                    parameters=[{
                        **asr_params_common,
                        'audio_wake_enabled': False,
                    }],
                ),
            ],
        ),

        # ── Command router (deterministic, replaces the LLM orchestrator) ──
        # Verb lexicon + fuzzy synonym match, guards against the live world
        # model. LLM is only a stateless intent-classification fallback
        # (needs OPENAI_API_KEY; without a key the node runs fully offline).
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='command_router_node.py',
                    name='command_router_node',
                    output='screen',
                    parameters=[{
                        'fuzzy_threshold':      0.8,
                        'fuzzy_floor':          0.6,
                        'llm_fallback_enabled': True,
                        'model_name':           'gpt-5-mini',
                        'action_timeout_sec':   120.0,
                    }],
                ),
            ]
        ),
    ])
