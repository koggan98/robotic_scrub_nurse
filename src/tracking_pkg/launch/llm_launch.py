"""
LLM-based Robotic Scrub Nurse Launch File
==========================================
New architecture with natural language interaction, OBB-based tool detection,
grasp reasoning, world model, and LLM-driven execution.

Cameras use the official realsense2_camera ROS2 package.
Set SCENE_CAM_SERIAL / TRAY_CAM_SERIAL env vars or edit serial_no below.

Startup order:
  1. Static TFs (world→base, world→tray_camera_color_optical_frame)
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
from launch.actions import (
    DeclareLaunchArgument,
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
    ur_type = LaunchConfiguration('ur_type')
    tracking_rviz = LaunchConfiguration('tracking_rviz')
    launch_servo = LaunchConfiguration('launch_servo')
    use_sim_time = LaunchConfiguration('use_sim_time')

    profile_config = PathJoinSubstitution(
        [FindPackageShare('tracking_pkg'), 'config', 'loop_mover_profiles.yaml']
    )
    rs_launch_file = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py']
    )

    # ── Camera serial numbers ─────────────────────────────────
    # Set via environment variable or hardcode here.
    scene_cam_serial = os.environ.get('SCENE_CAM_SERIAL', '239222300719')
    tray_cam_serial = os.environ.get('TRAY_CAM_SERIAL', '239222302690')

    # ── UR MoveIt include ─────────────────────────────────────
    # Brings up ur_robot_driver, ros2_control, MoveIt2, and publishes
    # robot_description / robot_description_semantic on the parameter server.
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py']
            )
        ),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': 'false',
            'launch_servo': launch_servo,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ── Robot description (for our own RViz node) ─────────────
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ',
                PathJoinSubstitution(
                    [FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']
                ),
                ' ur_type:=', ur_type,
                ' name:=ur',
                ' prefix:=',
            ]),
            value_type=str,
        )
    }
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            Command([
                'xacro ',
                PathJoinSubstitution(
                    [FindPackageShare('ur_moveit_config'), 'srdf', 'ur.srdf.xacro']
                ),
                ' ur_type:=', ur_type,
                ' name:=ur',
            ]),
            value_type=str,
        )
    }
    robot_description_kinematics_path = PathJoinSubstitution(
        [FindPackageShare('ur_moveit_config'), 'config', 'kinematics.yaml']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_llm',
        output='log',
        condition=IfCondition(tracking_rviz),
        arguments=[
            '-d',
            PathJoinSubstitution(
                [FindPackageShare('tracking_pkg'), 'rviz', 'view_robot_tracking.rviz']
            ),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics_path,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([

        DeclareLaunchArgument('ur_type', default_value='ur3e'),
        DeclareLaunchArgument('tracking_rviz', default_value='true'),
        DeclareLaunchArgument('launch_servo', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        SetEnvironmentVariable('LC_NUMERIC', 'en_US.UTF-8'),

        # ── Layer -1: UR driver + MoveIt ──────────────────────────
        ur_moveit_launch,


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
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_tray_camera_tf',
            output='screen',
            arguments=[
                '--x', '-0.075', '--y', '0.349', '--z', '0.4325',
                '--qx', '0.0', '--qy', '1.0', '--qz', '0.0', '--qw', '0.0',
                '--frame-id', 'world', '--child-frame-id', 'tray_camera_color_optical_frame',
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
                'publish_tf': 'false',
            }.items(),
        ),

        # ── Layer 1b: Tray Camera (realsense2_camera) ─────────────
        # Publishes /tray_camera/color/image_raw,
        #           /tray_camera/aligned_depth_to_color/image_raw,
        #           /tray_camera/color/camera_info.
        # Consumed by both tool_detection_node (continuous streaming) and
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
                'publish_tf': 'false',
            }.items(),
        ),

        # ── Layer 2: ArUco Marker Manager ─────────────────────────
        # Loads config/aruco_markers.yaml and handles:
        #   - Static TFs (marker frames + tool_holder_frame)
        #   - Camera-pose detection (one-shot static TFs per marker)
        Node(
            package='tracking_pkg',
            executable='aruco_marker_manager.py',
            name='aruco_marker_manager',
            output='screen',
            parameters=[{
                'publish_marker_static_tfs': False,
            }],
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
                        'camera_frame': 'scene_camera_color_optical_frame',
                        'world_frame': 'world',
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
        # Continuous OBB perception: subscribes to /tray_camera/*, runs
        # YOLOv8-OBB at inference_rate_hz, publishes ToolDetectionArray on
        # /detected_tools_obb. No grasp / semantics — that's downstream.
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='tool_detection_node.py',
                    name='tool_detection_node',
                    output='screen',
                    parameters=[{
                        'model_path': os.environ.get(
                            'OBB_MODEL_PATH',
                            '/home/mir/robotic_scrub_nurse_ws/ros_unrelated_scripts/first_obb_test.pt',
                        ),
                        'tray_camera_namespace': '/tray_camera',
                        'tray_camera_frame': 'tray_camera_color_optical_frame',
                        'world_frame': 'world',
                        'conf_threshold': 0.35,
                        'imgsz': 1024,
                        'device': 'cpu',
                        'handle_class_name': 'handle',
                        'inference_rate_hz': 5.0,
                        # Fixed-plane projection while RealSense is mounted
                        # closer than its min depth. Set to NaN to revert.
                        'fixed_tool_plane_z_m': 0.038,
                        'publish_annotated_image': True,
                    }],
                ),
            ]
        ),

        # ── Layer 4: Reasoning ────────────────────────────────────
        # Continuous grasp geometry: consumes /detected_tools_obb, derives
        # tool long axis + grasp point + functional_end_direction in world
        # frame, publishes GraspCandidateArray on /tool_grasp_candidates.
        # No semantics (handover_rule stays empty — filled by Part 3).
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

        # Semantic enrichment: looks up tool_class in tool_knowledge_base.yaml,
        # fills handover_rule + display_name + grip_strategy + functional_end_label
        # + handover_description + lift_height per GraspCandidate. Republishes
        # on /enriched_tool_grasp_candidates.
        Node(
            package='tracking_pkg',
            executable='tool_semantics_node.py',
            name='tool_semantics_node',
            output='screen',
            parameters=[{
                'input_topic': '/tool_grasp_candidates',
                'output_topic': '/enriched_tool_grasp_candidates',
            }],
        ),

        # World model aggregator: caches enriched candidates + hand state +
        # joint positions + system state, assigns persistent tool_ids via
        # XY-distance tracking, exposes:
        #   /get_world_state    (SystemState struct, typed ROS)
        #   /get_world_model    (JSON snapshot for LLM)
        #   /get_tool_candidates (filtered candidate list)
        Node(
            package='tracking_pkg',
            executable='world_model_node.py',
            name='world_model_node',
            output='screen',
            parameters=[{
                'track_distance_threshold_m': 0.05,
                'track_max_age_sec': 3.0,
                'candidates_topic': '/enriched_tool_grasp_candidates',
                'hand_state_topic': '/hand_state',
                'world_frame': 'world',
                'hand_confidence_threshold': 0.3,
            }],
        ),

        # On-demand world model builder: triggered via /build_world_model service.
        # Runs YOLOv8-OBB inference, pairs handle/body OBBs, computes grasp
        # points + orientations, and returns a JSON snapshot for the LLM.
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='world_model_builder.py',
                    name='world_model_builder',
                    output='screen',
                    parameters=[{
                        'model_path': os.environ.get(
                            'OBB_MODEL_PATH',
                            '/home/mir/robotic_scrub_nurse_ws/ros_unrelated_scripts/first_obb_test.pt',
                        ),
                        # Streaming mode: builder subscribes to /tray_camera/*
                        # alongside tool_detection_node. Shared realsense2_camera
                        # node owns the hardware.
                        'camera_mode': 'streaming',
                        'tray_camera_namespace': '/tray_camera',
                        'tray_camera_frame': 'tray_camera_color_optical_frame',
                        'world_frame': 'world',
                        'conf_threshold': 0.35,
                        'imgsz': 1024,
                        'device': 'cpu',
                        'handle_class_name': 'handle',
                        'grasp_offset_fraction': 1.0 / 16.0,
                        'fixed_tool_plane_z_m': 0.05,
                        'max_image_age_sec': 1.0,
                    }],
                ),
            ]
        ),

        # ── Layer 4b: Collision objects for MoveIt planning scene ─
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
                'start_offset_m': -0.10,
                'publish_hz': 2.0,
            }],
        ),

        # ── Layer 5: Execution ────────────────────────────────────
        # Skill executor: four ROS Actions wrapping MoveIt pick/handover/
        # release/return_home; reuses the same parameters as
        # tool_pick_test_node. Publishes /system_state_update so
        # world_model_node can surface state in /get_world_model.
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='skill_executor_node',
                    name='skill_executor_node',
                    output='screen',
                    parameters=[{
                        'approach_height_m': 0.05,
                        'tool_yaw_offset_rad': 1.57079632679,
                        'velocity_scale': 0.6,
                        'acceleration_scale': 0.6,
                        'gripper_pause_seconds': 1.0,
                        'handover_planning_time': 1.0,
                        'handover_velocity_scale': 0.6,
                        'handover_acceleration_scale': 0.6,
                        'pre_release_dwell_seconds': 0.3,
                        'post_zeroer_settle_seconds': 0.0,
                        'post_open_pause_seconds': 1.0,
                        'return_home_after_handover': True,
                        'gripper_done_timeout_seconds': 30.0,
                        # Pre-flight cartesian path tolerance (was 0.99,
                        # too strict — caused mid-pick aborts).
                        'cartesian_min_fraction': 0.95,
                        # Handover waits for the surgeon's double_open_close
                        # gesture before moving to the hand. 0.0 = wait
                        # indefinitely.
                        'gesture_wait_timeout_sec': 0.0,
                        'post_gesture_settle_sec': 0.5,
                        'hand_offset': [-0.08, 0.0, 0.05],
                        'handover_orientation': [-0.63, 0.63, -0.321, 0.321],
                        'home_joints': [-0.1601136366, -2.2975937329, 2.2748802344,
                                        -1.5248240244, -1.2305892150, -4.8166621367],
                    }],
                ),
            ]
        ),

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
                        'whisper_model': 'base.en',
                        'language': 'en',
                        'silence_threshold_seconds': 0.5,
                    }],
                ),
            ]
        ),

        # ── Layer 7: LLM ─────────────────────────────────────────
        # High-level planner: consumes /user_speech, runs an OpenAI
        # tool-calling loop, drives the skill actions. Needs OPENAI_API_KEY
        # in the environment (or openai_api_key parameter).
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='llm_orchestrator_node.py',
                    name='llm_orchestrator_node',
                    output='screen',
                    parameters=[{
                        'model_name': 'gpt-4o-mini',
                        'max_tool_turns': 8,
                        'action_timeout_sec': 120.0,
                    }],
                ),
            ]
        ),

        # ── Layer 8: RViz (optional, conditional on tracking_rviz) ─
        TimerAction(
            period=2.0,
            actions=[rviz_node],
        ),
    ])
