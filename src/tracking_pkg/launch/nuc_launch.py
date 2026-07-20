"""
NUC Launch File — Robot Control Layer
======================================
Runs on the Intel NUC (Humble, x86_64).
Handles UR driver, MoveIt, skill execution, and gripper.

Perception / AI nodes, including ASR and automatic microphone selection, run
separately on the Jetson via jetson_launch.py. The NUC does not open an audio
input. Both machines must share the same ROS_DOMAIN_ID and use CycloneDDS.

Env vars:
  UR_ROBOT_IP   IP of the UR controller (default: 192.168.0.100)
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
    ur_type      = LaunchConfiguration('ur_type')
    tracking_rviz = LaunchConfiguration('tracking_rviz')
    launch_servo = LaunchConfiguration('launch_servo')
    use_sim_time = LaunchConfiguration('use_sim_time')

    profile_config = PathJoinSubstitution(
        [FindPackageShare('tracking_pkg'), 'config', 'loop_mover_profiles.yaml']
    )

    # ── MoveIt (move_group) — standard ur_moveit_config ───────────
    # Starts ONLY move_group (correct robot_description for the installed UR
    # packages). The UR driver (ur_control.launch.py, robot_ip:=…) is started
    # separately by the operator. NOTE: the vendored rsn_ur_moveit.launch.py +
    # rsn_ur.urdf.xacro (from jazzy-spark) reference an old UR layout
    # ($(find ur_robot_driver)/urdf/ur.ros2_control.xacro, moved to ur_description
    # + changed macro signature) and are broken against UR 2.5/2.7 — do not use.
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py']
            )
        ),
        launch_arguments={
            'ur_type':      ur_type,
            'launch_rviz':  'false',
            'launch_servo': launch_servo,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ── Robot description (for RViz on NUC) — standard UR sources ─
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
        name='rviz2_nuc',
        output='screen',
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

        DeclareLaunchArgument('ur_type',       default_value='ur3e'),
        # RViz lives on the NUC (the Jetson is headless). Default on, like post_sfits;
        # pass tracking_rviz:=false for a headless NUC (e.g. over SSH without a display).
        DeclareLaunchArgument('tracking_rviz', default_value='true'),
        DeclareLaunchArgument('launch_servo',  default_value='false'),
        DeclareLaunchArgument('use_sim_time',  default_value='false'),
        SetEnvironmentVariable('LC_NUMERIC', 'en_US.UTF-8'),

        # ── Layer -1: UR driver + MoveIt ──────────────────────────
        ur_moveit_launch,

        # ── Layer 0: Static TF (world → base) ─────────────────────
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

        # ── Layer 5: Execution ────────────────────────────────────
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='tracking_pkg',
                    executable='skill_executor_node',
                    name='skill_executor_node',
                    output='screen',
                    parameters=[{
                        # Grasp TCP = fixed_tool_plane_z_m (jetson_launch) + this.
                        # Both trays are measured, so the plane now IS the tray
                        # surface and the gripper grasps right at it — which is what
                        # a flat instrument needs (the fingers must reach the surface
                        # to close around it sideways). Hence ~0.
                        #   instrument: +0.044 + 0.000 = +0.044
                        #   reclaim:    -0.137 + 0.001 = -0.136
                        # Both are exactly where grasping already worked. If you
                        # retune a plane for x/y accuracy, move the matching offset
                        # by the same amount in reverse or you change grasp depth.
                        'z_offset':                          0.000,
                        'reclaim_z_offset':                  0.001,
                        'approach_height_m':                 0.04,
                        'tool_yaw_offset_rad':               1.57079632679,
                        # Dynamic instrument-tray IK must remain elbow-up. If a
                        # complete approach/descend/lift path with elbow >= 0 is
                        # unavailable, the executor does not close the gripper.
                        'instrument_pick.elbow_min_rad':     0.0,
                        # One local re-plan from the unchanged lower reclaim
                        # stage before a failed pre-flight retreats to Home.
                        'reclaim_preflight_attempts':        2,
                        # Speed tuning 2026-07: 0.6 -> 0.8 (~25% faster
                        # motions). Handover scales stay 0.6 on purpose —
                        # that leg moves toward the surgeon's hand.
                        'velocity_scale':                    0.8,
                        'acceleration_scale':                0.8,
                        # Robotiq finishes open/close in ~0.5-0.7 s; the old
                        # 1.0 s slept ~1-1.5 s extra per pick sequence.
                        'gripper_pause_seconds':             0.6,
                        'handover_planning_time':            1.0,
                        'handover_velocity_scale':           0.6,
                        'handover_acceleration_scale':       0.6,
                        'pre_release_dwell_seconds':         0.15,
                        'post_zeroer_settle_seconds':        0.0,
                        'post_open_pause_seconds':           0.6,
                        'return_home_after_handover':        True,
                        'gripper_done_timeout_seconds':      30.0,
                        'cartesian_min_fraction':            0.95,
                        'gesture_wait_timeout_sec':          0.0,
                        'post_gesture_settle_sec':           0.5,
                        'return_release_height_m':           0.005,
                        'hand_offset':                       [-0.08, 0.0, 0.08],
                        'handover_orientation':              [-0.63, 0.63, -0.321, 0.321],
                        # home = the hub. Over the instrument tray, right of centre.
                        # All instrument tools are reachable from here by direct
                        # planning, and a pan rotation reaches the handover pose. The
                        # arm drives here once at launch.
                        'home_joints': [0.7702576518, -1.9044758282, 1.8983271758,
                                        -1.5910726986, -1.5716832320, 0.8188708425],
                        # Transit pose over the LEFT side of the instrument tray
                        # (counterpart of home, which is over the right). Placing a
                        # tool on a left slot routes through here, not home.
                        'instrument_left_stage_joints':
                            [2.1318871975, -1.1601789457, 1.1124246756,
                             -1.5276912202, -1.5967219512, 2.1804935932],
                        # ReturnToolHome-only deterministic post-lift arm pose.
                        # shoulder through wrist_2 remain fixed; wrist_3 keeps
                        # the planned lift-end value because tool roll is
                        # irrelevant here. The complete corridor is pre-planned.
                        'instrument_stage_joints':
                            [4.8766698837, -1.1527752441, 1.1332219283,
                             -1.5510326673, -1.5708482901, -2.9439778964],
                        'present_shoulder_pan_rad':          3.36332313,
                        'present_wrist1_rad':               -1.5248240244,
                        'present_wrist2_rad':               -1.2305892150,
                        'present_wrist3_rad':               -1.507562509029,

                        # ── Reclaim staging ───────────────────────────────────
                        # The reclaim tray sits under a 60 cm camera post, so the
                        # arm enters/leaves through two taught height poses over the
                        # tray (same x/y, different z): upper is post-clear (box is
                        # (de)attached there), lower is the grasp/place launch pad.
                        # Taught with joint_state_jogger_node + read_stage_pose.py.
                        'reclaim_stage_upper_joints':
                            [4.8814082146, -1.1096825761, 1.2962282340,
                             -1.7339645825, -1.5696294943, 0.1715736389],
                        'reclaim_stage_lower_joints':
                            [4.8818922043, -0.8805474800, 1.6040924231,
                             -2.2710281811, -1.5696328322, 0.1718008518],
                        # Left/right boundary of the instrument tray (world-x, tray
                        # centre = 0.0). Decides two things: a tool picked to the
                        # RIGHT is presented via home (left turns to the surgeon
                        # directly), and a tool placed back on a RIGHT slot transits
                        # via home (a left slot via instrument_left_stage_joints).
                        'instrument_right_side_x':           0.0,
                        # Tool classes whose held-tool collision box is built
                        # reversed (long reach toward the handle) — gripped near the
                        # functional end, so the body runs backward from the jaws.
                        'reversed_tool_box_classes':         ['hammer'],
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
            parameters=[
                profile_config,
                {
                    # Exclusive thin-tool rescue window. Retractor grasps have
                    # been observed at gPO=227; empty close is approximately 230.
                    'grasp_check.rescue_min_pos': 180,
                    'grasp_check.rescue_max_pos': 228,
                    # Require two conclusive negative monitor samples 100 ms
                    # apart before stopping a tool-carrying trajectory.
                    'grasp_check.loss_confirm_delay_sec': 0.1,
                },
            ],
        ),
        Node(
            package='tracking_pkg',
            executable='reclaim_controller.py',
            name='reclaim_controller',
            output='screen',
            parameters=[profile_config],
        ),

        # ── Collision Publishers (moved here from the Jetson) ─────
        # Publish /collision_object to the local move_group. Latched
        # (TRANSIENT_LOCAL) QoS means one publish reaches a late/restarting
        # move_group; kept at a slow 0.2 Hz so the objects are automatically
        # re-added if the planning scene is ever cleared. The instrument tray
        # needs world→tray_camera_color_optical_frame, which arrives from the
        # Jetson's aruco_marker_manager over DDS (its TF-wait loop handles the
        # startup ordering). mir/reclaim are pure world-frame geometry (no TF).
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
            parameters=[{'publish_hz': 0.2}],
        ),
        Node(
            package='tracking_pkg',
            executable='reclaim_tray_collision_publisher.py',
            name='reclaim_tray_collision_publisher',
            output='screen',
            # The tray's support surface IS in the scene (include_bottom_bar), sunk
            # 4 cm below the real surface so the descend keeps clearance. This only
            # works with the shortened gripper cylinder from files/ur.urdf.xacro.
            parameters=[{'publish_hz': 0.2}],
        ),

        # ── RViz (optional) ───────────────────────────────────────
        TimerAction(
            period=2.0,
            actions=[rviz_node],
        ),

        # ── HRI traffic-light display (surgeon-facing) ────────────
        # A large standalone window; drag it in front of RViz. Pure subscriber,
        # no effect on the control path. /asr_status arrives over DDS from the
        # Jetson's asr_node.
        Node(
            package='tracking_pkg',
            executable='hri_display_node.py',
            name='hri_display_node',
            output='screen',
            parameters=[{
                'canvas_width': 2560,
                'canvas_height': 1440,
            }],
        ),

        # ── Suppress move_group planning_scene_monitor INFO spam ──
        ExecuteProcess(
            cmd=['bash', '-c',
                 'sleep 12 && '
                 'ros2 service call /move_group/set_logger_level '
                 'rcl_interfaces/srv/SetLoggerLevel '
                 '"{logger_name: \'moveit.ros.planning_scene_monitor\', level: 30}" '
                 '> /dev/null 2>&1 ; '
                 'ros2 service call /move_group/set_logger_level '
                 'rcl_interfaces/srv/SetLoggerLevel '
                 '"{logger_name: \'moveit.moveit.ros.planning_scene_monitor\', level: 30}" '
                 '> /dev/null 2>&1 ; true'],
            output='log',
        ),
    ])
