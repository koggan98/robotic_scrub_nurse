"""
NUC Launch File — Robot Control Layer
======================================
Runs on the Intel NUC (Humble, x86_64).
Handles UR driver, MoveIt, skill execution, and gripper.

Perception / AI nodes run separately on the Jetson via jetson_launch.py.
Both machines must share the same ROS_DOMAIN_ID and use CycloneDDS.

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
                        'z_offset':                          0.004,
                        'approach_height_m':                 0.04,
                        'tool_yaw_offset_rad':               1.57079632679,
                        'velocity_scale':                    0.6,
                        'acceleration_scale':                0.6,
                        'gripper_pause_seconds':             1.0,
                        'handover_planning_time':            1.0,
                        'handover_velocity_scale':           0.6,
                        'handover_acceleration_scale':       0.6,
                        'pre_release_dwell_seconds':         0.3,
                        'post_zeroer_settle_seconds':        0.0,
                        'post_open_pause_seconds':           1.0,
                        'return_home_after_handover':        True,
                        'gripper_done_timeout_seconds':      30.0,
                        'cartesian_min_fraction':            0.95,
                        'gesture_wait_timeout_sec':          0.0,
                        'post_gesture_settle_sec':           0.5,
                        'return_release_height_m':           0.005,
                        # grasp_tool rises to this world z after lifting a tool off
                        # the reclaim tray, before the held-tool box is attached —
                        # clear of the tray bracket's top bar (z ~= +0.015).
                        'reclaim_hold_z_m':                  0.10,
                        'hand_offset':                       [-0.08, 0.0, 0.08],
                        'handover_orientation':              [-0.63, 0.63, -0.321, 0.321],
                        'home_joints': [-0.1601136366, -2.2975937329, 2.2748802344,
                                        -1.5248240244, -1.2305892150, -4.8166621367],
                        'present_shoulder_pan_rad':          3.36332313,
                        'present_wrist1_rad':               -1.5248240244,
                        'present_wrist2_rad':               -1.2305892150,
                        'present_wrist3_rad':               -1.507562509029,
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
            # include_bottom_bar defaults to False: modelling the tray's support
            # surface makes it ungraspable, because the gripper's collision
            # cylinder reaches 38 mm past the TCP. See the node for the geometry.
            parameters=[{'publish_hz': 0.2}],
        ),

        # ── RViz (optional) ───────────────────────────────────────
        TimerAction(
            period=2.0,
            actions=[rviz_node],
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
