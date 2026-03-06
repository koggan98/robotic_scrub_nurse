#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Int32, String

try:
    from rtde_control import RTDEControlInterface as RTDEControl
    from rtde_receive import RTDEReceiveInterface as RTDEReceive
except ImportError:
    RTDEControl = None
    RTDEReceive = None


STRATEGY_STANDARD = "standard"
STRATEGY_HAMMER_CARTESIAN = "hammer_cartesian"
STRATEGY_LONG_SCISSOR = "long_scissor"
STRATEGY_RETRACTOR_CARTESIAN = "retractor_cartesian"


@dataclass
class ToolProfile:
    tool_id: str
    strategy: str
    pick_position: List[float]
    pick_orientation: List[float]  # quaternion [x, y, z, w]
    hand_offset: List[float]
    handover_orientation: List[float]  # quaternion [x, y, z, w]
    lift_height: float


@dataclass
class MotionProfiles:
    home_joints: List[float]
    long_scissor_over_joints: List[float]
    long_scissor_after_joints: List[float]
    long_scissor_elbow_target: float
    long_scissor_elbow_tolerance_above: float
    long_scissor_elbow_tolerance_below: float


@dataclass
class ForceSensingConfig:
    axis_threshold_newtons: float
    force_norm_threshold_newtons: float
    torque_norm_threshold_newton_meters: float
    zero_delay_seconds: float
    poll_rate_hz: float
    debug_log_interval_seconds: float


class SocketMover(Node):
    def __init__(self) -> None:
        super().__init__("socket_mover")

        self._declare_rtde_parameters()
        self._declare_motion_limits()

        self.pre_release_dwell_seconds = max(
            0.0, self.declare_parameter("handover.pre_release_dwell_seconds", 0.35).value
        )
        self.post_open_pause_seconds = max(
            0.0, self.declare_parameter("handover.post_open_pause_seconds", 1.0).value
        )

        self.configuration_loaded = False
        self.tool_profiles: Dict[str, ToolProfile] = {}
        self.motion_profiles: Optional[MotionProfiles] = None

        self.reclaim_dropoff_position: List[float] = [0.0, 0.0, 0.0]
        self.reclaim_dropoff_orientation: List[float] = [0.0, 0.0, 0.0, 1.0]
        self.reclaim_post_close_wait_seconds = 1.0
        self.reclaim_dropoff_descend_distance = 0.05
        self.reclaim_post_open_pause_seconds = 1.0

        self.tool_position = [0.0, 0.0, 0.0]
        self.tool_orientation = [0.0, 0.0, 0.0, 1.0]
        self.handover_orientation = [0.0, 0.0, 0.0, 1.0]
        self.hand_offset = [0.0, 0.0, 0.0]
        self.active_tool_profile: Optional[ToolProfile] = None

        self.hand_pose = Pose()
        self.hand_pose_with_offset = Pose()
        self.last_handover_pose = Pose()

        self.hand_pose_received = False
        self.waiting_for_hand_pose = False
        self.waiting_for_gripper_done = False
        self.waiting_for_reclaim_done = False
        self.tool_has_been_picked_up = False
        self.has_last_handover_pose = False
        self.reclaim_in_progress = False
        self.handover_force_config: Optional[ForceSensingConfig] = None
        self.reclaim_force_config: Optional[ForceSensingConfig] = None

        self._declare_configuration_parameters()
        self.configuration_loaded = self._load_configuration()

        self.tool_selection_sub = self.create_subscription(
            String,
            "/tool_selection",
            self._tool_selection_callback,
            10,
        )
        self.hand_pose_sub = self.create_subscription(
            Pose,
            "/hand_pose",
            self._hand_pose_callback,
            10,
        )

        self.gripper_mover_pub = self.create_publisher(Bool, "/gripper_mover", 10)
        self.gripper_position_command_pub = self.create_publisher(Int32, "/gripper_position_command", 10)
        self.handover_event_pub = self.create_publisher(String, "/handover_event", 10)

        self.rtde_control = None
        self.rtde_receive = None
        self.rtde_ready = False
        self._initialize_rtde_interfaces()

        self.get_logger().info("Socket mover node initialized.")

    def _declare_rtde_parameters(self) -> None:
        self.robot_ip = self.declare_parameter("rtde.robot_ip", "192.168.12.10").value
        self.use_external_control = bool(
            self.declare_parameter("rtde.use_external_control", False).value
        )
        pose_frame_rotation_rpy = self.declare_parameter(
            "rtde.input_pose_frame_rotation_rpy",
            [0.0, 0.0, math.pi],
        ).value
        if not isinstance(pose_frame_rotation_rpy, list) or len(pose_frame_rotation_rpy) != 3:
            raise ValueError("rtde.input_pose_frame_rotation_rpy must contain exactly 3 values")
        self.pose_frame_rotation_rpy = [float(v) for v in pose_frame_rotation_rpy]
        self.pose_frame_rotation_quat = self._rpy_to_quaternion(self.pose_frame_rotation_rpy)
        tcp_pose = self.declare_parameter(
            "rtde.tcp_pose",
            [0.0, 0.0, 0.235, 0.0, 0.0, 0.0],
        ).value
        if not isinstance(tcp_pose, list) or len(tcp_pose) != 6:
            raise ValueError("rtde.tcp_pose must contain exactly 6 values")
        self.tcp_pose = [float(v) for v in tcp_pose]

    def _declare_motion_limits(self) -> None:
        self.max_joint_speed = float(self.declare_parameter("motion.max_joint_speed", 1.80).value)
        self.max_joint_acceleration = float(
            self.declare_parameter("motion.max_joint_acceleration", 2.50).value
        )
        self.max_linear_speed = float(self.declare_parameter("motion.max_linear_speed", 0.40).value)
        self.max_linear_acceleration = float(
            self.declare_parameter("motion.max_linear_acceleration", 1.80).value
        )

    def _declare_configuration_parameters(self) -> None:
        self.declare_parameter("tool_ids", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("motion_profiles.home_joints", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter(
            "motion_profiles.long_scissor.over_joints",
            Parameter.Type.DOUBLE_ARRAY,
        )
        self.declare_parameter(
            "motion_profiles.long_scissor.after_joints",
            Parameter.Type.DOUBLE_ARRAY,
        )
        self.declare_parameter("motion_profiles.long_scissor.elbow_target", float("nan"))
        self.declare_parameter(
            "motion_profiles.long_scissor.elbow_tolerance_above", float("nan")
        )
        self.declare_parameter(
            "motion_profiles.long_scissor.elbow_tolerance_below", float("nan")
        )
        self.declare_parameter("handover.force_axis_threshold_newtons", 2.0)
        self.declare_parameter("handover.force_norm_threshold_newtons", 2.0)
        self.declare_parameter("handover.torque_norm_threshold_newton_meters", 0.12)
        self.declare_parameter("handover.zero_delay_seconds", 0.0)
        self.declare_parameter("handover.force_poll_rate_hz", 50.0)
        self.declare_parameter("handover.force_debug_log_interval_seconds", 1.0)
        self.declare_parameter("reclaim.dropoff_position", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter(
            "reclaim.dropoff_orientation",
            Parameter.Type.DOUBLE_ARRAY,
        )
        self.declare_parameter("reclaim.force_threshold_newtons", 2.0)
        self.declare_parameter("reclaim.force_norm_threshold_newtons", 2.0)
        self.declare_parameter("reclaim.torque_norm_threshold_newton_meters", 0.12)
        self.declare_parameter("reclaim.zero_delay_seconds", 0.5)
        self.declare_parameter("reclaim.force_poll_rate_hz", 50.0)
        self.declare_parameter("reclaim.close_position", 250)
        self.declare_parameter("reclaim.close_speed", 255)
        self.declare_parameter("reclaim.close_force", 1)
        self.declare_parameter("reclaim.post_close_wait_seconds", 1.0)
        self.declare_parameter("reclaim.dropoff_descend_distance", 0.05)
        self.declare_parameter("reclaim.post_open_pause_seconds", 1.0)

        tool_ids = self.get_parameter("tool_ids").value
        if not isinstance(tool_ids, list):
            tool_ids = []
        for tool_id in tool_ids:
            self._declare_tool_profile_parameters(str(tool_id))

    def _declare_tool_profile_parameters(self, tool_id: str) -> None:
        prefix = f"tool_profiles.{tool_id}."
        if not self.has_parameter(prefix + "strategy"):
            self.declare_parameter(prefix + "strategy", "")
        if not self.has_parameter(prefix + "pick_position"):
            self.declare_parameter(prefix + "pick_position", Parameter.Type.DOUBLE_ARRAY)
        if not self.has_parameter(prefix + "pick_orientation"):
            self.declare_parameter(
                prefix + "pick_orientation",
                Parameter.Type.DOUBLE_ARRAY,
            )
        if not self.has_parameter(prefix + "hand_offset"):
            self.declare_parameter(prefix + "hand_offset", Parameter.Type.DOUBLE_ARRAY)
        if not self.has_parameter(prefix + "handover_orientation"):
            self.declare_parameter(
                prefix + "handover_orientation",
                Parameter.Type.DOUBLE_ARRAY,
            )
        if not self.has_parameter(prefix + "lift_height"):
            self.declare_parameter(prefix + "lift_height", float("nan"))

    def _load_configuration(self) -> bool:
        tool_ids = self.get_parameter("tool_ids").value
        if not tool_ids:
            self.get_logger().error("Missing or empty required parameter: tool_ids")
            return False

        home_joints = self._get_double_array_parameter("motion_profiles.home_joints", 6)
        long_scissor_over_joints = self._get_double_array_parameter(
            "motion_profiles.long_scissor.over_joints", 6
        )
        long_scissor_after_joints = self._get_double_array_parameter(
            "motion_profiles.long_scissor.after_joints", 6
        )
        long_scissor_elbow_target = self._get_double_parameter(
            "motion_profiles.long_scissor.elbow_target"
        )
        long_scissor_elbow_tolerance_above = self._get_double_parameter(
            "motion_profiles.long_scissor.elbow_tolerance_above"
        )
        long_scissor_elbow_tolerance_below = self._get_double_parameter(
            "motion_profiles.long_scissor.elbow_tolerance_below"
        )

        if (
            home_joints is None
            or long_scissor_over_joints is None
            or long_scissor_after_joints is None
            or long_scissor_elbow_target is None
            or long_scissor_elbow_tolerance_above is None
            or long_scissor_elbow_tolerance_below is None
        ):
            return False

        self.motion_profiles = MotionProfiles(
            home_joints=home_joints,
            long_scissor_over_joints=long_scissor_over_joints,
            long_scissor_after_joints=long_scissor_after_joints,
            long_scissor_elbow_target=long_scissor_elbow_target,
            long_scissor_elbow_tolerance_above=long_scissor_elbow_tolerance_above,
            long_scissor_elbow_tolerance_below=long_scissor_elbow_tolerance_below,
        )

        if not self._load_handover_force_configuration():
            return False

        if not self._load_reclaim_configuration():
            return False

        loaded_tool_profiles: Dict[str, ToolProfile] = {}
        for tool_id in tool_ids:
            tool_id_str = str(tool_id)
            profile = self._load_tool_profile(tool_id_str)
            if profile is None:
                return False
            loaded_tool_profiles[tool_id_str] = profile

        self.tool_profiles = loaded_tool_profiles
        self.get_logger().info(
            f"Loaded {len(self.tool_profiles)} tool profiles from ROS2 parameters."
        )
        return True

    def _load_handover_force_configuration(self) -> bool:
        axis_threshold = self._get_double_parameter("handover.force_axis_threshold_newtons")
        force_norm_threshold = self._get_double_parameter("handover.force_norm_threshold_newtons")
        torque_norm_threshold = self._get_double_parameter(
            "handover.torque_norm_threshold_newton_meters"
        )
        zero_delay_seconds = self._get_double_parameter("handover.zero_delay_seconds")
        poll_rate_hz = self._get_double_parameter("handover.force_poll_rate_hz")
        debug_log_interval_seconds = self._get_double_parameter(
            "handover.force_debug_log_interval_seconds"
        )

        if (
            axis_threshold is None
            or force_norm_threshold is None
            or torque_norm_threshold is None
            or zero_delay_seconds is None
            or poll_rate_hz is None
            or debug_log_interval_seconds is None
        ):
            return False

        if axis_threshold <= 0.0 or force_norm_threshold <= 0.0 or torque_norm_threshold <= 0.0:
            self.get_logger().error("Handover force thresholds must be > 0.0.")
            return False
        if zero_delay_seconds < 0.0:
            self.get_logger().error("Parameter handover.zero_delay_seconds must be >= 0.0.")
            return False
        if poll_rate_hz <= 0.0:
            self.get_logger().error("Parameter handover.force_poll_rate_hz must be > 0.0.")
            return False
        if debug_log_interval_seconds < 0.0:
            self.get_logger().error(
                "Parameter handover.force_debug_log_interval_seconds must be >= 0.0."
            )
            return False

        self.handover_force_config = ForceSensingConfig(
            axis_threshold_newtons=axis_threshold,
            force_norm_threshold_newtons=force_norm_threshold,
            torque_norm_threshold_newton_meters=torque_norm_threshold,
            zero_delay_seconds=zero_delay_seconds,
            poll_rate_hz=poll_rate_hz,
            debug_log_interval_seconds=debug_log_interval_seconds,
        )
        return True

    def _load_tool_profile(self, tool_id: str) -> Optional[ToolProfile]:
        prefix = f"tool_profiles.{tool_id}."

        strategy = self.get_parameter(prefix + "strategy").value
        if not isinstance(strategy, str) or not strategy:
            self.get_logger().error(f"Missing or empty required parameter: {prefix}strategy")
            return None

        if strategy not in {
            STRATEGY_STANDARD,
            STRATEGY_HAMMER_CARTESIAN,
            STRATEGY_LONG_SCISSOR,
            STRATEGY_RETRACTOR_CARTESIAN,
        }:
            self.get_logger().error(f"Invalid strategy '{strategy}' for tool id '{tool_id}'.")
            return None

        pick_position = self._get_double_array_parameter(prefix + "pick_position", 3)
        pick_orientation = self._get_double_array_parameter(prefix + "pick_orientation", 4)
        hand_offset = self._get_double_array_parameter(prefix + "hand_offset", 3)
        handover_orientation = self._get_double_array_parameter(prefix + "handover_orientation", 4)
        lift_height = self._get_double_parameter(prefix + "lift_height")

        if (
            pick_position is None
            or pick_orientation is None
            or hand_offset is None
            or handover_orientation is None
            or lift_height is None
        ):
            return None

        return ToolProfile(
            tool_id=tool_id,
            strategy=strategy,
            pick_position=pick_position,
            pick_orientation=pick_orientation,
            hand_offset=hand_offset,
            handover_orientation=handover_orientation,
            lift_height=lift_height,
        )

    def _load_reclaim_configuration(self) -> bool:
        reclaim_force_threshold_newtons = self._get_double_parameter(
            "reclaim.force_threshold_newtons"
        )
        reclaim_force_norm_threshold_newtons = self._get_double_parameter(
            "reclaim.force_norm_threshold_newtons"
        )
        reclaim_torque_norm_threshold_newton_meters = self._get_double_parameter(
            "reclaim.torque_norm_threshold_newton_meters"
        )
        reclaim_zero_delay_seconds = self._get_double_parameter("reclaim.zero_delay_seconds")
        reclaim_force_poll_rate_hz = self._get_double_parameter("reclaim.force_poll_rate_hz")
        reclaim_dropoff_position = self._get_double_array_parameter("reclaim.dropoff_position", 3)
        reclaim_dropoff_orientation = self._get_double_array_parameter(
            "reclaim.dropoff_orientation", 4
        )
        reclaim_post_open_pause_seconds = self._get_double_parameter(
            "reclaim.post_open_pause_seconds"
        )
        reclaim_post_close_wait_seconds = self._get_double_parameter(
            "reclaim.post_close_wait_seconds"
        )
        reclaim_dropoff_descend_distance = self._get_double_parameter(
            "reclaim.dropoff_descend_distance"
        )

        if (
            reclaim_force_threshold_newtons is None
            or reclaim_force_norm_threshold_newtons is None
            or reclaim_torque_norm_threshold_newton_meters is None
            or reclaim_zero_delay_seconds is None
            or reclaim_force_poll_rate_hz is None
            or reclaim_dropoff_position is None
            or reclaim_dropoff_orientation is None
            or reclaim_post_open_pause_seconds is None
            or reclaim_post_close_wait_seconds is None
            or reclaim_dropoff_descend_distance is None
        ):
            return False

        if reclaim_post_open_pause_seconds < 0.0:
            self.get_logger().error("Parameter reclaim.post_open_pause_seconds must be >= 0.0.")
            return False
        if reclaim_post_close_wait_seconds < 0.0:
            self.get_logger().error("Parameter reclaim.post_close_wait_seconds must be >= 0.0.")
            return False
        if reclaim_dropoff_descend_distance < 0.0:
            self.get_logger().error("Parameter reclaim.dropoff_descend_distance must be >= 0.0.")
            return False
        if (
            reclaim_force_threshold_newtons <= 0.0
            or reclaim_force_norm_threshold_newtons <= 0.0
            or reclaim_torque_norm_threshold_newton_meters <= 0.0
        ):
            self.get_logger().error("Reclaim force thresholds must be > 0.0.")
            return False
        if reclaim_zero_delay_seconds < 0.0:
            self.get_logger().error("Parameter reclaim.zero_delay_seconds must be >= 0.0.")
            return False
        if reclaim_force_poll_rate_hz <= 0.0:
            self.get_logger().error("Parameter reclaim.force_poll_rate_hz must be > 0.0.")
            return False

        self.reclaim_dropoff_position = reclaim_dropoff_position
        self.reclaim_dropoff_orientation = reclaim_dropoff_orientation
        self.reclaim_post_open_pause_seconds = reclaim_post_open_pause_seconds
        self.reclaim_post_close_wait_seconds = reclaim_post_close_wait_seconds
        self.reclaim_dropoff_descend_distance = reclaim_dropoff_descend_distance
        debug_interval_seconds = 1.0
        if self.handover_force_config is not None:
            debug_interval_seconds = self.handover_force_config.debug_log_interval_seconds
        self.reclaim_force_config = ForceSensingConfig(
            axis_threshold_newtons=reclaim_force_threshold_newtons,
            force_norm_threshold_newtons=reclaim_force_norm_threshold_newtons,
            torque_norm_threshold_newton_meters=reclaim_torque_norm_threshold_newton_meters,
            zero_delay_seconds=reclaim_zero_delay_seconds,
            poll_rate_hz=reclaim_force_poll_rate_hz,
            debug_log_interval_seconds=debug_interval_seconds,
        )
        return True

    def _get_double_array_parameter(self, name: str, expected_size: int) -> Optional[List[float]]:
        value = self.get_parameter(name).value
        if not isinstance(value, list):
            self.get_logger().error(f"Missing required parameter: {name}")
            return None
        if len(value) != expected_size:
            self.get_logger().error(
                f"Parameter {name} must contain exactly {expected_size} values but has {len(value)}."
            )
            return None

        result: List[float] = []
        for element in value:
            if not isinstance(element, (int, float)):
                self.get_logger().error(f"Parameter {name} contains a non-numeric value.")
                return None
            result.append(float(element))
        return result

    def _get_double_parameter(self, name: str) -> Optional[float]:
        value = self.get_parameter(name).value
        if not isinstance(value, (int, float)):
            self.get_logger().error(f"Missing or invalid required parameter: {name}")
            return None

        value_float = float(value)
        if not math.isfinite(value_float):
            self.get_logger().error(f"Missing or invalid required parameter: {name}")
            return None
        return value_float

    def _initialize_rtde_interfaces(self) -> None:
        if RTDEControl is None or RTDEReceive is None:
            self.get_logger().error(
                "ur_rtde is not available. Install with 'python3 -m pip install --user ur_rtde'."
            )
            return

        try:
            if self.use_external_control:
                flags = 0
                if hasattr(RTDEControl, "FLAG_USE_EXT_UR_CAP"):
                    flags |= RTDEControl.FLAG_USE_EXT_UR_CAP
                if hasattr(RTDEControl, "FLAG_VERBOSE"):
                    flags |= RTDEControl.FLAG_VERBOSE
                self.get_logger().info(
                    "Initializing RTDE in External Control mode. A running External Control / RTDE control program is required on the controller."
                )
            else:
                flags = 0
                if hasattr(RTDEControl, "FLAG_UPLOAD_SCRIPT"):
                    flags |= RTDEControl.FLAG_UPLOAD_SCRIPT
                if hasattr(RTDEControl, "FLAG_VERBOSE"):
                    flags |= RTDEControl.FLAG_VERBOSE
                self.get_logger().info(
                    "Initializing RTDE in upload-script mode. The controller should accept uploaded RTDE control scripts."
                )

            try:
                self.rtde_control = RTDEControl(self.robot_ip, -1.0, flags)
            except TypeError:
                self.rtde_control = RTDEControl(self.robot_ip)

            self.rtde_receive = RTDEReceive(self.robot_ip)
            self.rtde_control.setTcp(self.tcp_pose)
            self.rtde_ready = True
            self.get_logger().info(
                f"RTDE interfaces connected to robot {self.robot_ip} (external_control={self.use_external_control})."
            )
            self.get_logger().info(f"Active RTDE TCP pose set to {self.tcp_pose}.")
            self.get_logger().info(
                "Applying input pose frame rotation RPY %s before sending Cartesian goals to the controller."
                % self.pose_frame_rotation_rpy
            )

            if not self.configuration_loaded:
                self.get_logger().error(
                    "Motion configuration is invalid. Rejecting motion execution until configuration is fixed."
                )
                return

            self.move_to_home_position_using_joints()
            self.get_logger().info("Opening gripper...")
            self.publish_gripper_mover(True)
        except Exception as exc:
            self.rtde_ready = False
            self.rtde_control = None
            self.rtde_receive = None
            self.get_logger().error(f"Failed to initialize RTDE interfaces: {exc}")

    def _is_rtde_ready(self) -> bool:
        if not self.rtde_ready or self.rtde_control is None:
            self.get_logger().warn("Action rejected: RTDE interface is not initialized.")
            return False
        return True

    @staticmethod
    def _quaternion_to_rotvec(quat_xyzw: List[float]) -> List[float]:
        qx, qy, qz, qw = quat_xyzw
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-12:
            return [0.0, 0.0, 0.0]

        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        qw = max(-1.0, min(1.0, qw))
        angle = 2.0 * math.acos(qw)
        sin_half = math.sqrt(max(0.0, 1.0 - qw * qw))

        if sin_half < 1e-8 or abs(angle) < 1e-8:
            return [0.0, 0.0, 0.0]

        ax = qx / sin_half
        ay = qy / sin_half
        az = qz / sin_half
        return [ax * angle, ay * angle, az * angle]

    @staticmethod
    def _normalize_quaternion(quat_xyzw: List[float]) -> List[float]:
        qx, qy, qz, qw = quat_xyzw
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-12:
            return [0.0, 0.0, 0.0, 1.0]
        return [qx / norm, qy / norm, qz / norm, qw / norm]

    @staticmethod
    def _quaternion_multiply(lhs_xyzw: List[float], rhs_xyzw: List[float]) -> List[float]:
        lx, ly, lz, lw = lhs_xyzw
        rx, ry, rz, rw = rhs_xyzw
        return [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ]

    @staticmethod
    def _rpy_to_quaternion(rpy: List[float]) -> List[float]:
        roll, pitch, yaw = rpy
        half_roll = roll * 0.5
        half_pitch = pitch * 0.5
        half_yaw = yaw * 0.5

        cr = math.cos(half_roll)
        sr = math.sin(half_roll)
        cp = math.cos(half_pitch)
        sp = math.sin(half_pitch)
        cy = math.cos(half_yaw)
        sy = math.sin(half_yaw)

        return SocketMover._normalize_quaternion(
            [
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy,
            ]
        )

    @staticmethod
    def _rotate_vector_by_quaternion(vector_xyz: List[float], quat_xyzw: List[float]) -> List[float]:
        qx, qy, qz, qw = SocketMover._normalize_quaternion(quat_xyzw)
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        vx, vy, vz = vector_xyz
        return [
            (1.0 - 2.0 * (yy + zz)) * vx + 2.0 * (xy - wz) * vy + 2.0 * (xz + wy) * vz,
            2.0 * (xy + wz) * vx + (1.0 - 2.0 * (xx + zz)) * vy + 2.0 * (yz - wx) * vz,
            2.0 * (xz - wy) * vx + 2.0 * (yz + wx) * vy + (1.0 - 2.0 * (xx + yy)) * vz,
        ]

    def _transform_pose_to_controller_frame(self, pose: Pose) -> Pose:
        transformed_pose = Pose()
        rotated_position = self._rotate_vector_by_quaternion(
            [pose.position.x, pose.position.y, pose.position.z],
            self.pose_frame_rotation_quat,
        )
        rotated_orientation = self._normalize_quaternion(
            self._quaternion_multiply(
                self.pose_frame_rotation_quat,
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
            )
        )

        transformed_pose.position.x = rotated_position[0]
        transformed_pose.position.y = rotated_position[1]
        transformed_pose.position.z = rotated_position[2]
        transformed_pose.orientation.x = rotated_orientation[0]
        transformed_pose.orientation.y = rotated_orientation[1]
        transformed_pose.orientation.z = rotated_orientation[2]
        transformed_pose.orientation.w = rotated_orientation[3]
        return transformed_pose

    def _pose_to_ur_pose(self, pose: Pose) -> List[float]:
        controller_pose = self._transform_pose_to_controller_frame(pose)
        rotvec = self._quaternion_to_rotvec(
            [
                controller_pose.orientation.x,
                controller_pose.orientation.y,
                controller_pose.orientation.z,
                controller_pose.orientation.w,
            ]
        )
        return [
            float(controller_pose.position.x),
            float(controller_pose.position.y),
            float(controller_pose.position.z),
            rotvec[0],
            rotvec[1],
            rotvec[2],
        ]

    @staticmethod
    def _joint_scaled(base_value: float, scale: float) -> float:
        return max(0.01, base_value * scale)

    @staticmethod
    def _linear_scaled(base_value: float, scale: float) -> float:
        return max(0.005, base_value * scale)

    def _get_actual_q(self) -> Optional[List[float]]:
        if self.rtde_receive is None:
            return None
        if not hasattr(self.rtde_receive, "getActualQ"):
            return None
        try:
            q = self.rtde_receive.getActualQ()
            if isinstance(q, list) and len(q) == 6:
                return [float(v) for v in q]
        except Exception as exc:
            self.get_logger().warn(f"Failed to read current joint state from RTDE receive: {exc}")
        return None

    def _get_actual_tcp_force(self) -> Optional[List[float]]:
        if self.rtde_receive is None:
            return None

        get_force_fn = getattr(self.rtde_receive, "getActualTCPForce", None)
        if get_force_fn is None:
            self.get_logger().error("RTDE receive interface does not provide getActualTCPForce().")
            return None

        try:
            wrench = get_force_fn()
        except Exception as exc:
            self.get_logger().error(f"Failed to read current TCP force from RTDE receive: {exc}")
            return None

        if not isinstance(wrench, list) or len(wrench) != 6:
            self.get_logger().error(
                f"RTDE getActualTCPForce() returned invalid data: {wrench!r}"
            )
            return None

        return [float(v) for v in wrench]

    @staticmethod
    def _force_triggered(delta_wrench: List[float], config: ForceSensingConfig) -> bool:
        force_norm = math.sqrt(
            delta_wrench[0] * delta_wrench[0]
            + delta_wrench[1] * delta_wrench[1]
            + delta_wrench[2] * delta_wrench[2]
        )
        torque_norm = math.sqrt(
            delta_wrench[3] * delta_wrench[3]
            + delta_wrench[4] * delta_wrench[4]
            + delta_wrench[5] * delta_wrench[5]
        )
        return (
            abs(delta_wrench[0]) > config.axis_threshold_newtons
            or abs(delta_wrench[1]) > config.axis_threshold_newtons
            or abs(delta_wrench[2]) > config.axis_threshold_newtons
            or force_norm > config.force_norm_threshold_newtons
            or torque_norm > config.torque_norm_threshold_newton_meters
        )

    def _wait_for_force_trigger(
        self,
        label: str,
        config: Optional[ForceSensingConfig],
    ) -> bool:
        if config is None:
            self.get_logger().error(f"{label}: force sensing configuration is unavailable.")
            return False
        if not self._is_rtde_ready():
            return False

        self.get_logger().info(
            "%s armed. Waiting %.2f s before capturing baseline."
            % (label, config.zero_delay_seconds)
        )

        baseline_wrench: Optional[List[float]] = None
        armed_time = time.time()
        last_debug_log_time = 0.0
        poll_sleep_seconds = max(0.001, 1.0 / config.poll_rate_hz)

        while rclpy.ok():
            if time.time() - armed_time < config.zero_delay_seconds:
                time.sleep(poll_sleep_seconds)
                continue

            current_wrench = self._get_actual_tcp_force()
            if current_wrench is None:
                self.get_logger().error(f"{label}: RTDE force read failed, aborting action.")
                return False

            if baseline_wrench is None:
                baseline_wrench = current_wrench
                self.get_logger().info(f"{label}: baseline captured.")
                time.sleep(poll_sleep_seconds)
                continue

            delta_wrench = [
                current_wrench[index] - baseline_wrench[index] for index in range(6)
            ]
            force_norm = math.sqrt(
                delta_wrench[0] * delta_wrench[0]
                + delta_wrench[1] * delta_wrench[1]
                + delta_wrench[2] * delta_wrench[2]
            )
            torque_norm = math.sqrt(
                delta_wrench[3] * delta_wrench[3]
                + delta_wrench[4] * delta_wrench[4]
                + delta_wrench[5] * delta_wrench[5]
            )

            now = time.time()
            if (
                config.debug_log_interval_seconds > 0.0
                and now - last_debug_log_time >= config.debug_log_interval_seconds
            ):
                self.get_logger().info(
                    "%s active: dF=(%.2f, %.2f, %.2f) |dF|=%.2f N, "
                    "dT=(%.3f, %.3f, %.3f) |dT|=%.3f Nm"
                    % (
                        label,
                        delta_wrench[0],
                        delta_wrench[1],
                        delta_wrench[2],
                        force_norm,
                        delta_wrench[3],
                        delta_wrench[4],
                        delta_wrench[5],
                        torque_norm,
                    )
                )
                last_debug_log_time = now

            if self._force_triggered(delta_wrench, config):
                self.get_logger().info(
                    "%s accepted: force trigger detected (|dF|=%.2f N, |dT|=%.3f Nm)."
                    % (label, force_norm, torque_norm)
                )
                return True

            time.sleep(poll_sleep_seconds)

        self.get_logger().warn(f"{label}: force sensing cancelled because ROS is shutting down.")
        return False

    def _ik_has_solution(self, ur_pose: List[float], qnear: Optional[List[float]]) -> bool:
        if self.rtde_control is None:
            return False

        has_solution_fn = getattr(self.rtde_control, "getInverseKinematicsHasSolution", None)
        if has_solution_fn is not None:
            try:
                if qnear is not None:
                    return bool(has_solution_fn(ur_pose, qnear))
                return bool(has_solution_fn(ur_pose))
            except TypeError:
                try:
                    return bool(has_solution_fn(ur_pose))
                except Exception as exc:
                    self.get_logger().warn(f"IK solution precheck failed: {exc}")
                    return False
            except Exception as exc:
                self.get_logger().warn(f"IK solution precheck failed: {exc}")
                return False

        ik_solution = self._compute_ik_solution(ur_pose, qnear)
        if ik_solution is None:
            ik_fn = getattr(self.rtde_control, "getInverseKinematics", None)
            if ik_fn is None:
                return True
            return False
        return len(ik_solution) == 6

    def _compute_ik_solution(
        self, ur_pose: List[float], qnear: Optional[List[float]]
    ) -> Optional[List[float]]:
        if self.rtde_control is None:
            return None

        ik_fn = getattr(self.rtde_control, "getInverseKinematics", None)
        if ik_fn is None:
            return None

        try:
            ik_solution = ik_fn(ur_pose, qnear) if qnear is not None else ik_fn(ur_pose)
        except TypeError:
            try:
                ik_solution = ik_fn(ur_pose)
            except Exception as exc:
                self.get_logger().warn(f"IK computation failed during precheck: {exc}")
                return None
        except Exception as exc:
            self.get_logger().warn(f"IK computation failed during precheck: {exc}")
            return None

        if not isinstance(ik_solution, list) or len(ik_solution) != 6:
            return None
        return [float(v) for v in ik_solution]

    def _is_ur_pose_reachable(self, ur_pose: List[float]) -> bool:
        if not self._is_rtde_ready():
            return False

        pose_safety_fn = getattr(self.rtde_control, "isPoseWithinSafetyLimits", None)
        if pose_safety_fn is not None:
            try:
                if not bool(pose_safety_fn(ur_pose)):
                    return False
            except Exception as exc:
                self.get_logger().warn(f"Pose safety precheck failed: {exc}")
                return False

        qnear = self._get_actual_q()
        if not self._ik_has_solution(ur_pose, qnear):
            return False

        ik_fn = getattr(self.rtde_control, "getInverseKinematics", None)
        joints_safety_fn = getattr(self.rtde_control, "isJointsWithinSafetyLimits", None)
        if ik_fn is not None and joints_safety_fn is not None:
            try:
                ik_q = ik_fn(ur_pose, qnear) if qnear is not None else ik_fn(ur_pose)
            except TypeError:
                try:
                    ik_q = ik_fn(ur_pose)
                except Exception as exc:
                    self.get_logger().warn(f"IK safety precheck failed: {exc}")
                    return False
            except Exception as exc:
                self.get_logger().warn(f"IK safety precheck failed: {exc}")
                return False

            if isinstance(ik_q, list) and len(ik_q) == 6:
                try:
                    if not bool(joints_safety_fn(ik_q)):
                        return False
                except Exception as exc:
                    self.get_logger().warn(f"Joint safety precheck failed: {exc}")
                    return False

        return True

    def _move_to_joints(
        self,
        joints: List[float],
        velocity_scale: float,
        acceleration_scale: float,
        success_log: Optional[str],
        failure_log: str,
    ) -> bool:
        if not self._is_rtde_ready():
            return False

        if len(joints) != 6:
            self.get_logger().error(failure_log)
            self.get_logger().error("Joint target must have exactly 6 values.")
            return False

        joints_safety_fn = getattr(self.rtde_control, "isJointsWithinSafetyLimits", None)
        if joints_safety_fn is not None:
            try:
                if not bool(joints_safety_fn(joints)):
                    self.get_logger().error(failure_log)
                    return False
            except Exception as exc:
                self.get_logger().warn(f"Joint safety check failed: {exc}")
                self.get_logger().error(failure_log)
                return False

        speed = self._joint_scaled(self.max_joint_speed, velocity_scale)
        acceleration = self._joint_scaled(self.max_joint_acceleration, acceleration_scale)

        try:
            try:
                command_ok = bool(self.rtde_control.moveJ(joints, speed, acceleration, False))
            except TypeError:
                command_ok = bool(self.rtde_control.moveJ(joints, speed, acceleration))
        except Exception as exc:
            self.get_logger().error(f"{failure_log} ({exc})")
            return False

        if not command_ok:
            self.get_logger().error(failure_log)
            return False

        if success_log:
            self.get_logger().info(success_log)
        return True

    def _move_to_pose_jik(
        self,
        target_pose: Pose,
        velocity_scale: float,
        acceleration_scale: float,
        success_log: Optional[str],
        failure_log: str,
        precheck: bool = True,
    ) -> bool:
        if not self._is_rtde_ready():
            return False

        ur_pose = self._pose_to_ur_pose(target_pose)
        if precheck and not self._is_ur_pose_reachable(ur_pose):
            self.get_logger().error(failure_log)
            return False

        speed = self._joint_scaled(self.max_joint_speed, velocity_scale)
        acceleration = self._joint_scaled(self.max_joint_acceleration, acceleration_scale)

        try:
            try:
                command_ok = bool(self.rtde_control.moveJ_IK(ur_pose, speed, acceleration, False))
            except TypeError:
                command_ok = bool(self.rtde_control.moveJ_IK(ur_pose, speed, acceleration))
        except Exception as exc:
            self.get_logger().error(f"{failure_log} ({exc})")
            return False

        if not command_ok:
            self.get_logger().error(failure_log)
            return False

        if success_log:
            self.get_logger().info(success_log)
        return True

    def _move_to_pose_linear(
        self,
        target_pose: Pose,
        velocity_scale: float,
        acceleration_scale: float,
        success_log: Optional[str],
        failure_log: str,
    ) -> bool:
        if not self._is_rtde_ready():
            return False

        ur_pose = self._pose_to_ur_pose(target_pose)
        if not self._is_ur_pose_reachable(ur_pose):
            self.get_logger().error(failure_log)
            return False

        speed = self._linear_scaled(self.max_linear_speed, velocity_scale)
        acceleration = self._linear_scaled(self.max_linear_acceleration, acceleration_scale)

        try:
            try:
                command_ok = bool(self.rtde_control.moveL(ur_pose, speed, acceleration, False))
            except TypeError:
                command_ok = bool(self.rtde_control.moveL(ur_pose, speed, acceleration))
        except Exception as exc:
            self.get_logger().error(f"{failure_log} ({exc})")
            return False

        if not command_ok:
            self.get_logger().error(failure_log)
            return False

        if success_log:
            self.get_logger().info(success_log)
        return True

    def _create_pose(self, position: List[float], orientation: List[float]) -> Pose:
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        return pose

    def activate_tool_profile(self, profile: ToolProfile) -> None:
        self.active_tool_profile = profile
        self.tool_position = profile.pick_position
        self.tool_orientation = profile.pick_orientation
        self.hand_offset = profile.hand_offset
        self.handover_orientation = profile.handover_orientation

    def publish_gripper_mover(self, open_gripper: bool) -> None:
        msg = Bool()
        msg.data = open_gripper
        self.gripper_mover_pub.publish(msg)

    def publish_gripper_position_command(self, position: int) -> None:
        msg = Int32()
        msg.data = int(position)
        self.gripper_position_command_pub.publish(msg)

    def publish_handover_event(self, event_name: str) -> None:
        msg = String()
        msg.data = event_name
        self.handover_event_pub.publish(msg)
        self.get_logger().info(f"Published handover event: {event_name}")

    def move_to_home_position_using_joints(self) -> None:
        if self.motion_profiles is None:
            self.get_logger().error(
                "Cannot move home: configured motion profiles are unavailable."
            )
            return

        if len(self.motion_profiles.home_joints) != 6:
            self.get_logger().error("Cannot move home: configured home joint preset is invalid.")
            return

        self._move_to_joints(
            self.motion_profiles.home_joints,
            velocity_scale=1.0,
            acceleration_scale=1.0,
            success_log="Planning to Home-Position (Joints) successful, executing...",
            failure_log="Planning to Home-Position (Joints) failed.",
        )

    def move_to_pose_target(
        self,
        target_pose: Pose,
        velocity_scale: float,
        acceleration_scale: float,
        success_log: str,
        failure_log: str,
    ) -> bool:
        return self._move_to_pose_jik(
            target_pose,
            velocity_scale,
            acceleration_scale,
            success_log=success_log,
            failure_log=failure_log,
            precheck=True,
        )

    def start_reclaim_sequence(self) -> None:
        self.publish_gripper_mover(True)
        time.sleep(0.2)

        if not self.move_to_pose_target(
            self.last_handover_pose,
            0.6,
            0.6,
            "Reached stored handover pose for reclaim.",
            "Action rejected: reclaim start pose is unreachable.",
        ):
            self.reclaim_in_progress = False
            self.waiting_for_reclaim_done = False
            return

        self.reclaim_in_progress = True
        self.waiting_for_reclaim_done = True
        self.get_logger().info("Action accepted: reclaim started at last handover pose.")

        if not self._wait_for_force_trigger("Reclaim close sensing", self.reclaim_force_config):
            self.reclaim_in_progress = False
            self.waiting_for_reclaim_done = False
            self.get_logger().error("Reclaim aborted: RTDE force sensing failed.")
            return

        reclaim_close_position = self.get_parameter("reclaim.close_position").value
        self.publish_gripper_position_command(int(reclaim_close_position))
        self.tool_has_been_picked_up = True
        self.get_logger().info("Reclaim accepted: close command published to gripper.")
        time.sleep(self.reclaim_post_close_wait_seconds)

        dropoff_approach_pose = self._create_pose(
            self.reclaim_dropoff_position,
            self.reclaim_dropoff_orientation,
        )

        dropoff_release_position = list(self.reclaim_dropoff_position)
        dropoff_release_position[2] -= self.reclaim_dropoff_descend_distance
        dropoff_release_pose = self._create_pose(
            dropoff_release_position,
            self.reclaim_dropoff_orientation,
        )

        if not self.move_to_pose_target(
            dropoff_approach_pose,
            0.7,
            0.7,
            "Reached reclaim dropoff approach pose.",
            "Reclaim failed: dropoff approach pose is unreachable.",
        ):
            self.reclaim_in_progress = False
            self.waiting_for_reclaim_done = False
            self.waiting_for_hand_pose = False
            self.waiting_for_gripper_done = False
            self.hand_pose_received = False
            self.tool_has_been_picked_up = False
            return

        if not self.move_to_pose_target(
            dropoff_release_pose,
            0.5,
            0.5,
            "Reached reclaim dropoff release pose.",
            "Reclaim failed: dropoff release pose is unreachable.",
        ):
            self.reclaim_in_progress = False
            self.waiting_for_reclaim_done = False
            self.waiting_for_hand_pose = False
            self.waiting_for_gripper_done = False
            self.hand_pose_received = False
            self.tool_has_been_picked_up = False
            return

        self.publish_gripper_mover(True)
        time.sleep(self.reclaim_post_open_pause_seconds)
        self.move_to_home_position_using_joints()

        self.reclaim_in_progress = False
        self.waiting_for_reclaim_done = False
        self.tool_has_been_picked_up = False
        self.waiting_for_hand_pose = False
        self.waiting_for_gripper_done = False
        self.hand_pose_received = False
        self.get_logger().info("Reclaim complete: returned to home.")

    def perform_handover_to_hand_pose(self) -> None:
        target_pose = Pose()
        target_pose.position.x = self.hand_pose_with_offset.position.x
        target_pose.position.y = self.hand_pose_with_offset.position.y
        target_pose.position.z = self.hand_pose_with_offset.position.z
        target_pose.orientation.x = self.handover_orientation[0]
        target_pose.orientation.y = self.handover_orientation[1]
        target_pose.orientation.z = self.handover_orientation[2]
        target_pose.orientation.w = self.handover_orientation[3]

        self.get_logger().info(
            "Target for handover: x=%.2f y=%.2f z=%.2f"
            % (target_pose.position.x, target_pose.position.y, target_pose.position.z)
        )

        handover_ur_pose = self._pose_to_ur_pose(target_pose)
        if not self._is_ur_pose_reachable(handover_ur_pose):
            self.publish_handover_event("reachability:unreachable_plan_failed")
            self.get_logger().error("Reachability decision: unreachable.")
            self.get_logger().error("Planning to above hand failed.")
            return

        self.get_logger().info("Reachability decision: reachable.")
        self.get_logger().info("Moving above hand...")

        if not self._move_to_pose_jik(
            target_pose,
            velocity_scale=0.6,
            acceleration_scale=0.6,
            success_log=None,
            failure_log="Execution to above hand failed.",
            precheck=False,
        ):
            self.publish_handover_event("reachability:unreachable_execution_failed")
            self.get_logger().error("Reachability decision: unreachable.")
            return

        self.last_handover_pose = target_pose
        self.has_last_handover_pose = True
        self.waiting_for_hand_pose = False
        self.hand_pose_received = True

        self.get_logger().info("Reached handover pose, holding before release sensing.")
        time.sleep(self.pre_release_dwell_seconds)
        self.waiting_for_gripper_done = True
        if not self._wait_for_force_trigger("Handover release sensing", self.handover_force_config):
            self.waiting_for_gripper_done = False
            self.get_logger().error("Handover release sensing aborted.")
            return

        self.get_logger().info("Handover accepted: opening gripper after RTDE force trigger.")
        self.publish_gripper_mover(True)
        time.sleep(self.post_open_pause_seconds)
        self.get_logger().info("Returning to home after handover release pause.")
        self.move_to_home_position_using_joints()

        self.hand_pose_received = False
        self.waiting_for_hand_pose = False
        self.waiting_for_gripper_done = False
        self.tool_has_been_picked_up = False
        self.get_logger().info("Ready for next tool command.")

    def _reset_state_flags(self) -> None:
        self.hand_pose_received = False
        self.waiting_for_hand_pose = False
        self.waiting_for_gripper_done = False
        self.waiting_for_reclaim_done = False
        self.tool_has_been_picked_up = False
        self.reclaim_in_progress = False

    def _tool_selection_callback(self, msg: String) -> None:
        cmd = msg.data

        if cmd == "0":
            self.get_logger().warn("RESETTING SYSTEM STATE MANUALLY.")
            self._reset_state_flags()
            self.publish_gripper_mover(True)

            if self.configuration_loaded and self._is_rtde_ready():
                self.move_to_home_position_using_joints()
            else:
                self.get_logger().warn(
                    "Reset skipped home motion because RTDE interface or configuration is unavailable."
                )

            self.get_logger().info("System reset complete.")
            return

        if not self.configuration_loaded:
            self.get_logger().error(
                f"Action rejected: tool command '{cmd}' received but motion configuration is invalid."
            )
            return

        if not self._is_rtde_ready():
            self.get_logger().warn(
                f"Action rejected: tool command '{cmd}' received before RTDE initialization."
            )
            return

        if self.reclaim_in_progress:
            self.get_logger().warn("Action rejected: reclaim is already in progress.")
            return

        if cmd == "9":
            if self.waiting_for_hand_pose or self.waiting_for_gripper_done:
                self.get_logger().warn(
                    "Action rejected: reclaim requested while another handover action is still active."
                )
                return

            if not self.has_last_handover_pose:
                self.get_logger().warn(
                    "Action rejected: reclaim requested but no stored handover pose is available."
                )
                return

            self.start_reclaim_sequence()
            return

        if self.waiting_for_gripper_done:
            self.get_logger().warn(
                "Action rejected: still waiting for gripper completion from the previous handover."
            )
            return

        if self.waiting_for_hand_pose:
            self.get_logger().warn("Already waiting for hand pose - ignoring tool command.")
            return

        profile = self.tool_profiles.get(cmd)
        if profile is None:
            self.get_logger().warn(f"Unknown command: '{cmd}'")
            return

        self.activate_tool_profile(profile)
        self.get_logger().info(
            f"Action accepted: tool {cmd} selected with configured strategy."
        )

        if profile.strategy == STRATEGY_STANDARD:
            self.grab_tool(profile.pick_position[0], profile.pick_position[1], profile.pick_position[2])
        elif profile.strategy == STRATEGY_HAMMER_CARTESIAN:
            self.grab_hammer(profile.pick_position[0], profile.pick_position[1], profile.pick_position[2])
        elif profile.strategy == STRATEGY_LONG_SCISSOR:
            self.grab_long_scissor(
                profile.pick_position[0], profile.pick_position[1], profile.pick_position[2]
            )
        elif profile.strategy == STRATEGY_RETRACTOR_CARTESIAN:
            self.grab_retractor(
                profile.pick_position[0], profile.pick_position[1], profile.pick_position[2]
            )

        if self.waiting_for_hand_pose:
            self.get_logger().info("Waiting for hand pose...")
        else:
            self.get_logger().warn(
                f"Action rejected: tool {cmd} did not complete pickup, see previous planning errors."
            )

    def _hand_pose_callback(self, msg: Pose) -> None:
        self.get_logger().info("Hand detected: received /hand_pose message.")

        if not self.waiting_for_hand_pose:
            self.get_logger().info(
                "Action rejected: hand pose received but system is not waiting for a gesture."
            )
            return

        if not self.tool_has_been_picked_up:
            self.get_logger().info(
                "Action rejected: hand pose received but no tool has been picked up yet."
            )
            return

        self.hand_pose = msg
        self.hand_pose_with_offset = Pose()
        self.hand_pose_with_offset.position.x = msg.position.x + self.hand_offset[0]
        self.hand_pose_with_offset.position.y = msg.position.y + self.hand_offset[1]
        self.hand_pose_with_offset.position.z = msg.position.z + self.hand_offset[2]
        self.hand_pose_with_offset.orientation = msg.orientation

        self.publish_handover_event("gesture_detected")
        self.get_logger().info(
            "Gesture detected from hand pose message, proceeding with handover..."
        )
        self.perform_handover_to_hand_pose()

    def grab_tool(self, x: float, y: float, z: float) -> None:
        self.get_logger().info(f"Moving to object at x={x:.2f} y={y:.2f} z={z:.2f}")
        self.publish_gripper_mover(True)

        object_pose = self._create_pose([x, y, z], self.tool_orientation)
        lift_pose = self._create_pose([x, y, z + self.active_tool_profile.lift_height], self.tool_orientation)

        if not self._move_to_pose_jik(
            lift_pose,
            1.0,
            1.0,
            "Planning above object successful, executing...",
            "Planning above object failed.",
        ):
            return

        if not self._move_to_pose_jik(
            object_pose,
            0.5,
            0.5,
            "Planning to object successful, executing...",
            "Planning to object failed.",
        ):
            return

        self.get_logger().info("Closing gripper on object...")
        self.publish_gripper_mover(False)
        time.sleep(0.2)
        self.tool_has_been_picked_up = True

        if self._move_to_pose_jik(
            lift_pose,
            1.0,
            1.0,
            "Lifting object...",
            "Lift motion failed.",
        ):
            self.waiting_for_hand_pose = True
            self.move_to_home_position_using_joints()

    def grab_hammer(self, x: float, y: float, z: float) -> None:
        self.get_logger().info(f"Moving to object at x={x:.2f} y={y:.2f} z={z:.2f}")
        self.publish_gripper_mover(True)

        object_pose = self._create_pose([x, y, z], self.tool_orientation)
        lift_pose = self._create_pose([x, y, z + self.active_tool_profile.lift_height], self.tool_orientation)

        if not self._move_to_pose_jik(
            lift_pose,
            1.0,
            1.0,
            "Planning above object successful, executing...",
            "Planning above object failed.",
        ):
            return

        if not self._move_to_pose_linear(
            object_pose,
            1.0,
            1.0,
            "Linear path to object achieved, executing...",
            "Failed to compute linear Cartesian path to object.",
        ):
            return

        self.get_logger().info("Closing gripper on object...")
        self.publish_gripper_mover(False)
        time.sleep(0.2)
        self.tool_has_been_picked_up = True

        if self._move_to_pose_linear(
            lift_pose,
            1.0,
            1.0,
            "Lifting object with linear motion...",
            "Cartesian lift motion failed.",
        ):
            self.waiting_for_hand_pose = True
            self.move_to_home_position_using_joints()

    def grab_long_scissor(self, x: float, y: float, z: float) -> None:
        if self.motion_profiles is None:
            self.get_logger().error("Long scissor profile unavailable.")
            return

        self.get_logger().info(f"Moving to object at x={x:.2f} y={y:.2f} z={z:.2f}")
        self.publish_gripper_mover(True)

        if not self._move_to_joints(
            self.motion_profiles.long_scissor_over_joints,
            1.0,
            1.0,
            "Planning above Scissor successful, executing...",
            "Planning above Scissor failed.",
        ):
            return

        object_pose = self._create_pose([x, y, z], self.tool_orientation)
        object_ur_pose = self._pose_to_ur_pose(object_pose)
        ik_solution = self._compute_ik_solution(object_ur_pose, self._get_actual_q())

        if ik_solution is not None:
            elbow_joint = ik_solution[2]
            lower_bound = (
                self.motion_profiles.long_scissor_elbow_target
                - self.motion_profiles.long_scissor_elbow_tolerance_below
            )
            upper_bound = (
                self.motion_profiles.long_scissor_elbow_target
                + self.motion_profiles.long_scissor_elbow_tolerance_above
            )
            if elbow_joint < lower_bound or elbow_joint > upper_bound:
                self.get_logger().error("Planning to object pose failed.")
                self.get_logger().error(
                    "Long scissor elbow constraint rejected IK solution (elbow=%.5f, range=[%.5f, %.5f])."
                    % (elbow_joint, lower_bound, upper_bound)
                )
                return

            if not self._move_to_joints(
                ik_solution,
                1.0,
                1.0,
                "Planned motion to object pose successful, executing...",
                "Planning to object pose failed.",
            ):
                return
        else:
            if not self._move_to_pose_jik(
                object_pose,
                1.0,
                1.0,
                "Planned motion to object pose successful, executing...",
                "Planning to object pose failed.",
            ):
                return

        self.get_logger().info("Closing gripper on object...")
        self.publish_gripper_mover(False)
        self.tool_has_been_picked_up = True

        if not self._move_to_joints(
            self.motion_profiles.long_scissor_after_joints,
            1.0,
            1.0,
            "Planning above Scissor successful, executing...",
            "Planning above Scissor failed.",
        ):
            return
        self.waiting_for_hand_pose = True

        if self._move_to_joints(
            self.motion_profiles.long_scissor_over_joints,
            1.0,
            1.0,
            "Planning above Scissor successful, executing...",
            "Planning above Scissor failed.",
        ):
            self.waiting_for_hand_pose = True
            self.move_to_home_position_using_joints()

    def grab_retractor(self, x: float, y: float, z: float) -> None:
        self.get_logger().info(f"Moving to object at x={x:.2f} y={y:.2f} z={z:.2f}")
        self.publish_gripper_mover(True)

        object_pose = self._create_pose([x, y, z], self.tool_orientation)
        lift_pose = self._create_pose([x, y, z + self.active_tool_profile.lift_height], self.tool_orientation)

        if not self._move_to_pose_jik(
            lift_pose,
            1.0,
            1.0,
            "Planning above object successful, executing...",
            "Planning above object failed.",
        ):
            return

        if not self._move_to_pose_linear(
            object_pose,
            1.0,
            1.0,
            "Linear path to object achieved, executing...",
            "Failed to compute linear Cartesian path to object.",
        ):
            return

        self.get_logger().info("Closing gripper on object...")
        self.publish_gripper_mover(False)
        time.sleep(1.0)
        self.tool_has_been_picked_up = True

        if self._move_to_pose_linear(
            lift_pose,
            1.0,
            1.0,
            "Lifting object with linear motion...",
            "Cartesian lift motion failed.",
        ):
            self.waiting_for_hand_pose = True
            self.move_to_home_position_using_joints()

    def destroy_node(self) -> bool:
        if self.rtde_control is not None:
            stop_script_fn = getattr(self.rtde_control, "stopScript", None)
            if stop_script_fn is not None:
                try:
                    stop_script_fn()
                except Exception as exc:
                    self.get_logger().warn(f"Failed to stop RTDE control script cleanly: {exc}")

        if self.rtde_receive is not None:
            disconnect_fn = getattr(self.rtde_receive, "disconnect", None)
            if disconnect_fn is not None:
                try:
                    disconnect_fn()
                except Exception as exc:
                    self.get_logger().warn(f"Failed to disconnect RTDE receive: {exc}")

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SocketMover()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down by user request.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
