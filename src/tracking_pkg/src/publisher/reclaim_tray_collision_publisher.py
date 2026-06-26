#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive

# ===== USER-TUNABLE PARAMETERS (edit here to finetune) =====
# Placement of the whole reclaim tray in the world frame:
ORIGIN_XYZ = [-0.1, -0.36, 0.0]   # [x, y, z] in m (y=0.35 per spec; tune x,z for MiR end)
ORIGIN_RPY = [0.0, 0.0, 0.0]    # [roll, pitch, yaw] in rad (tune rotation)

# Inverted-Z geometry: top bar +X, drop -Z, bottom bar +X (extruded WIDTH_Y along Y):
SEGMENT1_LENGTH_X = 0.21        # top bar length, +X
DROP_LENGTH_Z = 0.17           # vertical drop, -Z
SEGMENT2_LENGTH_X = 0.27        # bottom bar length, +X
WIDTH_Y = 0.13                 # total width along world Y
BAR_THICKNESS_Z = 0.03         # Z-thickness of horizontal bars (10mm < instrument tray)
POST_THICKNESS_X = 0.03        # X-thickness of the vertical drop

# 40x40 vertical post rising +Z from the bottom of the drop, flush to outer (+X) face:
POST40_SIZE = 0.04             # 40x40 profile cross-section (X & Y), in m
POST40_LENGTH = 0.600           # 550 mm, vertical (+Z, up)
# ===========================================================


class ReclaimTrayCollisionPublisher(Node):
    def __init__(self):
        super().__init__("reclaim_tray_collision_publisher")

        self.frame_id = self.declare_parameter("frame_id", "world").value
        self.collision_topic = self.declare_parameter(
            "collision_topic", "/collision_object").value
        self.object_id = self.declare_parameter("object_id", "reclaim_tray").value
        self.origin_xyz = self._declare_vector3("origin_xyz", ORIGIN_XYZ)
        self.origin_rpy = self._declare_vector3("origin_rpy", ORIGIN_RPY)
        self.segment1_length_m = float(
            self.declare_parameter("segment1_length_m", SEGMENT1_LENGTH_X).value)
        self.drop_length_m = float(
            self.declare_parameter("drop_length_m", DROP_LENGTH_Z).value)
        self.segment2_length_m = float(
            self.declare_parameter("segment2_length_m", SEGMENT2_LENGTH_X).value)
        self.width_y_m = float(
            self.declare_parameter("width_y_m", WIDTH_Y).value)
        self.bar_thickness_m = float(
            self.declare_parameter("bar_thickness_m", BAR_THICKNESS_Z).value)
        self.post_thickness_m = float(
            self.declare_parameter("post_thickness_m", POST_THICKNESS_X).value)
        self.post40_size_m = float(
            self.declare_parameter("post40_size_m", POST40_SIZE).value)
        self.post40_length_m = float(
            self.declare_parameter("post40_length_m", POST40_LENGTH).value)
        self.publish_hz = float(self.declare_parameter("publish_hz", 0.5).value)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            CollisionObject, self.collision_topic, qos)

        period = 1.0 / self.publish_hz if self.publish_hz > 0.0 else 0.5
        self.timer = self.create_timer(period, self.publish_collision_object)

        self.get_logger().info(
            f"Publishing reclaim tray collision object '{self.object_id}' "
            f"in {self.frame_id}: "
            f"origin_xyz=[{self.origin_xyz[0]:.3f}, "
            f"{self.origin_xyz[1]:.3f}, {self.origin_xyz[2]:.3f}], "
            f"origin_rpy=[{self.origin_rpy[0]:.3f}, "
            f"{self.origin_rpy[1]:.3f}, {self.origin_rpy[2]:.3f}], "
            f"shape=+X {self.segment1_length_m:.3f}m, "
            f"-Z {self.drop_length_m:.3f}m, "
            f"+X {self.segment2_length_m:.3f}m, "
            f"width_y {self.width_y_m:.3f}m, "
            f"bar_thickness_z {self.bar_thickness_m:.3f}m, "
            f"post40 {self.post40_size_m:.3f}x{self.post40_size_m:.3f}x"
            f"{self.post40_length_m:.3f}m (+Z, flush +X/-Y)."
        )
        self.publish_collision_object()

    def _declare_vector3(self, name, default):
        values = list(self.declare_parameter(name, default).value)
        if len(values) != 3:
            raise ValueError(
                f"Parameter '{name}' must contain exactly 3 numeric values.")
        return [float(value) for value in values]

    def publish_collision_object(self):
        msg = CollisionObject()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.id = self.object_id
        msg.operation = CollisionObject.ADD

        for dimensions, local_center in self._local_segments():
            msg.primitives.append(self._box(dimensions))
            msg.primitive_poses.append(self._pose_from_local_center(local_center))

        self.publisher.publish(msg)

    def _local_segments(self):
        # Inverted-Z in the X-Z plane, extruded by width_y along Y.
        # Local origin = start of the top bar; top-bar centerline at z=0.
        return [
            (
                # top bar, runs +X
                [self.segment1_length_m, self.width_y_m, self.bar_thickness_m],
                [self.segment1_length_m / 2.0, 0.0, 0.0],
            ),
            (
                # vertical drop, runs -Z, at the end of the top bar
                [self.post_thickness_m, self.width_y_m, self.drop_length_m],
                [self.segment1_length_m, 0.0, -self.drop_length_m / 2.0],
            ),
            (
                # bottom bar, runs +X, at the bottom of the drop
                [self.segment2_length_m, self.width_y_m, self.bar_thickness_m],
                [
                    self.segment1_length_m + (self.segment2_length_m / 2.0),
                    0.0,
                    -self.drop_length_m,
                ],
            ),
            (
                # 40x40 vertical post, +Z (up) from the bottom of the drop,
                # flush against the drop's outer (+X) face and the -Y edge.
                [self.post40_size_m, self.post40_size_m, self.post40_length_m],
                [
                    self.segment1_length_m
                    + (self.post_thickness_m / 2.0)
                    + (self.post40_size_m / 2.0),
                    -(self.width_y_m / 2.0) + (self.post40_size_m / 2.0),
                    -self.drop_length_m + (self.post40_length_m / 2.0),
                ],
            ),
        ]

    def _box(self, dimensions):
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dimensions
        return primitive

    def _pose_from_local_center(self, local_center):
        rotated = self._rotate_xyz(local_center)
        pose = Pose()
        pose.position.x = self.origin_xyz[0] + rotated[0]
        pose.position.y = self.origin_xyz[1] + rotated[1]
        pose.position.z = self.origin_xyz[2] + rotated[2]
        pose.orientation = self._quaternion_from_rpy(self.origin_rpy)
        return pose

    def _rotate_xyz(self, xyz):
        roll, pitch, yaw = self.origin_rpy
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        x, y, z = xyz
        rx = cy * cp * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z
        ry = sy * cp * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z
        rz = -sp * x + cp * sr * y + cp * cr * z
        return [rx, ry, rz]

    def _quaternion_from_rpy(self, rpy):
        roll, pitch, yaw = rpy
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return Quaternion(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ReclaimTrayCollisionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
