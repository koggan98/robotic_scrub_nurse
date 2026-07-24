#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive

# ===== USER-TUNABLE PARAMETERS (edit here to finetune) =====
# The display mounted next to the MiR, modelled as one tilted box in the world
# frame (arm root == world). The TOP edge is the near/high point; the panel leans
# so the bottom juts out in +x (away from the robot) and downward. Geometry is
# anchored by the top edge so the numbers below map 1:1 to how the display was
# measured; the box centre is solved from them.
FRAME_ID = "world"
OBJECT_ID = "display"

LENGTH_Y = 0.35      # panel length, along world Y, centered at y=0
THICKNESS = 0.02     # panel thickness (face normal), pre-tilt local X
HEIGHT = 0.20        # panel height (long face), pre-tilt local Z
TILT_DEG = 30.0      # tilt about Y; bottom juts out in +x (away from robot), top edge near

# World pose of the top edge midpoint (the near/high edge of the panel):
TOP_EDGE_X = 0.07    # ~7 cm from the robot origin in +x (moved 1 cm closer to robot)
TOP_EDGE_Z = 0.08    # top edge height in world; lowered 5 cm from 0.13; TUNE to box_0
Y_CENTER = 0.0       # panel centered on the x-axis (symmetric about y=0)

PUBLISH_HZ = 2.0     # overridden from the launch file (0.2 Hz there)
# ===========================================================


class DisplayCollisionPublisher(Node):
    def __init__(self):
        super().__init__("display_collision_publisher")

        self.frame_id = self.declare_parameter("frame_id", FRAME_ID).value
        self.collision_topic = self.declare_parameter(
            "collision_topic", "/collision_object").value
        self.object_id = self.declare_parameter("object_id", OBJECT_ID).value
        self.length_y_m = float(self.declare_parameter("length_y_m", LENGTH_Y).value)
        self.thickness_m = float(self.declare_parameter("thickness_m", THICKNESS).value)
        self.height_m = float(self.declare_parameter("height_m", HEIGHT).value)
        self.tilt_rad = math.radians(
            float(self.declare_parameter("tilt_deg", TILT_DEG).value))
        self.top_edge_x_m = float(
            self.declare_parameter("top_edge_x_m", TOP_EDGE_X).value)
        self.top_edge_z_m = float(
            self.declare_parameter("top_edge_z_m", TOP_EDGE_Z).value)
        self.y_center_m = float(self.declare_parameter("y_center_m", Y_CENTER).value)
        self.publish_hz = float(self.declare_parameter("publish_hz", PUBLISH_HZ).value)

        # The panel leans so the TOP edge is the near/high point and the bottom
        # juts out in +x (away from the robot) and downward => negative pitch.
        self.pitch = -self.tilt_rad
        # Top-edge midpoint is local (0, 0, +height/2); after the pitch about Y its
        # world offset from the box centre is (sin(pitch)*h/2, 0, cos(pitch)*h/2).
        off_x = math.sin(self.pitch) * (self.height_m / 2.0)
        off_z = math.cos(self.pitch) * (self.height_m / 2.0)
        self.center = [
            self.top_edge_x_m - off_x,
            self.y_center_m,
            self.top_edge_z_m - off_z,
        ]

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
            f"Publishing display collision object '{self.object_id}' in "
            f"{self.frame_id}: size=[{self.thickness_m:.3f}, {self.length_y_m:.3f}, "
            f"{self.height_m:.3f}]m, tilt {math.degrees(self.tilt_rad):.1f}deg about Y "
            f"(bottom juts +x/down), center=[{self.center[0]:.3f}, {self.center[1]:.3f}, "
            f"{self.center[2]:.3f}], top edge=[{self.top_edge_x_m:.3f}, "
            f"{self.y_center_m:.3f}, {self.top_edge_z_m:.3f}]."
        )
        self.publish_collision_object()

    def publish_collision_object(self):
        msg = CollisionObject()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.id = self.object_id
        msg.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [self.thickness_m, self.length_y_m, self.height_m]

        pose = Pose()
        pose.position.x = self.center[0]
        pose.position.y = self.center[1]
        pose.position.z = self.center[2]
        pose.orientation = self._quaternion_from_rpy((0.0, self.pitch, 0.0))

        msg.primitives.append(primitive)
        msg.primitive_poses.append(pose)
        self.publisher.publish(msg)

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
    node = DisplayCollisionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
