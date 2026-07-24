#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive

# ===== USER-TUNABLE PARAMETERS (edit here to finetune) =====
# A thin horizontal ceiling plate above the workspace: a hard upper bound the
# planner keeps the arm below. Box extents [x, y, z] and its centre in `world`.
BOX_SIZE_XYZ = [1.0, 1.0, 0.01]   # [x, y, z] extents in m
ORIGIN_XYZ = [0.0, 0.0, 0.55]     # box centre [x, y, z] in m (world frame)
# ===========================================================


class UpperBoundPublisher(Node):
    def __init__(self):
        super().__init__("upper_bound_publisher")

        self.frame_id = self.declare_parameter("frame_id", "world").value
        self.collision_topic = self.declare_parameter(
            "collision_topic", "/collision_object").value
        self.object_id = self.declare_parameter("object_id", "upper_bound").value
        self.box_size_xyz = self._declare_vector3("box_size_xyz", BOX_SIZE_XYZ)
        self.origin_xyz = self._declare_vector3("origin_xyz", ORIGIN_XYZ)
        self.publish_hz = float(self.declare_parameter("publish_hz", 2.0).value)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            CollisionObject, self.collision_topic, qos
        )

        self.last_subscription_count = None
        period = 1.0 / self.publish_hz if self.publish_hz > 0.0 else 0.5
        self.timer = self.create_timer(period, self.publish_collision_object)
        self.connection_timer = self.create_timer(1.0, self.report_connection)
        self.publish_collision_object()
        self.get_logger().info(
            f"Publishing '{self.object_id}' on {self.collision_topic} in "
            f"{self.frame_id}: size={self.box_size_xyz}, "
            f"centre={self.origin_xyz}, every {period:.2f}s."
        )

    def _declare_vector3(self, name, default):
        values = list(self.declare_parameter(name, default).value)
        if len(values) != 3:
            raise ValueError(
                f"Parameter '{name}' must contain exactly 3 numeric values.")
        return [float(value) for value in values]

    def report_connection(self):
        subscription_count = self.publisher.get_subscription_count()
        if subscription_count == self.last_subscription_count:
            return

        self.last_subscription_count = subscription_count
        if subscription_count == 0:
            self.get_logger().warning(
                f"No subscriber on {self.collision_topic}. "
                f"MoveIt cannot receive {self.object_id}."
            )
        else:
            self.get_logger().info(
                f"{self.collision_topic} has {subscription_count} subscriber(s)."
            )

    def publish_collision_object(self):
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = self.object_id
        collision_object.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(self.box_size_xyz)

        pose = Pose()
        pose.position.x = self.origin_xyz[0]
        pose.position.y = self.origin_xyz[1]
        pose.position.z = self.origin_xyz[2]
        pose.orientation.w = 1.0

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)

        self.publisher.publish(collision_object)


def main(args=None):
    rclpy.init(args=args)
    node = UpperBoundPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
