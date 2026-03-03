#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool, Int32


class ReclaimController(Node):
    def __init__(self):
        super().__init__("reclaim_controller")

        self.declare_parameter("reclaim.force_threshold_newtons", 4.0)
        self.declare_parameter("reclaim.zero_delay_seconds", 0.5)
        self.declare_parameter("reclaim.close_position", 250)
        self.declare_parameter("reclaim.close_speed", 255)
        self.declare_parameter("reclaim.close_force", 1)

        self.force_threshold_newtons = float(self.get_parameter("reclaim.force_threshold_newtons").value)
        self.zero_delay_seconds = float(self.get_parameter("reclaim.zero_delay_seconds").value)
        self.close_position = int(self.get_parameter("reclaim.close_position").value)
        self.close_speed = int(self.get_parameter("reclaim.close_speed").value)
        self.close_force = int(self.get_parameter("reclaim.close_force").value)

        if self.zero_delay_seconds < 0.0:
            raise ValueError("reclaim.zero_delay_seconds must be >= 0.0")

        self.state = "idle"
        self.force_offset = None
        self.trigger_armed_time = None

        self.gripper_position_publisher = self.create_publisher(Int32, "/gripper_position_command", 10)
        self.reclaim_done_publisher = self.create_publisher(Bool, "/gripper_reclaim_done", 10)

        self.reclaim_trigger_sub = self.create_subscription(
            Bool,
            "/gripper_reclaim_trigger",
            self.reclaim_trigger_callback,
            10,
        )
        self.force_sub = self.create_subscription(
            WrenchStamped,
            "/force_torque_sensor_broadcaster/wrench",
            self.force_callback,
            10,
        )
        self.gripper_position_done_sub = self.create_subscription(
            Bool,
            "/gripper_position_done",
            self.gripper_position_done_callback,
            10,
        )

        self.get_logger().info(
            f"Reclaim controller initialized with threshold {self.force_threshold_newtons:.2f} N, "
            f"zero delay {self.zero_delay_seconds:.2f} s, "
            f"close position {self.close_position}, speed {self.close_speed}, force {self.close_force}."
        )

    def reclaim_trigger_callback(self, msg: Bool):
        if msg.data:
            self.force_offset = None
            self.trigger_armed_time = self.get_clock().now()
            self.state = "armed_waiting_for_zero_delay"
            self.get_logger().info("Reclaim sensor armed. Waiting before zeroing force baseline.")
            return

        if self.state != "idle":
            self.get_logger().info("Reclaim sensor cancelled.")

        self.reset_state()

    def force_callback(self, msg: WrenchStamped):
        if self.state == "idle" or self.state == "waiting_for_close_ack":
            return

        current_force = (
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
        )

        if self.state == "armed_waiting_for_zero_delay":
            if self.trigger_armed_time is None:
                self.trigger_armed_time = self.get_clock().now()

            elapsed_seconds = (
                self.get_clock().now() - self.trigger_armed_time
            ).nanoseconds / 1_000_000_000.0

            if elapsed_seconds < self.zero_delay_seconds:
                return

            self.force_offset = current_force
            self.state = "armed_waiting_for_force_threshold"
            self.get_logger().info("Reclaim baseline captured after zero delay.")
            return

        if self.state == "armed_waiting_for_force_threshold" and self.force_offset is None:
            self.force_offset = current_force
            self.get_logger().warn("Reclaim force offset was empty unexpectedly; recapturing baseline.")
            return

        force_x = current_force[0] - self.force_offset[0]
        force_y = current_force[1] - self.force_offset[1]
        force_z = current_force[2] - self.force_offset[2]

        if (
            abs(force_x) > self.force_threshold_newtons
            or abs(force_y) > self.force_threshold_newtons
            or abs(force_z) > self.force_threshold_newtons
        ):
            command_msg = Int32()
            command_msg.data = self.close_position
            self.gripper_position_publisher.publish(command_msg)
            self.state = "waiting_for_close_ack"
            self.get_logger().info(
                f"Force threshold exceeded during reclaim. Requested gripper close to {self.close_position}."
            )

    def gripper_position_done_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.state != "waiting_for_close_ack":
            self.get_logger().warn("Ignoring gripper position acknowledgement outside reclaim close.")
            return

        done_msg = Bool()
        done_msg.data = True
        self.reclaim_done_publisher.publish(done_msg)
        self.get_logger().info("Published reclaim completion after gripper close.")
        self.reset_state()

    def reset_state(self):
        self.state = "idle"
        self.force_offset = None
        self.trigger_armed_time = None


def main(args=None):
    rclpy.init(args=args)
    node = ReclaimController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down by user request.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
