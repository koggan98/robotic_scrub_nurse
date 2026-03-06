#!/usr/bin/env python3

import socket
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class URCommand:
    def __init__(self, robot_ip: str, robot_command_port: int, gripper_port: int):
        self.socket_ur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_ur.connect((robot_ip, robot_command_port))
        self.socket_gripper = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_gripper.connect((robot_ip, gripper_port))

    def gripper_command(self, command: str) -> None:
        self.socket_gripper.sendall(command.encode("utf-8"))

    def command_gripper(self, position: int, speed: int = 255, force: int = 255) -> None:
        if not (0 <= position <= 255 and 0 <= speed <= 255 and 0 <= force <= 255):
            raise ValueError("Position, speed and force must be between 0 and 255.")

        self.gripper_command(f"SET POS {position}\n")
        self.gripper_command(f"SET SPE {speed}\n")
        self.gripper_command(f"SET FOR {force}\n")
        self.gripper_command("SET GTO 1\n")

        time_for_speed = 4 - (3.25 * (speed - 1) / 254)
        time_for_position = (position / 255) * time_for_speed
        time_to_sleep = time_for_speed - time_for_position
        time.sleep(max(0.0, time_to_sleep))

    def close_connections(self) -> None:
        self.socket_ur.close()
        self.socket_gripper.close()


class SocketGripperController(Node):
    def __init__(self) -> None:
        super().__init__("socket_gripper_controller")

        robot_ip = str(self.declare_parameter("robot_ip", "192.168.12.10").value)
        robot_command_port = int(self.declare_parameter("robot_command_port", 30002).value)
        gripper_port = int(self.declare_parameter("gripper_port", 63352).value)
        self.open_position = int(self.declare_parameter("gripper.open_position", 150).value)
        self.close_position = int(self.declare_parameter("gripper.close_position", 250).value)
        self.reclaim_close_position = int(self.declare_parameter("reclaim.close_position", 250).value)
        self.reclaim_close_speed = int(self.declare_parameter("reclaim.close_speed", 200).value)
        self.reclaim_close_force = int(self.declare_parameter("reclaim.close_force", 1).value)

        self.ur_node = URCommand(robot_ip, robot_command_port, gripper_port)

        self.gripper_position_done_publisher = self.create_publisher(Bool, "/gripper_position_done", 10)
        self.create_subscription(Bool, "/gripper_mover", self._gripper_mover_callback, 10)
        self.create_subscription(
            Int32,
            "/gripper_position_command",
            self._gripper_position_callback,
            10,
        )

        self.get_logger().info("Socket gripper controller initialized.")

    def _gripper_mover_callback(self, msg: Bool) -> None:
        try:
            if msg.data:
                self.get_logger().info("Opening gripper.")
                self.ur_node.command_gripper(self.open_position, speed=255, force=1)
            else:
                self.get_logger().info("Closing gripper.")
                self.ur_node.command_gripper(self.close_position, speed=255, force=1)
        except Exception as exc:
            self.get_logger().error(f"Failed to execute /gripper_mover command: {exc}")

    def _gripper_position_callback(self, msg: Int32) -> None:
        position = int(msg.data)
        if not 0 <= position <= 255:
            self.get_logger().error(f"Invalid gripper position command: {position}")
            return

        speed = self.reclaim_close_speed if position == self.reclaim_close_position else 255
        force = self.reclaim_close_force if position == self.reclaim_close_position else 1

        try:
            self.get_logger().info(
                f"Moving gripper to position {position} with speed {speed} and force {force}."
            )
            self.ur_node.command_gripper(position, speed=speed, force=force)
        except Exception as exc:
            self.get_logger().error(f"Failed to execute /gripper_position_command: {exc}")
            return

        done_msg = Bool()
        done_msg.data = True
        self.gripper_position_done_publisher.publish(done_msg)
        self.get_logger().info("Published gripper position completion.")

    def destroy_node(self) -> bool:
        self.ur_node.close_connections()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SocketGripperController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down by user request.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
