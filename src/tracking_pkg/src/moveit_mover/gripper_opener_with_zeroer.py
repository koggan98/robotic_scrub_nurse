#!/usr/bin/env python3

# gripper_mover true = gripper open
# gripper_mover false = gripper close
# gripper_zeroer true = sensing aktiv
# gripper_zeroer false = sensing inaktiv

# offsetten: node.reset_force_offset()
# in code offsetten: self.reset_force_offset()


import time
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool, Int32


class URCommand:
    def __init__(self, robot_ip, robot_command_port, gripper_port):
        self.robot_ip = robot_ip
        self.robot_command_port = robot_command_port
        self.gripper_port = gripper_port
        self.socket_ur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_ur.connect((self.robot_ip, self.robot_command_port))
        self.socket_gripper = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_gripper.connect((self.robot_ip, self.gripper_port))
        print('UR Node started.')

    def ur_command(self, command):
        full_command = f"def my_prog():\n{command}\nend\n"
        try:
            self.socket_ur.sendall(full_command.encode('utf-8'))
            print(f'Sent URScript command: {full_command}')
        except socket.error as e:
            print(f"Socket error: {e}")

    def gripper_command(self, command):
        self.socket_gripper.sendall(command.encode('utf-8'))
        #print(f'Sent Gripper command: {command}')

    def command_gripper(self, position, speed=255, force=255):
        if 0 <= position <= 255 and 0 <= speed <= 255 and 0 <= force <= 255:
            command_pos = f'SET POS {position}\n'
            self.gripper_command(command_pos)
            command_speed = f'SET SPE {speed}\n'
            self.gripper_command(command_speed)
            command_force = f'SET FOR {force}\n'
            self.gripper_command(command_force)
            command_gto = 'SET GTO 1\n'
            self.gripper_command(command_gto)

            # Berechnung der Dauer für den Gripper-Vorgang
            time_for_speed = 4 - (3.25 * (speed - 1) / 254)
            time_for_position = (position / 255) * time_for_speed
            time_to_sleep = time_for_speed - time_for_position

            time.sleep(time_to_sleep)
        else:
            print('Invalid gripper command. Position, speed and force must be between 0 and 255.')

    def close_connections(self):
        self.socket_ur.close()
        self.socket_gripper.close()
        print('Closed connection to robot.')


class SocketControllerNode(Node):
    def __init__(self):
        super().__init__('socket_controller')
        self.robot_ip = "192.168.12.10"
        #self.robot_ip = "192.168.12.21"
        #self.robot_ip = "mirur_ur3e_eth"
        self.robot_command_port = 30002
        self.gripper_port = 63352
        self.ur_node = URCommand(self.robot_ip, self.robot_command_port, self.gripper_port)
        self.get_logger().info("Socket Mover Node initialized")
        self.declare_parameter("reclaim.close_speed", 255)
        self.declare_parameter("reclaim.close_force", 1)
        self.reclaim_close_speed = int(self.get_parameter("reclaim.close_speed").value)
        self.reclaim_close_force = int(self.get_parameter("reclaim.close_force").value)

        # publisher für gripper-status
        self.status_publisher = self.create_publisher(Bool, '/gripper_done', 10)
        self.gripper_position_done_publisher = self.create_publisher(Bool, '/gripper_position_done', 10)

        # Offsets für Kraft und Drehmoment
        self.force_offset = None
        self.torque_offset = None

        # Bool für Node aktivierung
        self.zeroer_active_ = False

        # Subscriber für die Kraftsensor-Daten
        self.subscription = self.create_subscription(WrenchStamped,'/force_torque_sensor_broadcaster/wrench', self.force_callback, 10)

        # Subscriber für den Gripper-Befehl
        self.subscription2 = self.create_subscription(Bool, '/gripper_mover', self.gripper_mover_callback, 10)

        # Subscriber für den Gripper-Zeroer
        self.subscription3 = self.create_subscription(Bool, '/gripper_zeroer', self.gripper_zeroer_callback, 10)
        # Subscriber für explizite Positionskommandos
        self.subscription4 = self.create_subscription(Int32, '/gripper_position_command', self.gripper_position_callback, 10)

    def gripper_mover_callback(self, bool_msg):
        # Extrahiere bool aus nachricht
        gripper_state = bool_msg.data
        if gripper_state:
            self.get_logger().info(f"Opening gripper")
            self.ur_node.command_gripper(175, speed=255, force=1)
        else:
            self.get_logger().info(f"Closing gripper")
            self.ur_node.command_gripper(250, speed=255, force=1)           


    def gripper_zeroer_callback(self, bool_msg):
        # Extrahiere bool aus nachricht
        self.zeroer_active_ = bool_msg.data
        self.get_logger().info(f"Aktivitätsstatus: {'Aktiv' if self.zeroer_active_ else 'Inaktiv'}")
        if self.zeroer_active_:
            self.reset_force_offset()

    def gripper_position_callback(self, position_msg):
        position = int(position_msg.data)
        if not 0 <= position <= 255:
            self.get_logger().error(f"Invalid reclaim gripper position: {position}")
            return

        self.get_logger().info(f"Moving gripper to reclaim position {position}")
        self.ur_node.command_gripper(
            position,
            speed=self.reclaim_close_speed,
            force=self.reclaim_close_force,
        )

        done_msg = Bool()
        done_msg.data = True
        self.gripper_position_done_publisher.publish(done_msg)
        self.get_logger().info("Published reclaim gripper position completion.")

    def force_callback(self, msg):
        if not self.zeroer_active_:
            return
        
        # Falls der Offset noch nicht gesetzt wurde, speichere ihn als Nullpunkt
        if self.force_offset is None:
            self.force_offset = msg.wrench.force
            self.torque_offset = msg.wrench.torque
            self.get_logger().info("🔹 Offset gespeichert: Setze aktuelle Werte als Null.")

        # Korrigierte Werte berechnen
        force_x = msg.wrench.force.x - self.force_offset.x
        force_y = msg.wrench.force.y - self.force_offset.y
        force_z = msg.wrench.force.z - self.force_offset.z

        # Nur wenn sich die Kraft von der Nullposition signifikant ändert, soll der Greifer öffnen
        if abs(force_x) > 2 or abs(force_y) > 2 or abs(force_z) > 2:
            self.get_logger().info("Force threshold exceeded, opening gripper.")
            self.ur_node.command_gripper(150, speed=255, force=1) # 0 = auf, 255 = ganz zu
            msg = Bool()
            msg.data = True
            self.status_publisher.publish(msg)
            self.get_logger().info("Gripper open.")
            self.zeroer_active_ = False


    def reset_force_offset(self):
        """Setzt die Kraft- und Drehmomentwerte auf 0 zurück."""
        self.force_offset = None
        self.torque_offset = None
        self.get_logger().info("Kraftsensor-Offset wurde zurückgesetzt!")

    def destroy_node(self):
        self.ur_node.close_connections()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SocketControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down by user request.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
