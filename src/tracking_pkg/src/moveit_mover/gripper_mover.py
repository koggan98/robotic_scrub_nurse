#!/usr/bin/env python3

# gripper_command true = gripper close
# gripper_command false = gripper open

import socket
import rclpy
from rclpy.node import Node
from package import *
from subpackage import *
from ur_feedback_socket import *
from std_msgs.msg import Bool


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
        print(f'Sent Gripper command: {command}')

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
        self.robot_command_port = 30002
        self.gripper_port = 63352
        self.ur_node = URCommand(self.robot_ip, self.robot_command_port, self.gripper_port)
        self.first_move_finished = False

        # Subscriber für /hand_pose
        self.subscription = self.create_subscription(Bool,'/gripper_command',self.gripper_command_callback,10)
        self.get_logger().info("Socket Mover Node initialized")

   
    def gripper_command_callback(self, bool_msg):
        # Extrahiere bool aus nachricht
        gripper_state = bool_msg.data

        if gripper_state:
            self.get_logger().info(f"Closing gripper")
            self.ur_node.command_gripper(250, speed=255, force=1)

        else:
            self.get_logger().info(f"Opening gripper")
            self.ur_node.command_gripper(0, speed=255, force=1)           

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
