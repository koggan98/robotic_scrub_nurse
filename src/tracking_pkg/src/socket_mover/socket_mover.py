#!/usr/bin/env python3


import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time
from package import *
from subpackage import *
from ur_feedback_socket import *
from scipy.spatial.transform import Rotation as R

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

    def command_position(self, position, orientation, acceleration=3.0, speed=6.0, radius=0):
        # position = np.array(position) 
        script_command = (
            f"movej(p[{position[0]}, {position[1]}, {position[2]}, " 
            f"{orientation[0]}, {orientation[1]}, {orientation[2]}], "
            f"{acceleration}, {speed}, {radius})\n"
        )
        self.ur_command(script_command)


    def command_linear_position(self, position, orientation, acceleration=3.0, speed=6.0, radius=0):
        # position = np.array(position) 
        script_command = (
            f"movel(p[{position[0]}, {position[1]}, {position[2]}, " 
            f"{orientation[0]}, {orientation[1]}, {orientation[2]}], "
            f"{acceleration}, {speed}, {radius})\n"
        )
        self.ur_command(script_command)

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
        # self.robot_ip = "192.168.56.101"  # Simulation
        self.robot_ip = "192.168.1.168" 

        self.robot_command_port = 30002
        self.gripper_port = 63352
        self.ur_node = URCommand(self.robot_ip, self.robot_command_port, self.gripper_port)
        self.first_move_finished = False

        # Subscriber für /hand_pose
        self.subscription = self.create_subscription(Pose,'/hand_pose',self.hand_pose_callback,10)
        self.get_logger().info("Socket Mover Node initialized")

        # Hebe objekt auf
        #self.get_logger().info("Picking up object...")
        #self.ur_node.command_gripper(0, speed=128, force=10)
        #self.ur_node.command_position([0.3, 0.2, 0.15], [3.14159, 0, 0], acceleration=2.0, speed=1.5, radius=0)
        #time.sleep(2)
        #self.ur_node.command_linear_position([0.3, 0.2, 0.1], [3.14159, 0, 0], acceleration=0.5, speed=0.2, radius=0)
        #time.sleep(1)
        #self.ur_node.command_gripper(250, speed=128, force=10)
        #time.sleep(1)
        #self.ur_node.command_linear_position([0.3, 0.2, 0.15], [3.14159, 0, 0], acceleration=2.0, speed=1.5, radius=0)


    def quaternion_to_rpy(self, quaternion):
        """
        Konvertiert eine Quaternion in Roll, Pitch, Yaw (RPY)-Winkel.
        """
        r = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        rpy = r.as_euler('xyz', degrees=False)  # Ausgabe in Radiant
        return rpy  # Rückgabe als [Roll, Pitch, Yaw]
    
    def hand_pose_callback(self, pose_msg):
        # Extrahiere Position und Orientierung aus der Pose-Nachricht
        position = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z + 0.03]
        published_orientation = pose_msg.orientation

        # Konvertiere Quaternionen in RPY-Winkel
        rpy = self.quaternion_to_rpy(published_orientation)

        # Debug-Ausgabe
        self.get_logger().info(f"Received position: {position}, adjusted RPY orientation: {rpy}")

        # Sende die Position und angepasste Orientierung an den Roboter
        if self.first_move_finished == False:
            try:
                self.ur_node.command_position(position,rpy,acceleration=3.0,speed=6,radius=0)
                time.sleep(3)
                self.ur_node.command_gripper(0, speed=255, force=1)  # Example command to set gripper position

                # Fährt zu home
                time.sleep(0.0)
                self.get_logger().info("Movement to hand position completed. Returning to home position...")
                self.ur_node.command_position([0.0, -0.2, 0.3], [3.14159, 0, 0], acceleration=2.0, speed=1.5, radius=0)
                self.first_move_finished = True
            except Exception as e:
                self.get_logger().error(f"Error commanding robot: {str(e)}")

        if self.first_move_finished == True:
            try:
                self.ur_node.command_position(position,rpy,acceleration=3.0,speed=6,radius=0)
               
                # Fährt zu home
                time.sleep(2.0)
                self.get_logger().info("Movement to hand position completed. Returning to home position...")
                self.ur_node.command_position([0.0, -0.2, 0.3], [3.14159, 0, 0], acceleration=2.0, speed=1.5, radius=0)
            except Exception as e:
                self.get_logger().error(f"Error commanding robot: {str(e)}")
            

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
