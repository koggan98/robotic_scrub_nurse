#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

class JointStateServer(Node):
    def __init__(self):
        super().__init__('joint_state_server')
        self.latest_joint_state = None

        self.sub_joint = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.sub_trigger = self.create_subscription(Bool, '/request_joint_state', self.trigger_callback, 10)
        self.pub_response = self.create_publisher(JointState, '/joint_state_buffered', 10)
        self.get_logger().info(f"Joint States Node Active")

    def joint_callback(self, msg):
        self.latest_joint_state = msg

    def trigger_callback(self, msg):
        if msg.data and self.latest_joint_state:
            self.get_logger().info('Sending buffered joint state')
            self.pub_response.publish(self.latest_joint_state)
        elif msg.data:
            self.get_logger().warn('Trigger received but no joint state buffered yet!')


def main():
    rclpy.init()
    node = JointStateServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()