#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

class HandPositionMarkerNode(Node):
    def __init__(self):
        super().__init__('gesture_pose_marker')
        
        # Subscriber für /hand_position
        self.subscription = self.create_subscription(
            Pose,
            '/hand_pose', self.hand_position_callback,10)
        self.get_logger().info("Hand Pose Visualization Node initialized")
        
        # Publisher für Marker
        self.marker_publisher = self.create_publisher(Marker, '/gesture_pose_marker', 10)
    
    def hand_position_callback(self, pose_msg):
        # Erstelle einen Marker für die Sphäre
        marker = Marker()
        marker.header.frame_id = "base" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gesture_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position aus Pose-Nachricht übernehmen
        marker.pose.position.x = -(pose_msg.position.x)
        marker.pose.position.y = -(pose_msg.position.y)
        marker.pose.position.z = pose_msg.position.z
        marker.pose.orientation = pose_msg.orientation
        
        # Größe der Sphäre (Radius)
        marker.scale.x = 0.05  # 10 cm
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # Farbe der Sphäre
        marker.color.r = 1.0  
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Vollständig sichtbar
        
        # Marker veröffentlichen
        self.marker_publisher.publish(marker)
        # self.get_logger().info(f"Marker published at position: {pose_msg.position}")

def main(args=None):
    rclpy.init(args=args)
    node = HandPositionMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
