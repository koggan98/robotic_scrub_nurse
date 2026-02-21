#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class BoxScenePublisher(Node):
    def __init__(self):
        super().__init__('box_scene_publisher')
        self.publisher = self.create_publisher(CollisionObject, 'collision_object', 10)
        self.add_box()
        self.get_logger().info('BoxScenePublisher gestartet.')

    def add_box(self):
        # Erstelle ein CollisionObject
        box = CollisionObject()
        box.header.frame_id = 'base'
        box.id = 'ground_box'
        box.operation = CollisionObject.ADD

        # Definiere die Form und Größe der Box 
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.1, 0.1, 0.1]

        # Definiere die Pose der Box (mittig unter dem Ursprung)
        pose = Pose()
        pose.position.x = -0.3
        pose.position.y = -0.1
        pose.position.z = -0.255  #  Höhe nach unten versetzt

        # Füge die Form und Pose zum CollisionObject hinzu
        box.primitives.append(primitive)
        box.primitive_poses.append(pose)

        # Publiziere das CollisionObject
        self.publisher.publish(box)
        self.get_logger().info('Box zur Planning Scene hinzugefügt.')

        # Node nach einmaligem Publizieren beenden
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BoxScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
