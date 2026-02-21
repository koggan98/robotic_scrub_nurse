#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import Mesh, MeshTriangle
import trimesh
import os

def get_mesh_path(file_name):
    return os.path.join(os.getcwd(), 'src', 'tracking_pkg', 'src', 'publisher', 'collisionObjects', 'stls', file_name)


class SurgeryScenePublisher(Node):
    def __init__(self):
        super().__init__('surgery_scene_publisher')
        self.publisher = self.create_publisher(CollisionObject, 'collision_object', 10)
        self.publish_all_objects()
        self.get_logger().info('SurgeryScenePublisher gestartet.')

    def add_mesh(self, mesh_path, object_id, position, orientation, scale=(1.0, 1.0, 1.0)):
        try:
            mesh = trimesh.load(mesh_path)
            mesh.apply_scale(scale)
            triangles = []
            vertices = []

            for face in mesh.faces:
                triangle = MeshTriangle(vertex_indices=[face[0], face[1], face[2]])
                triangles.append(triangle)

            for vertex in mesh.vertices:
                point = Point(x=float(vertex[0]), y=float(vertex[1]), z=float(vertex[2]))
                vertices.append(point)

            # Erstelle das Mesh-Objekt
            mesh_msg = Mesh()
            mesh_msg.triangles = triangles
            mesh_msg.vertices = vertices

            # Erstelle das CollisionObject
            collision_object = CollisionObject()
            collision_object.header.frame_id = 'base'
            collision_object.id = object_id
            collision_object.meshes = [mesh_msg]

            # Definiere die Pose des Meshs
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = position
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation

            collision_object.mesh_poses = [pose]
            collision_object.operation = CollisionObject.ADD

            # Publiziere das CollisionObject
            self.publisher.publish(collision_object)
            self.get_logger().info(f'Mesh "{object_id}" zur Planning Scene hinzugefügt.')
        except Exception as e:
            self.get_logger().error(f'Fehler beim Laden des Meshs {mesh_path}: {e}')

    def publish_all_objects(self):
        objects = [
            {"file": "emptyTray.STL", "id": "empty_tray", "position": (-0.30, 0.1725, -0.725), "orientation": (1.57, -1.60, 0.0, 0.0), "scale": (0.0008, 0.0008, 0.0008)},
            {"file": "medicalBed.stl", "id": "medical_bed", "position": (0.60, -0.90, 0.00), "orientation": (1.58, 0.0, 0.0, 0.0), "scale": (0.01, 0.01, 0.01)},
            #{"file": "human.STL", "id": "human_model", "position": (-0.2, 0.65, 0.05), "orientation": (0.707, 0.0, 0.707, 0.0), "scale": (0.0008, 0.0008, 0.0008)},
            #{"file": "tube.STL", "id": "tube", "position": (-0.6, -0.3, -0.73), "orientation": (0.707, 0.0, 0.707, 0.0), "scale": (0.0008, 0.0008, 0.0008)},
            #{"file": "kipferli.STL", "id": "kipferli", "position": (0.2, 0.0, -0.2), "orientation": (0.707, 0.0, 0.707, 0.0), "scale": (0.001, 0.001, 0.001)},
            #{"file": "turningTube.STL", "id": "turning_tube", "position": (0.0, 0.25, 0.0), "orientation": (0.707, 0.0, 0.0, 0.707), "scale": (0.001, 0.001, 0.001)}
        ]

        for obj in objects:
            mesh_path = get_mesh_path(obj["file"])
            self.add_mesh(mesh_path, obj["id"], obj["position"], obj["orientation"], obj["scale"])

        # Node nach einmaligem Publizieren beenden
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SurgeryScenePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
