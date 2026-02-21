#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import trimesh
import os
import math
from transforms3d.euler import euler2quat
from ament_index_python.packages import get_package_share_directory

class AddSTLObject(Node):
    def __init__(self):
        super().__init__('add_stl_object')
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)

        pkg_share = get_package_share_directory('tracking_pkg')
        stl_file_path = os.path.join(pkg_share, 'collisionObjects', 'stls', 'base_link.STL')
        
        self.get_logger().info(f"Loading STL from: {stl_file_path}")

        # Load STL file
        mir_mesh = self.load_stl_as_ros_mesh(stl_file_path)

        # Create pose with 90° rotation around Z-axis
        mir_pose = Pose()
        mir_pose.position.x = -0.18
        mir_pose.position.y = 0.0 
        mir_pose.position.z = -1.03/2 + 0.06
        
        # 90° rotation around Z-axis (π/2 radians)
        q = euler2quat(0, 0, 0)  # (roll=0, pitch=0, yaw=90°)
        mir_pose.orientation = Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])

        # Create CollisionObject
        collision_object = CollisionObject()
        collision_object.id = "mir"
        collision_object.header = Header(frame_id="world")
        collision_object.meshes.append(mir_mesh)
        collision_object.mesh_poses.append(mir_pose)
        collision_object.operation = CollisionObject.ADD

        self.publisher.publish(collision_object)
        self.get_logger().info("Published STL object")

    def load_stl_as_ros_mesh(self, stl_path):
        """Load and convert STL to ROS Mesh with vertex transformation"""
        mesh = trimesh.load(stl_path)
        ros_mesh = Mesh()

        # Apply 90° rotation around Z to vertices
        rotation_matrix = trimesh.transformations.rotation_matrix(
            angle=math.pi/2,
            direction=[0, 0, 1]  # Z-axis
        )
        mesh.apply_transform(rotation_matrix)

        # Convert faces
        for face in mesh.faces:
            triangle = MeshTriangle()
            triangle.vertex_indices = [int(i) for i in face]
            ros_mesh.triangles.append(triangle)

        # Convert vertices
        for vertex in mesh.vertices:
            point = Point()
            point.x, point.y, point.z = vertex
            ros_mesh.vertices.append(point)

        return ros_mesh

def main(args=None):
    rclpy.init(args=args)
    node = AddSTLObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()