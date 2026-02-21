#!/usr/bin/env python3

import cv2
import time
from mediapipe_and_gesture import *
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np
from scipy.spatial.transform import Rotation as R
from threading import Thread
from visualization_msgs.msg import Marker
import tf2_ros

class HandPositionPublisher(Node):
    def __init__(self):
        super().__init__('hand_tracker')  # ROS-Node initialisieren
        self.publisher_ = self.create_publisher(Pose, 'hand_pose', 10)
        self.marker_publisher = self.create_publisher(Marker, 'hand_pose_marker', 10)
        
        self.bridge = CvBridge()
        self.mp_tracker = MediaPipeTracker()
        self.gesture_tracker = HandGestureTracker()

        self.rgb_image = None
        self.depth_image = None
        self.camera_matrix = None

        # Abonnieren der RGB, Tiefenbilder und Parameter
        self.create_subscription(Image, 'color_image', self.rgb_callback, 10)
        self.create_subscription(Image, 'depth_image', self.depth_callback, 10)
        self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, 10)

        # Image publisher erstellung
        self.annotated_image_publisher = self.create_publisher(Image, 'annotated_hand_image', 10)

        #Für den TF erhalt
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"Tracking Node initialized")
        self.waiting_for_images_logged = False # Flag für einmalige Ausgabe von "Warte auf RGB- und Tiefenbilder..."

    def rgb_callback(self, msg):
        """Speichert das RGB-Bild aus der ROS-Message."""
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.waiting_for_images_logged = False  # Sobald ein Bild empfangen wird, zurücksetzen

    def depth_callback(self, msg):
        """Speichert das Tiefenbild aus der ROS-Message."""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        self.waiting_for_images_logged = False  # Sobald ein Bild empfangen wird, zurücksetzen

    def camera_info_callback(self, msg):
        """Speichert die Kameramatrix aus der ROS-Message."""
        # self.get_logger().info("Kameraparameter empfangen")
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.waiting_for_images_logged = False  # Sobald Cam-Info empfangen wird, zurücksetzen


    def transform_to_metric(self, pixel, depth):
        """
        Berechnet die Kamerakoordinaten aus Pixelposition und Tiefe.
        """

        if self.camera_matrix is None:
            self.get_logger().warn("Kameraparameter sind noch nicht empfangen worden.")
            return None
    
        u, v = pixel
        z = depth

        # Aus der Kameramatrix die intrinsischen Parameter extrahieren
        fx = self.camera_matrix[0, 0]  # Brennweite in x-Richtung
        fy = self.camera_matrix[1, 1]  # Brennweite in y-Richtung
        cx = self.camera_matrix[0, 2]  # Principal Point in x-Richtung
        cy = self.camera_matrix[1, 2]  # Principal Point in y-Richtung

        # Pixel -> Kamerakoordinaten
        x_camera = (u - cx) / fx * z
        y_camera = (v - cy) / fy * z
        return np.array([x_camera, y_camera, z])

    def transform_camera_to_world(self, camera_coords):
        try:
            # Warte auf die Transformationen
            trans_camera_to_board = self.tf_buffer.lookup_transform('aruco_board_frame', 'camera_frame', rclpy.time.Time())
            self.get_logger().info(f"trans_camera_to_board: {trans_camera_to_board}")

            trans_board_to_world = self.tf_buffer.lookup_transform('world', 'aruco_board_frame', rclpy.time.Time())
            self.get_logger().info(f"trans_board_to_world: {trans_board_to_world}")

            # Transformationen in numpy-Matrizen umwandeln
            T_camera_to_board = self.transform_to_matrix(trans_camera_to_board)
            T_board_to_world = self.transform_to_matrix(trans_board_to_world)

            # Gesamttransformation berechnen
            T_camera_to_robot_base = np.dot(T_board_to_world, T_camera_to_board)
            self.get_logger().info(f"T_camera_to_robot_base: \n{T_camera_to_robot_base}")

        except Exception as e:
            self.get_logger().fatal("CRITICAL: No Aruco markers detected!")

        try:
            # Kamerakoordinaten in homogene Form bringen
            point_camera = np.array([camera_coords[0], camera_coords[1], camera_coords[2], 1])
            self.get_logger().info(f"point_camera: {point_camera}")

            # Punkt in Roboter-Base-Koordinaten transformieren
            point_robot = np.dot(T_camera_to_robot_base, point_camera)
            self.get_logger().info(f"point_robot: {point_robot}")

            return point_robot[:3]  # Nur x, y, z zurückgeben

        except Exception as e:
            self.get_logger().warn(f"Fehler beim Transformieren der Koordinaten: {e}")
            return None


    def transform_to_matrix(self, transform):
        """Konvertiert eine TransformStamped-Nachricht in eine 4x4-Transformationsmatrix."""
        t = transform.transform.translation
        q = transform.transform.rotation

        # Rotation als Matrix
        rotation = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        # Transformationsmatrix erstellen
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = [t.x, t.y, t.z]

        return T


    def publish_hand_marker(self, position):
        marker = Marker()
        marker.header.frame_id = 'camera_frame'  # Frame, in dem die Hand erkannt wird
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        self.marker_publisher.publish(marker)

        
    def run(self):

        while rclpy.ok():
            if self.rgb_image is None or self.depth_image is None:
                if not self.waiting_for_images_logged:
                    self.get_logger().warn("Warte auf RGB- und Tiefenbilder...")
                    self.waiting_for_images_logged = True  # Nur einmal ausgeben
                time.sleep(0.1)
                continue

            color_frame = self.rgb_image.copy()
            depth_frame = self.depth_image.copy()

            # MediaPipe verarbeiten
            results = self.mp_tracker.process_frame(color_frame)
            hand_landmarks = results.multi_hand_landmarks

            if hand_landmarks:
                self.mp_tracker.draw_landmarks(color_frame, hand_landmarks)

                for hand in hand_landmarks:
                    # Berechne die Mitte der Handfläche
                    x, y = self.gesture_tracker.get_hand_center_wrist(hand, color_frame.shape, self.mp_tracker.mp_hands)

                    if 0 <= y < depth_frame.shape[0] and 0 <= x < depth_frame.shape[1]:
                        z = depth_frame[y, x] / 1000.0  # Tiefe in Metern
                    else:
                        z = None

                    if z is not None:
                        camera_coords = self.transform_to_metric((x, y), z)
                        
                        self.publish_hand_marker(camera_coords)

                        if self.gesture_tracker.detect_double_open_close(hand, self.mp_tracker.mp_hands):
                            point_in_world = self.transform_camera_to_world(camera_coords)
                            hand_position = Pose()
                            hand_position.position.x = float(point_in_world[0]) # Hier roboterpunkte
                            hand_position.position.y = float(point_in_world[1])
                            hand_position.position.z = float(point_in_world[2])
                            hand_position.orientation.x = -0.63
                            hand_position.orientation.y = 0.63
                            hand_position.orientation.z = -0.321
                            hand_position.orientation.w = 0.321
                            self.publisher_.publish(hand_position)

                            self.get_logger().info(f"Handposition veröffentlicht (Roboterframe): x={point_in_world[0]}, y={point_in_world[1]}, z={point_in_world[2]}")
                     # Zeichne die Tiefe der Handflächenmitte auf das Bild
                    if z is not None:
                        cv2.circle(color_frame, (x, y), 5, (255, 0, 0), thickness=2)
                        cv2.putText(color_frame, f"{z}m", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                        
            # Zeige das annotierte RGB-Bild
            # cv2.imshow('MediaPipe Hands with Depth', color_frame) # image publishen "normal"
            ros_image = self.bridge.cv2_to_imgmsg(color_frame, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_frame"
            self.annotated_image_publisher.publish(ros_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

def main(args=None):
    rclpy.init(args=args)
    hand_position_publisher = HandPositionPublisher()

    # Starte die ROS-Logik in einem separaten Thread
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(hand_position_publisher)

    spin_thread = Thread(target=executor.spin)
    spin_thread.start()

    try:
        hand_position_publisher.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spin_thread.join()
        hand_position_publisher.destroy_node()
        rclpy.shutdown()




if __name__ == "__main__":
    main()
    
