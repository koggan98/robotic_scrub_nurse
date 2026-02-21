#!/usr/bin/env python3

# Notfalls TF frame fixieren(Statisch machen) oder filtern
import cv2
import numpy as np
import cv2.aruco as aruco
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import StaticTransformBroadcaster


# Board-Konfiguration
MARKER_SIZE = 0.10  # Größe eines Markers in Metern
MARKER_SEPARATION = 0.15  # Abstand zwischen den Markern in Metern
MARKER_OFFSET_Y = -0.3
MARKER_OFFSET_X = 0.05
BOARD_ROWS = 2
BOARD_COLS = 2
MARKER_IDS = [100,105,110,115]
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()

class ArucoBoard(Node):
    def __init__(self):
        super().__init__('frame_publisher')

        # CvBridge für die Konvertierung
        self.bridge = CvBridge()

        # Abonnieren des RGB-Bildes und der Kameraparameter
        self.rgb_image = None
        self.camera_matrix = None
        self.dist_coeffs = np.zeros((5, 1))

        self.create_subscription(Image, 'color_image', self.rgb_callback, 10)
        self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, 10)

        # TF-Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        #self.static_tf_broadcaster = StaticTransformBroadcaster(self) # Für statisches publishen   

        # ArUco-Parameter
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)

        # Board mit benutzerdefinierten IDs erstellen
        self.board = self.create_board()

        self.parameters = aruco.DetectorParameters()

        self.get_logger().info("ArucoBoard Node initialized.")

        # TF-Broadcaster für statisches Publizieren
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.camera_frame_published = False  # Statusvariable für einmaliges Publizieren


    

    def camera_info_callback(self, msg):
        """Speichert die Kameramatrix aus der ROS-Message."""
        # self.get_logger().info("Kameraparameter empfangen")
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def rgb_callback(self, msg):
        """Speichert das RGB-Bild aus der ROS-Message."""
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_and_publish()

    def create_board(self):
        """Erstellt ein ArUco-Board mit benutzerdefinierten Marker-IDs."""
        objPoints = [self._create_marker_corners(i) for i in MARKER_IDS]
        return cv2.aruco.Board(objPoints=objPoints, dictionary=ARUCO_DICT, ids=np.array(MARKER_IDS, dtype=np.int32).reshape(-1, 1))

    def _create_marker_corners(self, marker_id):
        """Erstellt die Eckpunkte für einen einzelnen Marker im Board."""
        half_size = MARKER_SIZE / 2
        return np.array([
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)
    
    

    def detect_and_publish(self):
        color_frame = self.rgb_image.copy()
        gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)

        # Marker erkennen
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            rvec = np.zeros((1, 3), dtype=np.float64)
            tvec = np.zeros((1, 3), dtype=np.float64)

            # Board-Pose schätzen
            ret, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.board, self.camera_matrix, self.dist_coeffs, rvec, tvec)

            if ret > 0:
                # Rotation in Matrix umwandeln
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                T_c_m = np.eye(4)
                T_c_m[:3, :3] = rotation_matrix
                T_c_m[:3, 3] = tvec.flatten()

                # Publiziere statische TF für camera_frame -> aruco_board_frame nur einmal
                if not hasattr(self, 'camera_frame_published') or not self.camera_frame_published:
                    self.publish_camera_frame(np.linalg.inv(T_c_m), "aruco_board_frame", "camera_frame")
                    self.camera_frame_published = True
                    self.get_logger().info("Static camera_frame published relative to aruco_board_frame.")

                # Publiziere statische TF für aruco_board_frame
                self.publish_aruco_board_frame()

                # Zeichne die Board-Achse
                cv2.drawFrameAxes(color_frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

        #else:
            # Dynamisches Publizieren für den world -> camera_frame Transform
            # print("No markers detected")


            # Zeige das Bild für Debugging
            # cv2.imshow("Aruco Board Detection", color_frame)
            # cv2.waitKey(1)

    def publish_camera_frame(self, matrix, parent_frame, child_frame):
        if self.camera_frame_published:
            return  # Verhindert mehrfaches Publizieren

        rotation_matrix = matrix[:3, :3]
        quat = R.from_matrix(rotation_matrix).as_quat()

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame

        transform.transform.translation.x = matrix[0, 3]
        transform.transform.translation.y = matrix[1, 3]
        transform.transform.translation.z = matrix[2, 3]

        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        # Statisches Publizieren
        self.static_tf_broadcaster.sendTransform(transform)
        self.camera_frame_published = True  # Markiere den Frame als veröffentlicht
        self.get_logger().info(f"Static TF published: {parent_frame} -> {child_frame}")
 

    def publish_aruco_board_frame(self):
        static_transform = TransformStamped()

        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "base"  # Basisframe des Roboters
        static_transform.child_frame_id = "aruco_board_frame"
        # Hard Coded Pose des ArUco-Boards relativ zur Roboterbasis
        static_transform.transform.translation.x = MARKER_OFFSET_X 
        static_transform.transform.translation.y = MARKER_OFFSET_Y 
        static_transform.transform.translation.z = 0.0
        # Rotation: Identisch zur Roboterbasis (keine Rotation)
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(static_transform)
        # self.get_logger().info("Published static TF for aruco_board_frame relative to robot_base")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoBoard()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()