from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from rclpy.node import Node
import rclpy

class CameraTFPublisher(Node):
    def __init__(self):
        super().__init__('camera_tf_publisher')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

    def publish_static_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base"  # Parent frame
        transform.child_frame_id = "camera_frame"  # Your camera's TF frame

        # ✅ Hier trägst du die bekannten Werte ein (in Meter und Quaternion!)
        transform.transform.translation.x = 0.22  # z.B. 10cm vor der Basis
        transform.transform.translation.y = -0.385
        transform.transform.translation.z = 0.07

        # Falls Kamera z.B. nach vorne und leicht geneigt schaut
        from scipy.spatial.transform import Rotation as R
        quat = R.from_euler('xyz', [-60, 0, 60], degrees=True).as_quat() # (-90,0,0) schaut in y achse auf roboter
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        self.static_broadcaster.sendTransform(transform)
        self.get_logger().info("Published static TF from base to camera_frame")

def main(args=None):
    rclpy.init(args=args)
    node = CameraTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
