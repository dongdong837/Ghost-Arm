"""Fortress 6.8.1 emits XYZ in camera-link axes despite its optical header."""
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2


def main():
    rclpy.init()
    node = rclpy.create_node('camera_cloud_frame_fix')
    cloud_qos = QoSProfile(depth=1)
    publisher = node.create_publisher(PointCloud2, '/camera/points', cloud_qos)

    def receive(message):
        # Values are already x-forward/y-left/z-up: correct only their label.
        message.header.frame_id = 'camera_link'
        publisher.publish(message)

    subscription = node.create_subscription(
        PointCloud2, '/camera/points_raw', receive, cloud_qos)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
