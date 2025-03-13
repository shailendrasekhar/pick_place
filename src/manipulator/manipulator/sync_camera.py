import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import message_filters

class CameraSyncNode(Node):
    def __init__(self):
        super().__init__('camera_sync_node')

        # Subscribers
        image_sub = message_filters.Subscriber(self, Image, "/camera_sensor/depth/image_raw")
        info_sub = message_filters.Subscriber(self, CameraInfo, "/camera_sensor/depth/camera_info")

        # Approximate Synchronization (allows small time mismatches)
        self.sync = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], queue_size=10, slop=0.3)
        self.sync.registerCallback(self.sync_callback)

    def sync_callback(self, image, camera_info):
        self.get_logger().info(f"Synchronized pair received at {image.header.stamp.sec}.{image.header.stamp.nanosec}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraSyncNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()