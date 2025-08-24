import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import numpy as np
from cv_bridge import CvBridge


class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')

        # Subscribe to Depth Camera Topic
        self.depth_sub = self.create_subscription(
            Image, '/carla/ego_vehicle/depth_front', self.depth_callback, 10
        )
        self.lidar_clusters_sub = self.create_subscription(
            Float32, '/carla/ego_vehicle/lidar_clusters', self.lidar_callback, 10
        )

        # Publisher for Depth-Based Distance
        self.depth_distance_pub = self.create_publisher(
            Float32, '/carla/ego_vehicle/depth_distance', 10
        )

        self.bridge = CvBridge()

    def depth_callback(self, msg):
        """Processes depth image and extracts distance information."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Convert CARLA depth image encoding
        R = cv_image[:, :, 0].astype(np.float32)
        G = cv_image[:, :, 1].astype(np.float32)
        B = cv_image[:, :, 2].astype(np.float32)
        depth_map = (R + G * 256 + B * 256**2) / 1000.0  # Convert to meters

        # Extract the central region for object detection
        image_height, image_width = depth_map.shape
        center_x, center_y = image_width // 2, image_height // 2

        # Extract the closest depth value in the center region
        region_size = 10  # Consider a 10x10 region in the center
        depth_region = depth_map[center_y - region_size // 2:center_y + region_size // 2,
                                 center_x - region_size // 2:center_x + region_size // 2]

        valid_depths = depth_region[depth_region > 0.1]  # Ignore zero-depth pixels

        if valid_depths.size > 0:
            estimated_distance = np.min(valid_depths)
            self.get_logger().info(f"Estimated Depth Distance: {estimated_distance:.2f} meters")

            # Publish the estimated distance
            distance_msg = Float32()
            distance_msg.data = estimated_distance
            self.depth_distance_pub.publish(distance_msg)
        else:
            self.get_logger().warn("No valid depth values found!")

def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Depth Processor")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
