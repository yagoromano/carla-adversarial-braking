import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2 
from std_msgs.msg import Float32, String
from sklearn.cluster import DBSCAN
import csv, time
from cv_bridge import CvBridge

import json, hmac, hashlib, os

SECRET_KEY = os.environ.get("LIDAR_HMAC_KEY", "super_secret_key").encode()

class LidarDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        # Subscribe to the LiDAR PointCloud2 topic
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/carla/ego_vehicle/lidar', self.lidar_callback, 10
        )

        # self.distance_publisher = self.create_publisher(
        #     Float32, '/carla/ego_vehicle/lidar_distance', 10
        # )
        self.distance_publisher = self.create_publisher(
            String, '/carla/ego_vehicle/lidar_distance', 10
        )

        self.depth_sub = self.create_subscription(
            Image, '/carla/ego_vehicle/depth_front/image', self.depth_callback, 10
        )

        #self.clustered_pointcloud_pub = self.create_publisher(PointCloud2, "/lidar/clustered_points", 10)

        self.bridge = CvBridge()
        self.latest_depth_image = None
        self.image_width = 400
        self.image_height = 70

        self.csv_file = "lidar_points.csv"
        
        #with open(self.csv_file, 'w', newline='') as csvfile:
        #    writer = csv.writer(csvfile)
        #    writer.writerow(['x', 'y', 'z'])


    def publish_signed_distance(self, distance: float):
        # 1) prepare payload
        ts = int(time.time() * 1000)
        payload_dict = {
            "distance": round(distance,3),
            "timestamp": ts
        }
        # 2) compute signature over “distance|timestamp”
        data = f"{payload_dict['distance']}|{ts}".encode()
        sig  = hmac.new(SECRET_KEY, data, hashlib.sha256).hexdigest()
        payload_dict["signature"] = sig

        # 3) publish as JSON string
        msg = String()
        msg.data = json.dumps(payload_dict)
        self.distance_publisher.publish(msg)
        self.get_logger().info(f"Key sent {payload_dict}")

    def write_points_to_csv(self, objects):
        with open(self.csv_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)

            # Write each object as a single row
            for obj in objects:
                points = obj['points']

                # Round values to 2 decimals for each coordinate
                x_values = [round(point[0], 2) for point in points]
                y_values = [round(point[1], 2) for point in points]
                z_values = [round(point[2], 2) for point in points]

                # Write a single row with grouped points and object dimensions
                writer.writerow([
                    ",".join(map(str, x_values)),  # Join x values as a single string
                    ",".join(map(str, y_values)),  # Join y values as a single string
                    ",".join(map(str, z_values))  # Join z values as a single string
                ])
 
    def lidar_callback(self, msg):
        # Parse PointCloud2 data using the same approach as lidar_fusion.py
        points = self.parse_pointcloud2(msg)

        # Process points into objects
        closest_distance = float(self.find_objects(points))

        

        # Process queued points
        #closest_distance = float(self.queued_points(points))
 
        # Testing authentication key
        self.publish_signed_distance(closest_distance)

        # distance_msg = Float32()
        # distance_msg.data = closest_distance
        # self.distance_publisher.publish(distance_msg)

        #self.write_points_to_csv(objects)

    def parse_pointcloud2(self, msg):
        # Use sensor_msgs_py.point_cloud2 to read points
        lidar_points = pc2.read_points(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
        )
        # Convert the data to a NumPy array
        return np.array(
            [(point[0], point[1], point[2], point[3]) for point in lidar_points],
            dtype=np.float32
        )


    def find_objects(self, points):
        """Filters points before applying DBSCAN clustering to detect objects in the lane."""
        filtered_points = []
        
        for point in points:
            x, y, z = point[0], point[1], point[2]
            if -1.5 <= y <= 1.5 and z > -1.9 and 2.5 < x < 50:
                filtered_points.append([x, y, z])

        # Convert list to numpy array for DBSCAN
        if len(filtered_points) < 5:
            return float(9999.99)  # No valid points to cluster

        filtered_points = np.array(filtered_points)


        # ### Attack Number 1 ###
        # num_points_to_remove = int(0.2 * len(filtered_points))  # 20% of total points
        # indices_to_keep = np.random.choice(len(filtered_points), len(filtered_points) - num_points_to_remove, replace=False)
        # attacked_points = filtered_points[indices_to_keep]

        # # Attacked points #
        # clusters = self.cluster_points(attacked_points)

        # Apply DBSCAN only on filtered points
        clusters = self.cluster_points(filtered_points)


        valid_distances = []
        closest_cluster = None

        for cluster in clusters:
            x_mean = np.mean(cluster[:, 0])
            z_mean = np.mean(cluster[:, 2])  
            valid_distances.append(x_mean)
            #self.get_logger().info(f"Cluster Detected: x_mean={x_mean:.2f}, z_mean={z_mean:.2f}")

            if closest_cluster is None or x_mean < min(valid_distances):
                closest_cluster = cluster


        if valid_distances:
            closest_distance = min(valid_distances)
            #depth_value = self.process_depth_for_cluster(closest_cluster)
            self.get_logger().info(f"Detected Object at {closest_distance:.2f} meters")

            #if depth_value is not None:
            #    self.get_logger().info(f"Depth Camera Estimated Distance: {depth_value:.2f} meters")
            return closest_distance

        return 9999.99
    

    def queued_points(self, points):
        # Initialize persistent attributes if they don't exist.
        if not hasattr(self, 'points_queue'):
            self.points_queue = []
        if not hasattr(self, 'first_delayed'):
            self.first_delayed = False  # Indicates whether the first message has been processed.
        if not hasattr(self, 'first_message_time'):
            self.first_message_time = None

        # Enqueue the incoming points.
        self.points_queue.append(points)

        # If this is the very first message, set its timestamp.
        if self.first_message_time is None:
            self.first_message_time = time.time()

        # For the first message, wait until 1 second has passed.
        if not self.first_delayed:
            if time.time() - self.first_message_time < 0.3:
                self.get_logger().info("Waiting for 1 second delay on first message.")
                # Return a default value until the first message is ready.
                return 9999.99
            else:
                # Mark that the first message has been delayed and will now be processed.
                self.first_delayed = True

        # Process the oldest message in the queue.
        if self.points_queue:
            pts = self.points_queue.pop(0)
            return self.find_objects(pts)
        
        return 9999.99
    

    def cluster_points(self, points):
        """
        Simple clustering using DBSCAN for proximity grouping.
        """
        clustering = DBSCAN(eps=0.8, min_samples=6)
        labels = clustering.fit_predict(points[:, :3])
        

        clusters = []
        for label in set(labels):
            if label == -1:  # Ignore noise
                continue
            clusters.append(points[labels == label])

        return clusters

    def depth_callback(self, image: Image):
        """Processes depth image and stores it for use in LiDAR processing."""
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        #self.get_logger().info("Received new depth image.")

    def process_depth_for_cluster(self, cluster):
        """Estimates object depth from the depth camera based on LiDAR cluster position."""

        if self.latest_depth_image is None:
            self.get_logger().warn("No depth image received yet.")
            return None  

        depth_map = self.latest_depth_image.astype(np.float32)

        # Camera intrinsic parameters (from CARLA settings)
        image_w = self.image_width
        image_h = self.image_height
        camera_fov = 90.0 

        # Compute focal length
        focal = image_w / (2.0 * np.tan(np.radians(camera_fov) / 2.0))

        # Camera intrinsic matrix
        K = np.array([
            [focal, 0, image_w / 2.0],
            [0, focal, image_h / 2.0],
            [0, 0, 1]
        ])

        # Store depth values
        depth_values = []

        for i, point in enumerate(cluster):
            X, Y, Z = point[:3]  

            # Transform LiDAR point to camera coordinate frame
            point_in_camera_coords = np.array([-Y, -Z, X])  

            # Project onto image plane
            projected = K @ point_in_camera_coords
            u, v = projected[:2] / projected[2]  # Normalize by depth

            # Convert to integer pixel values
            u, v = int(round(u)), int(round(v))

            # Ensure pixel coordinates are valid
            if not (0 <= u < image_w and 0 <= v < image_h):
                self.get_logger().warn(f"Projected coordinates out of bounds: u={u}, v={v}")
                continue

            # Extract depth value from image
            depth_value = depth_map[v, u]

            # Ignore zero-depth pixels
            if depth_value > 0:
                depth_values.append(depth_value)
                #self.get_logger().info(f"Depth at (u={u}, v={v}): {depth_value:.2f} meters")

        # Return the average depth of valid points
        if depth_values:
            estimated_depth = np.mean(depth_values)
            self.get_logger().info(f"Estimated Depth: {estimated_depth:.2f} meters")
            return estimated_depth

        #self.get_logger().warn("No valid depth values found for cluster.")
        return None



def main(args=None):
    rclpy.init(args=args)
    node = LidarDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Object Detector")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
