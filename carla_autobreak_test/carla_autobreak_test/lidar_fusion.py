import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32MultiArray, Float32
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from matplotlib import cm
import sensor_msgs_py.point_cloud2 as pc2
import os, csv

# Define the colormap
VIRIDIS = np.array(cm.get_cmap('viridis').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])

class LidarCameraOverlayNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_overlay_node')
        self.declare_parameter("camera_topic", "/carla/ego_vehicle/rgb_front/image")
        self.declare_parameter("lidar_topic", "/carla/ego_vehicle/lidar")
        self.declare_parameter("output_topic", "/carla/ego_vehicle/lidarfusion/image")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 640)
        self.declare_parameter("camera_fov", 60.0)
        self.declare_parameter("dot_extent", 2)
        self.image_width = self.get_parameter("image_width").get_parameter_value().integer_value
        self.image_height = self.get_parameter("image_height").get_parameter_value().integer_value

        self.bridge = CvBridge()

        # Subscriptions and Publishers for LiDAR Overlay only
        self.image_subscriber = self.create_subscription(
            Image, self.get_parameter("camera_topic").get_parameter_value().string_value, self.image_callback, 10
        )
        self.lidar_subscriber = self.create_subscription(
            PointCloud2, self.get_parameter("lidar_topic").get_parameter_value().string_value, self.lidar_callback, 10
        )
        self.bbox_subscriber = self.create_subscription(
            Float32MultiArray, "/carla/ego_vehicle/yolo/bbox", self.bbox_callback, 10
        )
        self.overlay_publisher = self.create_publisher(
            Image, self.get_parameter("output_topic").get_parameter_value().string_value, 10
        )
        self.distance_publisher = self.create_publisher(
            Float32, '/carla/ego_vehicle/lidar/estimated_distance', 10
        )


        # Placeholder for synchronized data
        self.current_image = None
        self.current_lidar = None
        self.bbox = None
        self.init_logging()

    def init_logging(self):
        base_directory = "lidar_logs"
        os.makedirs(base_directory, exist_ok=True)
        self.log_file = os.path.join(base_directory, 'lidar_distances.csv')
        with open(self.log_file, 'w', newline='') as csvfile:
            fieldnames = ['timestamp', 'distances', 'x (depth)', 'y (left/right)', 'z (up/down)']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()


    def log_distances(self, distances_before, points_3d_in_bbox):
        # Round distances, widths, and heights for logging
        distances = [round(dist, 2) for dist in distances_before]
        x_values = [round(point[0], 2) for point in points_3d_in_bbox]  # x values
        y_values = [round(point[1], 2) for point in points_3d_in_bbox]  # y values
        z_values = [round(point[2], 2) for point in points_3d_in_bbox]

        # Log the distances, widths, and heights to the CSV file
        with open(self.log_file, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=['timestamp', 'distances', 'x (depth)', 'y (left/right)', 'z (up/down)'])
            writer.writerow({
                'timestamp': self.get_clock().now().to_msg().sec,
                'distances': distances,
                'x (depth)': x_values,
                'y (left/right)': y_values,
                'z (up/down)': z_values,
            })


    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Image conversion error: {e}")
        self.try_to_overlay()

    def lidar_callback(self, msg):
        try:
            # Read the point cloud data and convert it into a structured array (x, y, z, intensity)
            lidar_points = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
            # Convert the data to a NumPy array
            self.current_lidar = np.array(
                [(point[0], point[1], point[2], point[3]) for point in lidar_points],
                dtype=np.float32
            )
        except Exception as e:
            self.get_logger().error(f"PointCloud conversion error: {e}")
        self.try_to_overlay()

    def bbox_callback(self, msg):
        self.bbox = msg.data
        #self.get_logger().info(f"Received bounding box: {self.bbox}")

    def try_to_overlay(self):
        if self.current_image is not None and self.current_lidar is not None:
            self.process_data()

    def process_data(self):
        # Camera parameters and intrinsic matrix setup
        image_w = self.get_parameter("image_width").get_parameter_value().integer_value
        image_h = self.get_parameter("image_height").get_parameter_value().integer_value
        camera_fov = self.get_parameter("camera_fov").get_parameter_value().double_value
        focal = image_w / (2.0 * np.tan(camera_fov * np.pi / 360.0))

        K = np.identity(3)
        K[0, 0] = K[1, 1] = focal
        K[0, 2] = image_w / 2.0
        K[1, 2] = image_h / 2.0

        if self.current_lidar.size > 0:
            points_3d = self.current_lidar[:, :3].T  # LiDAR points in 3D space
            intensity = self.current_lidar[:, 3]

            # Apply FOV mask
            horizontal_angles = np.arctan2(points_3d[1], points_3d[0]) * 180 / np.pi
            fov_mask = np.abs(horizontal_angles) < (camera_fov / 2.0)
            points_3d = points_3d[:, fov_mask]
            intensity = intensity[fov_mask]

            # Transform points to the camera's coordinate frame with height adjustment
            point_in_camera_coords = np.array([-points_3d[1], -points_3d[2], points_3d[0]])

            # Project the 3D points to 2D points on the camera frame
            points_2d = (K @ point_in_camera_coords)[:2] / point_in_camera_coords[2]
            points_2d = points_2d.T

            # Only keep points that are within the camera image boundaries
            points_in_canvas_mask = (
                (points_2d[:, 0] >= 0) & (points_2d[:, 0] < image_w) &
                (points_2d[:, 1] >= 0) & (points_2d[:, 1] < image_h)
            )
            points_2d = points_2d[points_in_canvas_mask]
            points_3d_in_canvas = points_3d.T[points_in_canvas_mask]
            intensity = intensity[points_in_canvas_mask]

            # Only process bounding box if bbox is available
            if self.bbox:
                # Unpack the original bounding box
                x_min, y_min, x_max, y_max = self.bbox

                # Calculate the current dimensions of the bounding box
                box_width = x_max - x_min
                box_height = y_max - y_min

                # Calculate the reduction amounts
                width_reduction = 0.3 * box_width
                height_reduction = 0.3 * box_height  

                # Apply the reduction to keep the box centered
                x_min += width_reduction / 2
                x_max -= width_reduction / 2
                y_min += height_reduction / 2
                y_max -= height_reduction / 2

                # Ensure the reduced bounding box stays within image bounds
                x_min = max(x_min, 0)
                y_min = max(y_min, 0)
                x_max = min(x_max, image_w - 1)
                y_max = min(y_max, image_h - 1)
                
                # Find points within the bounding box
                points_in_bbox_mask = (
                    (points_2d[:, 0] >= x_min) & (points_2d[:, 0] <= x_max) &
                    (points_2d[:, 1] >= y_min) & (points_2d[:, 1] <= y_max)
                )
                points_2d_in_bbox = points_2d[points_in_bbox_mask]
                points_3d_in_bbox = points_3d_in_canvas[points_in_bbox_mask]

                # Log distances
                #distances = np.linalg.norm(points_3d_in_bbox, axis=1)

                # Log distances, widths, and heights of all points within the bounding box
                distances = np.linalg.norm(points_3d_in_bbox, axis=1)
                self.log_distances(distances, points_3d_in_bbox)


                # Log the distances to the CSV file
                #self.log_distances(distances)

                # Choose a method to calculate the correct distance
                if len(distances) > 0:
                    # Clip distances to remove outliers
                    lower_bound = np.percentile(distances, 10)
                    upper_bound = np.percentile(distances, 90)
                    clipped_distances = distances[(distances >= lower_bound) & (distances <= upper_bound)]

                    # Check if clipped_distances is empty
                    if len(clipped_distances) > 0:
                        correct_distance = np.mean(clipped_distances)
                    else:
                        correct_distance = float('inf')  # Default to infinity if no valid points remain

                    # Log the estimated distance
                    self.get_logger().info(f"Estimated distance to stop sign: {correct_distance:.2f} meters")
                else:
                    correct_distance = float('inf')  # Default if no distances were found
                    self.get_logger().info("No valid points found within the bounding box.")

                # Publish the distance as a float
                self.distance_publisher.publish(Float32(data=float(correct_distance)))

                # Clear the bounding box after processing
                self.bbox = None


            # LiDAR fusion continues regardless of bounding box status
            u_coord = points_2d[:, 0].astype(int)
            v_coord = points_2d[:, 1].astype(int)

            # Color mapping based on intensity
            intensity = 4 * intensity - 3
            color_map = np.array([
                np.interp(intensity, VID_RANGE, VIRIDIS[:, 0]) * 255.0,
                np.interp(intensity, VID_RANGE, VIRIDIS[:, 1]) * 255.0,
                np.interp(intensity, VID_RANGE, VIRIDIS[:, 2]) * 255.0
            ]).astype(int).T

            dot_extent = self.get_parameter("dot_extent").get_parameter_value().integer_value
            for i in range(len(points_2d)):
                self.current_image[v_coord[i] - dot_extent:v_coord[i] + dot_extent, u_coord[i] - dot_extent:u_coord[i] + dot_extent] = color_map[i]

            # Publish the fused image
            try:
                self.overlay_publisher.publish(self.bridge.cv2_to_imgmsg(self.current_image, "bgr8"))
            except CvBridgeError as e:
                self.get_logger().error(f"Failed to publish overlay image: {e}")


    def project_lidar_to_image(self, points_3d, intrinsic_matrix, transform_matrix=None):
        if transform_matrix is not None:
            points_3d = (transform_matrix @ points_3d.T).T
        
        points_homogeneous = np.hstack((points_3d, np.ones((points_3d.shape[0], 1))))
        image_points = intrinsic_matrix @ points_homogeneous[:, :3].T
        image_points /= image_points[2, :]
        image_points[0, :] = np.clip(image_points[0, :], 0, self.image_width)
        image_points[1, :] = np.clip(image_points[1, :], 0, self.image_height)
        return image_points[:2, :].T

    def filter_lidar_in_bbox(self, image_points, bbox):
        x_min, y_min, x_max, y_max = bbox
        self.get_logger().info(f"Bounding box coords: {x_min}, {y_min}, {x_max}, {y_max}")

        mask = (image_points[:, 0] >= x_min) & (image_points[:, 0] <= x_max) & \
               (image_points[:, 1] >= y_min) & (image_points[:, 1] <= y_max)
        return mask

def main(args=None):
    rclpy.init(args=args)
    node = LidarCameraOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()