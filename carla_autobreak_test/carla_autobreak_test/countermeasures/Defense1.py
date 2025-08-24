import numpy as np
import torch
from copy import deepcopy
from ultralytics import YOLO

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from rosgraph_msgs.msg import Clock
import csv, os, math, time, threading
from simple_pid import PID

class YOLONode(CompatibleNode):
    def __init__(self):
        super().__init__('YOLONode')
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        self.yolo_model = self.init_model()
        
        self.role_name = self.get_param("role_name", "ego_vehicle")
        
        self.camera_views = ['front', 'front_right', 'front_left']
        self.image_subscribers = {}
        self.image_publishers = {}
        self.camera_info_subscribers = {}
        self.camera_info_publishers = {}
        
        for view in self.camera_views:
            self.image_subscribers[view] = self.new_subscription(
                Image, f"/carla/{self.role_name}/rgb_{view}/image",
                lambda msg, view=view: self.on_view_image(msg, view), qos_profile=10
            )
            self.camera_info_subscribers[view] = self.new_subscription(
                CameraInfo, f"/carla/{self.role_name}/rgb_{view}/camera_info",
                lambda msg, view=view: self.on_cam_info(msg, view), qos_profile=10
            )
            self.image_publishers[view] = self.new_publisher(
                Image, f"/carla/{self.role_name}/yolo_{view}/image",
                qos_profile=QoSProfile(depth=10)
            )
            self.camera_info_publishers[view] = self.new_publisher(
                CameraInfo, f"/carla/{self.role_name}/yolo_{view}/camera_info",
                qos_profile=QoSProfile(depth=10)
            )
        
        # Create a publisher to publish the object detection results list as a comma-separated string
        self.object_detection_publisher = self.new_publisher(
            String, f"/carla/{self.role_name}/yolo/object_detection_list",
            qos_profile=QoSProfile(depth=10)
        )

        # Create a publisher for vehicle control
        self.vehicle_control_publisher = self.new_publisher(
            CarlaEgoVehicleControl, f"/carla/{self.role_name}/vehicle_control_cmd",
            qos_profile=QoSProfile(depth=10)
        )

        # Subscribe to the /clock topic to get simulation time
        self.clock_subscriber = self.new_subscription(
            Clock, "/clock", self.clock_callback, qos_profile=QoSProfile(depth=10)
        )

        # Subscribe to the vehicle status topic to get the current speed
        self.vehicle_status_subscriber = self.new_subscription(
            CarlaEgoVehicleStatus, f"/carla/{self.role_name}/vehicle_status",
            self.vehicle_status_callback, qos_profile=QoSProfile(depth=10)
        )

        # Initialize logging
        self.init_logging()

        # Vehicle control state
        self.vehicle_speed = 0.0  # initial speed in km/h
        self.stop_sign_detected = False
        self.stop_sign_confidence_threshold = 0.5
        self.stop_sign_distance = float('inf')
        self.right_stop_sign_distance = float('inf')
        self.simulation_time = 0.0  # initialize simulation time
        self.throttle_delay = 6.0  # delay in seconds before starting throttle
        self.brake_applied = False  # flag to track if braking for stop sign
        self.initial_stop_sign_distance = None
        self.right_side_stop_detected = False
        self.target_speed_kmh = 80.0 # 80 # Target speed in km/h
        self.pid = PID(0.8, 0.05, 0.1, setpoint=self.target_speed_kmh)
        self.pid.output_limits = (0, 1)  # Throttle value between 0 and 1


    def init_model(self) -> YOLO:
        yolo = YOLO('yolov8x.pt') # load pretrained model
        return yolo

    def init_logging(self):
        base_log_dir = "yolo_detections"
        os.makedirs(base_log_dir, exist_ok=True)
        self.log_dir = os.path.join(base_log_dir, f"NewTest")
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_file = os.path.join(self.log_dir, 'Test.csv')
        with open(self.log_file, 'w', newline='') as csvfile:
            fieldnames = ['timestamp', 'class', 'bbox', 'confidence']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

    def log_detection(self, timestamp, detection):
        with open(self.log_file, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=['timestamp', 'class', 'bbox', 'confidence'])
            writer.writerow({
                'timestamp': timestamp,
                'class': detection['class'],
                'bbox': detection['bbox'],
                'confidence': detection['confidence']
            })

    def on_view_image(self, image: Image, view: str):
        array = np.frombuffer(image.data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))  # BGRA format
        img_array = array[:, :, :3]  # BGR 
        img_array = img_array[:, :, ::-1]  # RGB
        img_array = np.moveaxis(img_array, -1, 0)  # move the channel axis to the front

        W = image.width
        FOV = 60.0  # Field of view of the middle camera

        tensor = torch.from_numpy(img_array.copy()).float().to(self.device)
        tensor = tensor.unsqueeze(0)
        try:
            results = self.yolo_model.predict(tensor, conf=0.5, device=self.device, half=True, stream=True)
            
            bounding_boxes_image = deepcopy(image)
            bb_im_array = np.zeros((image.height, image.width, 4), dtype=np.uint8)
            od_str = String()
            detection_list = []

            for result in results:
                if result.boxes is not None and len(result.boxes.xyxy) > 0:
                    results_array = result.plot()
                    bb_im_array[:,:,:3] = results_array[:,:,::-1] 
                    bb_im_array[:,:,3] = array[:,:,3]  # copy the alpha channel
                    bb_im_array = np.ravel(bb_im_array)  # flatten the array
                    bounding_boxes_image._data = bb_im_array.tobytes() 

                    for (box, cls, conf) in zip(result.boxes.xyxy, result.boxes.cls, result.boxes.conf):
                        class_name = result.names[int(cls)]
                        bbox = box.cpu().numpy().tolist()
                        confidence = float(conf.cpu().numpy())
                        detection_list.append(f"{class_name} {bbox} {confidence}")

                        # Calculate the angle of the detection
                        x_center = (bbox[0] + bbox[2]) / 2
                        angle = ((x_center - W / 2) / (W / 2)) * (FOV / 2)

                        # Log the detection
                        timestamp = self.get_clock().now().to_msg().sec
                        detection_data = {
                            'class': class_name,
                            'bbox': bbox,
                            'confidence': confidence,
                            'angle': angle
                        }
                        self.log_detection(timestamp, detection_data)
                        # Handle vehicle control logic for stop sign
                        if self.handle_detection(class_name, angle, bbox, confidence, view):
                            self.stop_sign_detected = True
                            #self.loginfo(f"Stop sign detected with confidence {confidence}")
                        else:
                            self.stop_sign_detected = False
                            
                        if self.handle_right_detection(class_name, bbox, view): 
                            self.right_side_stop_detected = True
                        else: 
                            self.right_side_stop_detected = False

            od_str.data = ";".join(detection_list)
            self.image_publishers[view].publish(bounding_boxes_image)
            self.object_detection_publisher.publish(od_str)

            # Call publish_vehicle_control directly
            self.publish_vehicle_control()

        except Exception as e:
            self.logwarn(f"YOLO error: {e}")

    def handle_right_detection(self, class_name, bbox, view):
        if self.brake_applied and class_name == 'stop sign' and view == 'front_right':
            bbox_height = bbox[3] - bbox[1]
            estimated_distance = self.estimate_distance(bbox_height)
            self.right_stop_sign_distance = estimated_distance
            #self.loginfo(f"Right stop sign. Distance: {self.right_stop_sign_distance}")
            return True
        else: 
            return False
    def handle_detection(self, class_name, angle, bbox, confidence, view):
        if class_name == 'stop sign' and 0 <= angle <= 30 and view == 'front':
            bbox_height = bbox[3] - bbox[1]
            estimated_distance = self.estimate_distance(bbox_height)

            self.loginfo(f"{class_name} , {angle} , {confidence} , {view}")

            # Capture the first detected distance
            if self.initial_stop_sign_distance is None:
                self.initial_stop_sign_distance = estimated_distance
                self.loginfo(f"First stop sign detected. Initial Distance: {self.initial_stop_sign_distance}")
            self.stop_sign_distance = estimated_distance
            #self.loginfo(f"Detected stop sign. Distance: {self.stop_sign_distance}, Confidence: {confidence}, Angle: {angle}")          
            return True
        else: 
            return False


    def on_cam_info(self, cam_info: CameraInfo, view: str):
        try:
            self.camera_info_publishers[view].publish(cam_info)
        except Exception as e:
            self.logwarn(f"Could not send YOLO camera info {e}")
    
    def clock_callback(self, clock: Clock):
        self.simulation_time = clock.clock.sec + clock.clock.nanosec * 1e-9
        #self.loginfo(f"{self.simulation_time}")

    def vehicle_status_callback(self, status: CarlaEgoVehicleStatus):
        # Convert velocity from m/s to km/h
        velocity_kmh = status.velocity * 3.6
        #self.loginfo(f"Vehicle speed {self.vehicle_speed}")

        # Update the vehicle speed and log it
        self.vehicle_speed = velocity_kmh
        self.publish_vehicle_control()

    def estimate_distance(self, bbox_height):
        # This function estimates distance based on the bounding box height
        real_stop_sign_height = 1.0
        image_height = 640  # height of the camera sensor in pixels
        fov = 60.0  # field of view of the camera in degrees
        
        # Calculate the focal length in pixels
        focal_length = (image_height / 2) / math.tan(math.radians(fov / 2))
        
        # Distance estimation formula
        distance = (real_stop_sign_height * focal_length) / bbox_height

        first_detection = False

        if not first_detection:
            print(f"First stop sign detected! Distance: {distance:.2f} meters")
            self.first_detection = True
        

        return distance
    def publish_vehicle_control(self):
        control_msg = CarlaEgoVehicleControl()
        max_deceleration = 9.81  # Max deceleration (m/s^2)

        if not hasattr(self, 'start_time'):
            self.start_time = time.time()
        if not hasattr(self, 'initial_stop_sign_distance'):
            self.initial_stop_sign_distance = None

        vehicle_speed_ms = self.vehicle_speed / 3.6
        deceleration = (vehicle_speed_ms ** 2) / (2 * self.stop_sign_distance)
        # Ensure throttle delay is respected
        elapsed_time = time.time() - self.start_time
        if elapsed_time < self.throttle_delay:
            control_msg.throttle = 0.0
            control_msg.brake = 1.0
        elif self.stop_sign_detected and self.stop_sign_distance < 40:
            
            control_msg.throttle = 0.0
            self.loginfo(f"Saved velocity {self.vehicle_speed}, front")
            control_msg.brake = min((deceleration / max_deceleration)*0.6, 0.8)               
            self.brake_applied = True
        elif not self.brake_applied:
            # Control logic to maintain target speed in km/h
            throttle_output = float(self.pid(self.vehicle_speed))
            control_msg.throttle = max(throttle_output, 0.5)
            control_msg.brake = 0.0
        elif self.right_side_stop_detected:
            control_msg.throttle = 0.0
            self.loginfo(f"Saved velocity {self.vehicle_speed}, side")
            control_msg.brake = min((deceleration / max_deceleration)*0.8, 1.0)   

        self.vehicle_control_publisher.publish(control_msg)

        

def main(args=None):
    roscomp.init("yolo", args=args)

    try:
        executor = roscomp.executors.MultiThreadedExecutor()
        yolo_node = YOLONode()
        executor.add_node(yolo_node)

        while roscomp.ok():
            yolo_node.spin()
        
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.shutdown()

if __name__ == '__main__':
    main()
