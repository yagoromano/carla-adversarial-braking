import numpy as np
import torch
from copy import deepcopy
from ultralytics import YOLO

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile

from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image, CameraInfo
import math, os, csv

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
        
        self.object_detection_publisher = self.new_publisher(
            String, f"/carla/{self.role_name}/yolo/object_detection_list",
            qos_profile=QoSProfile(depth=10)
        )
        self.stop_sign_detected_publisher = self.new_publisher(
            Bool, f"/carla/{self.role_name}/yolo/stop_sign_detected",
            qos_profile=QoSProfile(depth=10)
        )
        self.stop_sign_distance_publisher = self.new_publisher(
            Float32, f"/carla/{self.role_name}/yolo/stop_sign_distance",
            qos_profile=QoSProfile(depth=10)
        )
        self.right_stop_sign_detected_publisher = self.new_publisher(
            Bool, f"/carla/{self.role_name}/yolo/right_stop_sign_detected",
            qos_profile=QoSProfile(depth=10)
        )
        self.right_stop_sign_distance_publisher = self.new_publisher(
            Float32, f"/carla/{self.role_name}/yolo/right_stop_sign_distance",
            qos_profile=QoSProfile(depth=10)
        )

        self.init_logging()
        self.stop_sign_detected = False
        self.right_side_stop_detected = False
        self.stop_sign_distance = float('inf')
        self.right_stop_sign_distance = float('inf')

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
                            
                        if self.handle_right_detection(class_name, bbox, view) and self.stop_sign_detected: 
                            self.right_side_stop_detected = True

            od_str.data = ";".join(detection_list)
            self.image_publishers[view].publish(bounding_boxes_image)
            self.object_detection_publisher.publish(od_str)

            # Publish detection results
            self.stop_sign_detected_publisher.publish(Bool(data=self.stop_sign_detected))
            self.stop_sign_distance_publisher.publish(Float32(data=self.stop_sign_distance))
            self.right_stop_sign_detected_publisher.publish(Bool(data=self.right_side_stop_detected))
            self.right_stop_sign_distance_publisher.publish(Float32(data=self.right_stop_sign_distance))

        except Exception as e:
            self.logwarn(f"YOLO error: {e}")

    def handle_right_detection(self, class_name, bbox, view):
        if class_name == 'stop sign' and view == 'front_right':
            bbox_height = bbox[3] - bbox[1]
            estimated_distance = self.estimate_distance(bbox_height)
            self.right_stop_sign_distance = estimated_distance
            return True
        else: 
            return False

    def handle_detection(self, class_name, angle, bbox, confidence, view):
        if class_name == 'stop sign' and 0 <= angle <= 30 and view == 'front':
            bbox_height = bbox[3] - bbox[1]
            estimated_distance = self.estimate_distance(bbox_height)

            self.loginfo(f"{class_name} , {angle} , {confidence} , {view}")

            self.stop_sign_distance = estimated_distance
            return True
        else: 
            return False

    def on_cam_info(self, cam_info: CameraInfo, view: str):
        try:
            self.camera_info_publishers[view].publish(cam_info)
        except Exception as e:
            self.logwarn(f"Could not send YOLO camera info {e}")

    def estimate_distance(self, bbox_height):
        real_stop_sign_height = 1.20
        image_height = 640
        fov = 60.0
        
        focal_length = (image_height / 2) / math.tan(math.radians(fov / 2))
        
        distance = (real_stop_sign_height * focal_length) / bbox_height
        return distance

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