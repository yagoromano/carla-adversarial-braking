import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile

from std_msgs.msg import Bool, Float32
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from rosgraph_msgs.msg import Clock
from simple_pid import PID
import os, csv

class VehicleControlNode(CompatibleNode):
    def __init__(self):
        super().__init__('VehicleControlNode')
        
        self.role_name = self.get_param("role_name", "ego_vehicle")

        self.stop_sign_detected_subscriber = self.new_subscription(
            Bool, f"/carla/{self.role_name}/yolo/stop_sign_detected",
            self.stop_sign_detected_callback, qos_profile=10
        )
        self.stop_sign_distance_subscriber = self.new_subscription(
            Float32, f"/carla/{self.role_name}/yolo/stop_sign_distance",
            self.stop_sign_distance_callback, qos_profile=10
        )
        self.right_stop_sign_detected_subscriber = self.new_subscription(
            Bool, f"/carla/{self.role_name}/yolo/right_stop_sign_detected",
            self.right_stop_sign_detected_callback, qos_profile=10
        )
        self.right_stop_sign_distance_subscriber = self.new_subscription(
            Float32, f"/carla/{self.role_name}/yolo/right_stop_sign_distance",
            self.right_stop_sign_distance_callback, qos_profile=10
        )
        self.simulation_time_subscriber = self.new_subscription(
            Clock, "/clock", self.simulation_time_callback, qos_profile=10
        )

        self.vehicle_control_publisher = self.new_publisher(
            CarlaEgoVehicleControl, f"/carla/{self.role_name}/vehicle_control_cmd",
            qos_profile=QoSProfile(depth=10)
        )

        self.vehicle_status_subscriber = self.new_subscription(
            CarlaEgoVehicleStatus, f"/carla/{self.role_name}/vehicle_status",
            self.vehicle_status_callback, qos_profile=10
        )
        self.angle_subscriber = self.new_subscription(
            Float32, f"/carla/{self.role_name}/yolo/angle",
            self.angle_callback, qos_profile=10
        )
        self.lidar_distance_subscriber = self.new_subscription(
            Float32,
            '/carla/ego_vehicle/lidar/estimated_distance',
            self.lidar_distance_callback,
            qos_profile=10
        )
        self.init_logging()

        self.vehicle_speed = 0.0
        self.stop_sign_detected = False
        self.right_stop_sign_detected = False
        self.stop_sign_distance = float('inf')
        self.right_stop_sign_distance = float('inf')
        self.simulation_time = 0.0
        self.start_delay = 10.0  # 10 seconds delay before starting to accelerate
        self.brake_applied = False
        self.target_speed_kmh = 75.0
        self.pid = PID(0.8, 0.05, 0.1, setpoint=self.target_speed_kmh)
        self.pid.output_limits = (0, 1)
        self.first_detection = None
        self.lidar_distance = None

    def init_logging(self):
        base_directory = "vehicle_speeds"
        os.makedirs(base_directory, exist_ok=True)
        self.log_file = os.path.join(base_directory, 'speed.csv')
        with open(self.log_file, 'w', newline='') as csvfile:
            fieldnames = ["Simulation Time", "Velocity (km/h)", "Distance to Stop Sign(m)"]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
    def log_vehicle_data(self, velocity, simulation_time, distance_to_stop_sign):
        with open(self.log_file, 'a', newline='') as csvfile:
            fieldnames = ["Simulation Time", "Velocity (km/h)", "Distance to Stop Sign(m)"]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writerow({
                "Simulation Time": simulation_time,
                "Velocity (km/h)": velocity,
                "Distance to Stop Sign(m)": distance_to_stop_sign
            })
    def lidar_distance_callback(self, msg: Float32):
        self.lidar_distance = msg.data
        #self.stop_sign_distance = self.lidar_distance - 3
    def stop_sign_detected_callback(self, msg: Bool):
        self.stop_sign_detected = msg.data
    def stop_sign_distance_callback(self, msg: Float32):
        self.front_distance = msg.data
        self.stop_sign_distance = self.front_distance

    def right_stop_sign_detected_callback(self, msg: Bool):
        self.right_stop_sign_detected = msg.data

    def right_stop_sign_distance_callback(self, msg: Float32):
        self.right_stop_sign_distance = msg.data


    def simulation_time_callback(self, clock: Clock):
        self.simulation_time = clock.clock.sec + clock.clock.nanosec * 1e-9
    def angle_callback(self, msg: Float32):
        self.angle = msg.data

    def vehicle_status_callback(self, status: CarlaEgoVehicleStatus):
        velocity_kmh = status.velocity * 3.6
        self.vehicle_speed = velocity_kmh
        self.publish_vehicle_control()

    def publish_vehicle_control(self):
        control_msg = CarlaEgoVehicleControl()
        max_deceleration = 9.81  # Max deceleration (m/s^2)
        vehicle_speed_ms = self.vehicle_speed / 3.6

        deceleration = (vehicle_speed_ms ** 2) / (2 * self.stop_sign_distance)

        # Store the first stop sign detection distance if it hasn't been set yet
        if self.first_detection is None and self.stop_sign_distance < 40.0:
            self.first_detection = self.stop_sign_distance
            self.loginfo(f"First stop sign detection at distance: {self.first_detection:.2f} meters")

        if hasattr(self, 'angle') and 0 <= self.angle <= 30:
            # Apply a stronger brake multiplier if first detection was less than 40 meters
            if self.first_detection is not None and self.first_detection < 30:
                brake_multiplier = 1.0
                deceleration = 1.0
                #self.loginfo(f'Applying stronger brake Angle: {self.angle}')
            else: 
                
                deceleration = (vehicle_speed_ms ** 2) / (2 * self.stop_sign_distance)
                brake_multiplier = 0.6 + (self.angle / 30) * 0.4 
                #self.loginfo(f'Angle: {self.angle}')
        else:
            brake_multiplier = 0.8  # Default multiplier if angle is negative or not set

        if self.simulation_time < self.start_delay:
            control_msg.throttle = 0.0
            control_msg.brake = 1.0
        elif self.right_stop_sign_detected and self.brake_applied:
            control_msg.throttle = 0.0
            control_msg.brake = 1.0
            self.log_vehicle_data(self.vehicle_speed, self.simulation_time, self.right_stop_sign_distance)  
        elif self.stop_sign_detected and self.stop_sign_distance < 40:   
            control_msg.throttle = 0.0
            #self.loginfo(f"Saved velocity {self.vehicle_speed}, front")
            control_msg.brake = min((deceleration / max_deceleration) * brake_multiplier, 1.0)
            self.log_vehicle_data(self.vehicle_speed, self.simulation_time, self.stop_sign_distance)
            self.brake_applied = True
        elif not self.brake_applied:
            throttle_output = float(self.pid(self.vehicle_speed))
            control_msg.throttle = min(throttle_output, 1.0)
            control_msg.brake = 0.0

        self.vehicle_control_publisher.publish(control_msg)


def main(args=None):
    roscomp.init("vehicle_control", args=args)

    try:
        executor = roscomp.executors.MultiThreadedExecutor()
        vehicle_control_node = VehicleControlNode()
        executor.add_node(vehicle_control_node)

        while roscomp.ok():
            vehicle_control_node.spin()
        
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.shutdown()

if __name__ == '__main__':
    main()
