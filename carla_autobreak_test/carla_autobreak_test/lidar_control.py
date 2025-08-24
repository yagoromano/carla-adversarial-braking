import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile

from std_msgs.msg import Float32, String
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from rosgraph_msgs.msg import Clock
from simple_pid import PID

import os, time, hmac, hashlib, json

SECRET_KEY   = os.environ.get("LIDAR_HMAC_KEY", "super_secret_key").encode()
MAX_DRIFT_MS = 500  # ±0.5 s allowed skew

class LidarControl(CompatibleNode):
    def __init__(self):
        super().__init__('lidar_control')

        self.role_name = self.get_param("role_name", "ego_vehicle")

        self.vehicle_control_publisher = self.new_publisher(
            CarlaEgoVehicleControl, f"/carla/{self.role_name}/vehicle_control_cmd",
            qos_profile=QoSProfile(depth=10)
        )

        self.vehicle_status_subscriber = self.new_subscription(
            CarlaEgoVehicleStatus, f"/carla/{self.role_name}/vehicle_status",
            self.vehicle_status_callback, qos_profile=10
        )
        self.simulation_time_subscriber = self.new_subscription(
            Clock, "/clock", self.simulation_time_callback, qos_profile=10
        )

        # self.distance_subscriber = self.new_subscription(
        #     Float32, '/carla/ego_vehicle/lidar_distance', self.distance_callback, 10
        # )

        self.distance_subscriber = self.new_subscription(
            String, '/carla/ego_vehicle/lidar_distance', self.auth_distance_callback, 10
        )

        self.target_speed_kmh = 80
        self.pid = PID(0.8, 0.05, 0.1, setpoint=self.target_speed_kmh)
        self.pid.output_limits = (0, 1)

        self.brake_intensity = 0.8

        self.simulation_time = 0.0
        self.start_time = None
        self.closest_distance = float('inf')
        self.current_speed = 0.0

    def auth_distance_callback(self, msg: String):
        try:
            d = json.loads(msg.data)
            dist = float(d["distance"])
            timestamp = int(d["timestamp"])
            signature = d["signature"]
        except Exception as e:
            self.get_logger().error(f"Bad payload: {e}")
            return

        # rebuild & verify HMAC
        data = f"{dist:.3f}|{timestamp}".encode()
        expect = hmac.new(SECRET_KEY, data, hashlib.sha256).hexdigest()
        now_ms = int(time.time() * 1000)

        if not hmac.compare_digest(expect, signature):
            self.get_logger().error("HMAC mismatch — dropping message")
            return

        if abs(now_ms - timestamp) > MAX_DRIFT_MS:
            self.get_logger().warn("Timestamp drift — dropping (replay?)")
            return

        # passed integrity and replay checks
        self.closest_distance = dist
        self.control_vehicle()

    def vehicle_status_callback(self, status: CarlaEgoVehicleStatus):
        self.current_speed = status.velocity * 3.6
        #self.loginfo(f"Speed: {self.current_speed}")

    def simulation_time_callback(self, clock: Clock):
        self.simulation_time = clock.clock.sec + clock.clock.nanosec * 1e-9

    def distance_callback(self, msg: Float32):
        """Receives object distance and applies acceleration or braking accordingly."""
        self.closest_distance = msg.data
        self.control_vehicle()

    def control_vehicle(self):
        """Controls acceleration and braking based on object distance."""
        control_msg = CarlaEgoVehicleControl()
        vehicle_speed_ms = self.current_speed / 3.6
        deceleration = (vehicle_speed_ms ** 2) / (2 * (self.closest_distance - 2))

        # Wait 5 seconds before enabling object detection
        if self.simulation_time < 10:
            control_msg.throttle = 0.0
            control_msg.brake = 1.0
            #self.get_logger().info(f"Stopped {self.simulation_time:.2f}")

        elif self.closest_distance < 10:
            control_msg.throttle = 0.0
            control_msg.brake = 1.0

        elif self.closest_distance < 40:
            control_msg.throttle = 0.0
            control_msg.brake = min((deceleration / 9.81), 1.0)
            #self.loginfo(f"Closest Distance: {self.closest_distance} Current speed {self.current_speed} Brake force {control_msg.brake}")

        else:

            #self.get_logger().info(f"Accelerating {self.simulation_time:.2f}")
            throttle_output = float(self.pid(self.current_speed))
            control_msg.throttle = min(throttle_output, 1.0)
            control_msg.brake = 0.0

        self.vehicle_control_publisher.publish(control_msg)

def main(args=None):
    roscomp.init("lidar_control", args=args) 

    try:
        executor = roscomp.executors.MultiThreadedExecutor()
        lidar_control_node = LidarControl() 
        executor.add_node(lidar_control_node)

        while roscomp.ok():
            lidar_control_node.spin()

    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.shutdown()

if __name__ == '__main__':
    main()
