import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile
from carla_msgs.msg import CarlaWeatherParameters
from rosgraph_msgs.msg import Clock

import time


class WeatherNode(CompatibleNode):

    def __init__(self):
        super().__init__("WeatherNode")


        self.weather_publisher = self.create_publisher(CarlaWeatherParameters, "/carla/weather_control", 10)
        self.clock_subscription = self.create_subscription(Clock, "/clock", self.clock_callback, 10)
        self.spawn_time = 2000.0  # Time in seconds when the vehicle should be spawned
        self.spawned = False  # Flag to ensure the vehicle is spawned only once
        

    def clock_callback(self, msg):
        current_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        if current_time >= self.spawn_time and not self.spawned:
            self.set_weather_command()
            self.spawned = True        

    def set_weather_command(self):
        msg = CarlaWeatherParameters()

        msg.cloudiness = 0.0
        msg.precipitation = 0.0
        msg.precipitation_deposits = 0.0
        msg.wind_intensity = 0.0
        msg.fog_density = 50.0
        msg.fog_distance = 45.0
        msg.wetness = 0.0
        msg.sun_azimuth_angle = 0.0
        msg.sun_altitude_angle = 40.0

        self.weather_publisher.publish(msg)
        self.get_logger().info("Published weather parameters")

        

        


def main(args=None):
    roscomp.init("weatherTest", args=args)

    try:
        executor = roscomp.executors.MultiThreadedExecutor()
        weather_node = WeatherNode()
        executor.add_node(weather_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            weather_node.destroy_node()
        
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()
