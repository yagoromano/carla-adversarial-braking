import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile
from geometry_msgs.msg import Pose
from rosgraph_msgs.msg import Clock
 
 
class SetSpawnNode(CompatibleNode):
 
    def __init__(self):
        super().__init__("SetSpawnNode")
        self.spawn_publisher = self.create_publisher(Pose, "/carla/ego_vehicle/control/set_transform", 10)
        self.clock_subscription = self.create_subscription(Clock, "/clock", self.clock_callback, 10)
        self.spawn_time = 2000.0  # Time in seconds when the vehicle should be spawned
        self.spawned = False  # Flag to ensure the vehicle is spawned only once
 
    def clock_callback(self, msg):
        current_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        if current_time >= self.spawn_time and not self.spawned:
            self.set_spawn_command()
            self.spawned = True
 
    def set_spawn_command(self):
        msg = Pose()
        msg.position.x = 193.72909545898439  # Town 02: 193.72909545898439    
        msg.position.y = -126.0927505493164  # Town 02: -126.0927505493164  
        msg.position.z = 1.0  # Town 02: 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.7066263234361524  # Town 02: 0.7066263234361524 
        msg.orientation.w = 0.7075869127019706  # Town 02: 0.7075869127019706  
 
        self.spawn_publisher.publish(msg)
        self.get_logger().info("Published spawn parameters")
 
def main(args=None):
    roscomp.init("spawnTest", args=args)
 
    try:
        executor = roscomp.executors.MultiThreadedExecutor()
        spawn_node = SetSpawnNode()
        executor.add_node(spawn_node)
 
        try:
            executor.spin()
        finally:
            executor.shutdown()
            spawn_node.destroy_node()
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.shutdown()
 
 
if __name__ == '__main__':
    main()