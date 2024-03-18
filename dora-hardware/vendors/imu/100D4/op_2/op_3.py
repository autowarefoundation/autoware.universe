from typing import Callable
from dora import DoraStatus
from typing import Callable
from dora import DoraStatus
import pyarrow as pa
import dora
class Operator:
    """
    Template docstring
    """
    def __init__(self):
        """Called on initialisation"""
         # Create a ROS2 Context
        self.ros2_context = dora.experimental.ros2_bridge.Ros2Context()
        self.ros2_node = self.ros2_context.new_node(
            "turtle_teleop",
            "/ros2_demo_imu",
            dora.experimental.ros2_bridge.Ros2NodeOptions(rosout=True),
        )

        # Define a ROS2 QOS
        self.topic_qos = dora.experimental.ros2_bridge.Ros2QosPolicies(
            reliable=True, max_blocking_time=0.1
        )

        # Create a publisher to imu topic
        self.turtle_imu_topic = self.ros2_node.create_topic(
            "/turtle1/imu", "sensor_msgs::Imu", self.topic_qos
        )
        self.imu_writer = self.ros2_node.create_publisher(self.turtle_imu_topic)
    
    def on_event(
            self,
            dora_event: dict,
            send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":    
            imu_dict = {
                "orientation":{
                    "x": 0.7,
                    "y": 0.8,
                    "z": 0.9,
                    "w": 1.1,
                },
                "angular_velocity": {
                    "x": 0.1,
                    "y": 0.2,
                    "z": 0.3,
                },
                "linear_acceleration": {
                    "x": 0.4,
                    "y": 0.5,
                    "z": 0.6,
                },
            }
            print(imu_dict)
            self.imu_writer.publish(pa.array([imu_dict]))
        
        return DoraStatus.CONTINUE

    def __del__(self):
        """Called before being deleted"""
        pass
    
