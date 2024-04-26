from typing import Callable, Optional
from dora import DoraStatus
import json
from json import *
import dora
import pyarrow as pa
import numpy as np
from typing import Dict, List

new_header={
                "frame_id": "map",
                 "stamp": {"sec": np.int32(111), "nanosec": np.uint32(222)}
            }
path_data_dict = {
                "header":new_header,
                "poses": []}
# path_data_dict = {
#                 "header":new_header,
#                 "poses": [{
#                 "orientation": {"w": np.float64(0), "x": np.float64(0), "y": np.float64(0), "z":np.float64(0)},
#                 "position": {"x": np.float64(0), "y": np.float64(0), "z": np.float64(0)}
#             }]}
class ROS2Path:
    def __init__(self, header: Dict, poses: List):
        self.header = header
        self.poses = poses

    def to_ros_format(self) -> Dict:
        # 将Python对象转换为ROS2兼容的格式
        ros_path = {
            'header': {
                'frame_id': self.header['frame_id'],
                'stamp': {
                    'sec': self.header['stamp']['sec'],
                    'nanosec': self.header['stamp']['nanosec'],
                }
            },
            'poses': [{
                'pose': {
                    'position': {
                        'x': pose['position']['x'],
                        'y': pose['position']['y'],
                        'z': pose['position']['z'],
                    },
                    'orientation': {
                        'x': pose['orientation']['x'],
                        'y': pose['orientation']['y'],
                        'z': pose['orientation']['z'],
                        'w': pose['orientation']['w'],
                    }
                },
                'header': {
                    'frame_id': self.header['frame_id'],
                    'stamp': {
                        'sec': self.header['stamp']['sec'],
                        'nanosec': self.header['stamp']['nanosec'],
                    }
                }
            } for pose in self.poses]
        }
        return ros_path
    
class Operator:
    def __init__(self) -> None:
        print("Python Operator Init")
        """Called on initialisation"""
         # Create a ROS2 Context
        self.ros2_context = dora.experimental.ros2_bridge.Ros2Context()
        self.ros2_node = self.ros2_context.new_node(
            "path2ros",
            "/ros2_bridge",
            dora.experimental.ros2_bridge.Ros2NodeOptions(rosout=True),
        )

        # Define a ROS2 QOS
        self.topic_qos = dora.experimental.ros2_bridge.Ros2QosPolicies(
            reliable=True, max_blocking_time=0.1
        )

        # Create a publisher to imu topic
        # self.path_data_topic = self.ros2_node.create_topic(
        #     "/ros2_bridge/lidar_data", "std_msgs::String", self.topic_qos
        # )

        self.path_data_topic = self.ros2_node.create_topic(
            "/ros2_bridge/Path_data", "nav_msgs::Path", self.topic_qos
        )

        self.path_data_publisher = self.ros2_node.create_publisher(self.path_data_topic)
 
    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:
        print("Python Operator working")

        if dora_event["type"] == "INPUT":
            data = dora_event["value"].to_pylist()
            # print("====================")
            # print(data)
            # print("=====================")
            json_string = ''.join(chr(int(num)) for num in data)
            # js
            print(json_string)
            # 假设 json_string 是收到的 JSON 数据字符串
            json_dict = json.loads(json_string)
            
            o_w = json_dict["pose"]["orientation"]["w"]
            o_x = json_dict["pose"]["orientation"]["x"]
            o_y = json_dict["pose"]["orientation"]["y"]
            o_z = json_dict["pose"]["orientation"]["z"]
            p_x = json_dict["pose"]["position"]["x"]
            p_y = json_dict["pose"]["position"]["y"]
            p_z = json_dict["pose"]["position"]["z"]
            new_pose = {
    "orientation": {"w": np.float64(o_w), "x": np.float64(o_x), "y": np.float64(o_y), "z": np.float64(o_z)},
    "position": {"x": np.float64(p_x), "y": np.float64(p_y), "z": np.float64(p_z)}
}
            path_data_dict["poses"].append(new_pose)
            
            ros2_path = ROS2Path(path_data_dict['header'], path_data_dict['poses'])
            ros_format_path = ros2_path.to_ros_format()

      
            self.path_data_publisher.publish(pa.array([ros_format_path]))
        return DoraStatus.CONTINUE
        
