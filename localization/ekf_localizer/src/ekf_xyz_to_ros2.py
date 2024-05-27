from typing import Callable, Optional
from dora import DoraStatus
import json
from json import *
import dora
import pyarrow as pa
import numpy as np
from typing import Dict, List
import time
from datetime import datetime


new_header={
                "frame_id": "map",
                "stamp": {"sec": np.int32(111), "nanosec": np.uint32(222)}
            }
# path_data_dict = {
#                 "header":new_header,
#                 "poses": []}
odom_data_dict = {
                "header":new_header,
                "pose": {
                "orientation": {"w": np.float64(0), "x": np.float64(0), "y": np.float64(0), "z":np.float64(0)},
                "position": {"x": np.float64(0), "y": np.float64(0), "z": np.float64(0)}
            }}
class ROS2Odom:
    def __init__(self, header: Dict, pose: List):
        self.header = header
        self.pose = pose

    def to_ros_format(self) -> Dict:
        # 将Python对象转换为ROS2兼容的格式
        
        current_time = time.time()
        sec = (current_time)
        nanosec = ((current_time - sec) * 1e9)
        
        ros_odom = {
            'header': {
                'frame_id': self.header['frame_id'],
                'stamp': {
                    'sec': np.int32(sec),
                    'nanosec': np.uint32(nanosec),
                }
            },
            'pose': {
                'pose': {
                    'position': {
                        'x': self.pose['position']['x'],
                        'y': self.pose['position']['y'],
                        'z': self.pose['position']['z'],
                    },
                    'orientation': {
                        'x': self.pose['orientation']['x'],
                        'y': self.pose['orientation']['y'],
                        'z': self.pose['orientation']['z'],
                        'w': self.pose['orientation']['w'],
                    }
                },
                # 'header': {
                #     'frame_id': self.header['frame_id'],
                #     'stamp': {
                #         'sec': np.int32(sec),
                #         'nanosec': np.uint32(nanosec),
                #     }
                # }
            }  
        }
        return ros_odom
    
class Operator:
    def __init__(self) -> None:
        # with open("out.txt", 'w') as self.file:
        #     self.file.write('# x y z qx qy qz qw \n')
        print("Python dora2rviz2 Operator Init")
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
            "/ros2_bridge/ekf/Odometry", "nav_msgs::Odometry", self.topic_qos
        )

        self.path_data_publisher = self.ros2_node.create_publisher(self.path_data_topic)
 
    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:


        if dora_event["type"] == "INPUT":
            data = dora_event["value"].to_pylist()
            now = datetime.now()# 获取当前时间
            timestamp_ms = round(now.timestamp() * 1000)# 转换为毫秒格式
            print(timestamp_ms)# 打印时间戳（毫秒）
            json_string = ''.join(chr(int(num)) for num in data)
            # js
            #print(json_string)
            # 假设 json_string 是收到的 JSON 数据字符串
            json_dict = json.loads(json_string)
            # 从 JSON 数据中提取关键字
            p_x = json_dict["position"]["x"]
            p_y = json_dict["position"]["y"]
            p_z = json_dict["position"]["z"]
            p_z = 0
        
            o_w = json_dict["orientation"]["w"]
            o_x = json_dict["orientation"]["x"]
            o_y = json_dict["orientation"]["y"]
            o_z = json_dict["orientation"]["z"]
 
            new_pose = {
                "orientation": {"w": np.float64(o_w), "x": np.float64(o_x), "y": np.float64(o_y), "z": np.float64(o_z)},
                "position": {"x": np.float64(p_x), "y": np.float64(p_y), "z": np.float64(p_z)}
            }
            odom_data_dict["pose"] = new_pose
            
            ros2_odom = ROS2Odom(odom_data_dict['header'], odom_data_dict['pose'])
            ros_format_odom = ros2_odom.to_ros_format()

      
            self.path_data_publisher.publish(pa.array([ros_format_odom]))
        return DoraStatus.CONTINUE
        
