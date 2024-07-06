from typing import Callable, Optional
from dora import DoraStatus
import json
from json import *
import dora
import pyarrow as pa
import numpy as np
from typing import Dict, List
import time
new_header={
                "frame_id": "map",
                 "stamp": {"sec": np.int32(111), "nanosec": np.uint32(222)}
            }
# path_data_dict = {
#                 "header":new_header,
#                 "pose": [],
#                 "color":
#                 {
#                     "a":1.0,
#                     "b":0,
#                     "c":0
#                 }
#             }
marks = {
                "markers":[]
            }
# path_data_dict = {
#                 "header":new_header,
#                 "poses": [{
#                 "orientation": {"w": np.float64(0), "x": np.float64(0), "y": np.float64(0), "z":np.float64(0)},
#                 "position": {"x": np.float64(0), "y": np.float64(0), "z": np.float64(0)}
#             }]}
class ROS2Marks:
    def __init__(self, header: Dict, poses: List):
        self.header = header
        self.poses = poses

    def to_ros_format(self) -> Dict:
        # 将Python对象转换为ROS2兼容的格式
        
        current_time = time.time()
        sec = (current_time)
        nanosec = ((current_time - sec) * 1e9)
        
        # mark = {
        #     'markers': [
        #         {
        #             'header': 
        #             {
        #                 'frame_id': self.header['frame_id'],
        #                 'stamp': 
        #                 {
        #                     'sec': np.int32(sec),
        #                     'nanosec': np.uint32(nanosec),
        #                 }
        #             },
        #             'position': 
        #             {
        #                 'x': pose['x_pos'],
        #                 'y': pose['y_pos'],
        #                 'z': pose['z_pos'],
        #             },
        #             # 'orientation': 
        #             # {
        #             #     'x': pose['orientation']['x'],
        #             #     'y': pose['orientation']['y'],
        #             #     'z': pose['orientation']['z'],
        #             #     'w': pose['orientation']['w'],
        #             # },
        #             'scale':
        #             {
        #                 'x': pose['lwh']['x'],
        #                 'y': pose['lwh']['y'],
        #                 'z': pose['lwh']['z'],
        #             },
        #             'color':
        #             {
        #                 'a': 1.0,
        #                 'b': 0,
        #                 'c': 0,
        #             }
        #         } for pose in self.poses]
        # }

        markers = []
        for pose in self.poses:
            marker = {
                'header': {
                    # 'frame_id': self.header['frame_id'],
                    'frame_id': "rslidar",
                    'stamp': {
                        'sec': np.int32(sec),
                        'nanosec': np.uint32(nanosec),
                    }
                },
                'ns': 'lidar_objects',
                'id': np.int32(pose.get('id', 0)),  # 假设每个pose有唯一的id
                'type': np.int32(1),  # Assuming type as CUBE
                'action': np.int32(0),  # Assuming action as ADD
                'pose': {
                    'position': {
                        'x': np.float64(pose['x_pos']),
                        'y': np.float64(pose['y_pos']),
                        'z': np.float64(pose['z_pos']),
                    }
                },
                'scale': {
                    'x': np.float64(pose['lwh']['x']),
                    'y': np.float64(pose['lwh']['y']),
                    'z': np.float64(pose['lwh']['z']),
                },
                'color': {
                    'r': np.float32(1.0),
                    'g': np.float32(0),
                    'b': np.float32(0),
                    'a': np.float32(1.0),
                },
                'lifetime': {'sec': np.int32(0), 'nanosec': np.uint32(0)},
                'frame_locked': False,
                'points': [],
                # 'colors': [],
                # 'text': '',
                # 'mesh_resource': '',
                'mesh_use_embedded_materials': False
            }

            for i in range(len(pose['bbox_point']['x'])):
                point = {
                    'x': np.float64(pose['bbox_point']['x'][i]),
                    'y': np.float64(pose['bbox_point']['y'][i]),
                    'z': np.float64(pose['bbox_point']['z'][i]),
                }
                marker['points'].append(point)
                # print("<====point====>")
                # print(point)
            
            markers.append(marker)

        print("<====markers====>")
        print(markers)
        return {'markers': markers}
    
class Operator:
    def __init__(self) -> None:
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

        self.path_data_topic = self.ros2_node.create_topic(
            "/ros2_bridge/Path_data", "visualization_msgs::MarkerArray", self.topic_qos
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
            json_string = ''.join(chr(int(num)) for num in data)
            print("<=========json_string=============>")
            print(json_string)
            # 假设 json_string 是收到的 JSON 数据字符串
            json_dict = json.loads(json_string)
            # 从 JSON 数据中提取关键字

            # j["LidarRawObjects"]["objs"] = jsonArray_LidarRawObjects_objs
            
            header = json_dict["header"]
            frame_id = json_dict["header"]["frame_id"]
            cnt = json_dict["header"]["cnt"]
            stamp = json_dict["header"]["stamp"]
            objs = json_dict["LidarRawObjects"]["objs"]
            # new_pose = {
            #     "orientation": {"w": np.float64(0), "x": np.float64(0), "y": np.float64(0), "z": np.float64(0)},
            #     "position": {"x": np.float64(0), "y": np.float64(0), "z": np.float64(0)}
            # }
            # path_data_dict["poses"].append(new_pose)
            
            ros2_marks = ROS2Marks(json_dict["header"], json_dict["LidarRawObjects"]["objs"])
            
            ros_format_marks = ros2_marks.to_ros_format()
            # print("<=========ros_format_marks=============>")
            # print(ros_format_marks)
      
            self.path_data_publisher.publish(pa.array([ros_format_marks]))
        return DoraStatus.CONTINUE
        
