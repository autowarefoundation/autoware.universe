# ros2 interface show sensor_msgs/msg/NavSatFix
# ros2 interface show nav_msgs/msg/Path
# 查看ROS2消息类型
from typing import Callable, Optional
import pyarrow as pa
from dora import DoraStatus
import dora
import numpy as np
import time
import json
from json import *
import pickle


ros_path = {
            'header': {
                'frame_id': "GNSS",
                'stamp': {
                    'sec': 0,
                    'nanosec': 0,
                }
            },
            'poses': [{
                'pose': {
                    'position': {
                        'x': 0,
                        'y': 0,
                        'z': 0,
                    },
                    'orientation': {
                        'x': 0,
                        'y': 0,
                        'z': 0,
                        'w': 0,
                    }
                },
                'header': {
                    'frame_id': "GNSS",
                    'stamp': {
                        'sec': 0,
                        'nanosec': 0,
                    }
                }
            } ]
        }

class Operator:
    """
    反序列化后，输出各类消息内容
    """

    def __init__(self):
        self.ros2_context = dora.experimental.ros2_bridge.Ros2Context()
        # create ros2 node
        self.ros2_node = self.ros2_context.new_node(
            "gps2ros",
            "/dora_ros2_bridge",
            dora.experimental.ros2_bridge.Ros2NodeOptions(rosout=True)
        )
        # create ros2 qos
        self.topic_qos = dora.experimental.ros2_bridge.Ros2QosPolicies(
            reliable=True, max_blocking_time=0.1
        )
        # create ros2 topic
        self.gps_data_topic = self.ros2_node.create_topic(
            "/dora_ros2_bridge/gps_path",
            "nav_msgs::Path",
            self.topic_qos
        )
        # create ros2 publisher
        self.gps_data_publisher = self.ros2_node.create_publisher(self.gps_data_topic)

    def on_event(
        self,
        dora_event: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):
        if "DoraSentence" == dora_input["id"]:
            pass

        elif "DoraNavSatFix" == dora_input["id"]:
            data = dora_input["value"].to_pylist()
            json_string = ''.join(chr(int(num)) for num in data)
            print(json_string)
            # 假设 json_string 是收到的 JSON 数据字符串
            json_dict = json.loads(json_string)

            # 从 JSON 数据中提取关键字
            x = json_dict["x"]
            y = json_dict["y"]
            z = json_dict["z"]

            # # 输出提取的关键字
            # print("x:", x)
            # print("y:", y)
            # print("z:", z)
            current_time = time.time()
            sec = (current_time)
            nanosec = ((current_time - sec) * 1e9)
            
            pose = {
                    'pose': {
                        'position': {
                            'x': np.float64(x),
                            'y': np.float64(y),
                            'z': np.float64(z),
                        },
                        'orientation': {
                            'x': 0,
                            'y': 0,
                            'z': 0,
                            'w': 1,
                        }
                    },
                    'header': {
                        'frame_id': "GNSS",
                        'stamp': {
                            'sec': sec,
                            'nanosec': nanosec,
                        }
                    }
                    }
            ros_path["poses"].append(pose)
            # ros_path = {
            #             'header': {
            #                 'frame_id': "GNSS",
            #                 'stamp': {
            #                     'sec': sec,
            #                     'nanosec': nanosec,
            #                 }
            #             },
            #             'poses': [{
            #                 'pose': {
            #                     'position': {
            #                         'x': np.float64(x),
            #                         'y': np.float64(y),
            #                         'z': np.float64(z),
            #                     },
            #                     'orientation': {
            #                         'x': 0,
            #                         'y': 0,
            #                         'z': 0,
            #                         'w': 0,
            #                     }
            #                 },
            #                 'header': {
            #                     'frame_id': "GNSS",
            #                     'stamp': {
            #                         'sec': sec,
            #                         'nanosec': nanosec,
            #                     }
            #                 }
            #             } for pose in self.poses]
            #         }
            # gnss_poser = {
            # # "header": {
            # #     "seq": np.uint32(1),
            # #     # "stamp": {
            # #     #     "sec": np.int32(sec),
            # #     #     "nanosec": np.uint32(nanosec),
            # #     # },
            # #     "frame_id": "gps",
            # # },
            # "poses":{
            #     "pose":{
            #         "position": {
            #             "x": np.float64(x),
            #             "y": np.float64(y),
            #             "z": np.float64(z),
            #         },
            #         # "orientation":{
            #         #     "x": 0,
            #         #     "y": 0,
            #         #     "z": 0,
            #         #     "w": 1,
            #         # },
            #         }
            #     }
            # }
            self.gps_data_publisher.publish(pa.array([ros_path]))
            pass

        elif "DoraQuaternionStamped" == dora_input["id"]:
            pass

        elif "DoraTwistStamped" == dora_input["id"]:
            pass
            
        # current_time = time.time()
        # sec = (current_time)
        # nanosec = ((current_time - sec) * 1e9)
        # gps_data_dict = {
        #     "header": {
        #         #"seq": np.uint32(1),
        #         "stamp": {
        #             "sec": np.int32(sec),
        #             "nanosec": np.uint32(nanosec),
        #         },
        #         "frame_id": "gps",
        #     },
        #     "status":{
        #         "status": np.int8(1),
        #         "service": np.uint16(1),          
        #     },
        #     "latitude": np.float64(self.receDoraNavSatFix.latitude),
        #     "longitude": np.float64(self.receDoraNavSatFix.longitude),
        #     "altitude": np.float64(self.receDoraNavSatFix.altitude),
        #     "position_covariance_type":  np.uint8(2)
        # }
        # #print(pa.array([gps_data_dict]))
        # self.gps_data_publisher.publish(pa.array([gps_data_dict]))

        return DoraStatus.CONTINUE