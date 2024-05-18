from typing import Callable, Optional
from dora import DoraStatus
import json
from json import *
import dora
import pyarrow as pa
import numpy as np
from typing import Dict, List
import time
import pickle
import math
from transforms3d._gohlketransforms import euler_from_quaternion

pose = {
    'header': 
    {
        'frame_id': "map",
        'stamp': 
        {
            'sec': np.int32(0),
            'nanosec': np.uint32(0),
        }
    },
    "orientation": 
    {
        "Roll": np.float64(0), 
        "Pitch": np.float64(0), 
        "Heading": np.float64(0), 
        "w": np.float64(0), 
        "x": np.float64(0), 
        "y": np.float64(0), 
        "z": np.float64(0)
    },
    "position": 
    {
        "x": np.float64(0), 
        "y": np.float64(0), 
        "z": np.float64(0),
        "vx": np.float64(0), 
        "vy": np.float64(0), 
        "vz": np.float64(0)
    }
    }
class Operator:
    def __init__(self) -> None:
        pass
    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE
    
    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):
        print("gndd_poser_sub")

        if "DoraNavSatFix" == dora_input["id"]:
            #print("DoraNavSatFix   crp")
            data = dora_input["value"].to_pylist()
            json_string = ''.join(chr(int(num)) for num in data)
            # js
            print(json_string)
            # 假设 json_string 是收到的 JSON 数据字符串
            json_dict = json.loads(json_string)
            # 从 JSON 数据中提取关键字
            pose["header"]["frame_id"] = "gnss"
            # pose["header"]["stamp"]["sec"] = json_dict["sec"]
            # pose["header"]["stamp"]["nanosec"] = json_dict["nanosec"]
            pose["position"]["x"] = np.float64(json_dict["x"])
            pose["position"]["y"] = np.float64(json_dict["y"])
            pose["position"]["z"] = np.float64(json_dict["z"])
            pose["position"]["vx"] = 0.0
            pose["position"]["vy"] = 0.0
            pose["position"]["vz"] = 0.0

        if "DoraQuaternionStamped" == dora_input["id"]:
            #print("DoraQuaternionStamped")
            data = dora_input["value"].to_pylist()
            json_string = ''.join(chr(int(num)) for num in data)
            # jss
            #print(json_string)
            json_dict = json.loads(json_string)
            # 假设 json_string 是收到的 JSON 数据字符串
            pose["header"]["frame_id"] = "gnss"
            pose["header"]["stamp"]["sec"] =  json_dict["sec"]
            pose["header"]["stamp"]["nanosec"] =  (json_dict["nanosec"])
            pose["orientation"]["x"] = np.float64(json_dict["x"])
            pose["orientation"]["y"] = np.float64(json_dict["y"])
            pose["orientation"]["z"] = np.float64(json_dict["z"])
            pose["orientation"]["w"] = np.float64(json_dict["w"])

            q = np.empty((4, ))
            q[0] =  pose["orientation"]["w"]
            q[1] =  pose["orientation"]["x"]
            q[2] =  pose["orientation"]["y"]
            q[3] =  pose["orientation"]["z"]
            [Roll ,Pitch ,Heading]= euler_from_quaternion(q)
            print("roll-pitch-yaw:  ",Roll*57.3,"  ",Pitch*57.3,"  ",Heading*57.3)            

            pose["orientation"]["Roll"] = np.float64(Roll)
            pose["orientation"]["Pitch"] = np.float64(Pitch)
            pose["orientation"]["Heading"] = np.float64(Heading)
  
            # 发布JSON-DoraNavSatFix消息
            json_string = json.dumps(pose, indent=4)  # 使用indent参数设置缩进宽度为4
            print(json_string)
            json_bytes = json_string.encode('utf-8')
            send_output("DoraGnssPose",json_bytes,dora_input["metadata"],)
        return DoraStatus.CONTINUE

    
