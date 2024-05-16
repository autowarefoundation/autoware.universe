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
        print("Python Operator working")
        if "DoraNavSatFix" == dora_input["id"]:
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
            pose["position"]["x"] = json_dict["x"]
            pose["position"]["y"] = json_dict["y"]
            pose["position"]["z"] = json_dict["z"]
            pose["position"]["vx"] = 0.0
            pose["position"]["vy"] = 0.0
            pose["position"]["vz"] = 0.0
        if "DoraQuaternionStamped" == dora_input["id"]:
            data = dora_input["value"].to_pylist()
            json_string = ''.join(chr(int(num)) for num in data)
            # jss
            print(json_string)
            # 假设 json_string 是收到的 JSON 数据字符串
            pose["header"]["frame_id"] = "gnss"
            pose["header"]["stamp"]["sec"] = json_dict["sec"]
            pose["header"]["stamp"]["nanosec"] = json_dict["nanosec"]
            pose["orientation"]["x"] = json_dict["x"]
            pose["orientation"]["y"] = json_dict["y"]
            pose["orientation"]["z"] = json_dict["z"]
            pose["orientation"]["w"] = json_dict["w"]
  
        # 发布JSON-DoraNavSatFix消息
        json_string = json.dumps(pose, indent=4)  # 使用indent参数设置缩进宽度为4
        print(json_string)
        json_bytes = json_string.encode('utf-8')
        send_output("DoraGnssPose",json_bytes,dora_input["metadata"],)


    
