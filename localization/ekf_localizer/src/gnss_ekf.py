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
from datetime import datetime
import threading



class Operator:
    def __init__(self) -> None:
        self.flage1 =0
        self.flage2=0
        self.pose = {
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
    
    def pub_ekf_pose_pthread(self,dora_event,send_output,dora_input):
            now = datetime.now()# 获取当前时间
            timestamp_ms = round(now.timestamp() * 1000)# 转换为毫秒格式
            # print(timestamp_ms)# 打印时间戳（毫秒）
            # 发布JSON-DoraNavSatFix消息
            json_string = json.dumps(self.pose, indent=4)  # 使用indent参数设置缩进宽度为4
            print("EKF pub time:  ",timestamp_ms," x-y-heading",self.pose["position"]["x"],self.pose["position"]["y"],self.pose["orientation"]["Heading"])
            json_bytes = json_string.encode('utf-8')
            send_output("DoraGnssPose",json_bytes,dora_input["metadata"],)
        
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
        #print("ekf_sub")

        if "DoraNavSatFix" == dora_input["id"]:
            #print("DoraNavSatFix ")
            data = dora_input["value"].to_pylist()
            json_string = ''.join(chr(int(num)) for num in data)
            # js
            #print(json_string)
            # 假设 json_string 是收到的 JSON 数据字符串
            json_dict = json.loads(json_string)
            # 从 JSON 数据中提取关键字
            self.pose["header"]["frame_id"] = json_dict["frame_id"]
            # pose["header"]["stamp"]["sec"] = json_dict["sec"]
            # pose["header"]["stamp"]["nanosec"] = json_dict["nanosec"]
            self.pose["position"]["x"] = np.float64(json_dict["x"])
            self.pose["position"]["y"] = np.float64(json_dict["y"])
            self.pose["position"]["z"] = np.float64(json_dict["z"])
            self.pose["position"]["vx"] = 0.0
            self.pose["position"]["vy"] = 0.0
            self.pose["position"]["vz"] = 0.0
            self.flage1=1
            now = datetime.now()# 获取当前时间
            timestamp_ms = round(now.timestamp() * 1000)# 转换为毫秒格式
            print("DoraNavSatFix seq:", json_dict["seq"]," time: ",timestamp_ms," rx-y-z:  ",self.pose["position"]["x"],"  ",self.pose["position"]["y"],"  ",self.pose["position"]["z"])       
        elif "DoraQuaternionStamped" == dora_input["id"]:
            #print("DoraQuaternionStamped")
            data = dora_input["value"].to_pylist()
            json_string = ''.join(chr(int(num)) for num in data)
            # jss
            #print(json_string)
            json_dict = json.loads(json_string)
            # 假设 json_string 是收到的 JSON 数据字符串
            self.pose["header"]["frame_id"] = "gnss"
            self.pose["header"]["stamp"]["sec"] =  json_dict["sec"]
            self.pose["header"]["stamp"]["nanosec"] =  (json_dict["nanosec"])
            self.pose["orientation"]["x"] = np.float64(json_dict["x"])
            self.pose["orientation"]["y"] = np.float64(json_dict["y"])
            self.pose["orientation"]["z"] = np.float64(json_dict["z"])
            self.pose["orientation"]["w"] = np.float64(json_dict["w"])

            q = np.empty((4, ))
            q[0] =  self.pose["orientation"]["w"]
            q[1] =  self.pose["orientation"]["x"]
            q[2] =  self.pose["orientation"]["y"]
            q[3] =  self.pose["orientation"]["z"]

            q[0] = 0.9586
            q[1] = -0.02
            q[2] = -0.0377
            q[3] = 0.0702
 
            [Roll ,Pitch ,Heading]= euler_from_quaternion(q)
            now = datetime.now()# 获取当前时间
            timestamp_ms = round(now.timestamp() * 1000)# 转换为毫秒格式
            print("QuaternionStamped seq:",json_dict["seq"]," time: ",timestamp_ms," roll-pitch-yaw:  ",Roll,"  ",Pitch,"  ",Heading)            

            self.pose["orientation"]["Roll"] = np.float64(Roll)
            self.pose["orientation"]["Pitch"] = np.float64(Pitch)
            self.pose["orientation"]["Heading"] = np.float64(Heading)
            self.flage2=1
        else:
            if self.flage1>0 and self.flage2>0 :
                now = datetime.now()# 获取当前时间
                timestamp_ms = round(now.timestamp() * 1000)# 转换为毫秒格式
                # print(timestamp_ms)# 打印时间戳（毫秒）
                # 发布JSON-DoraNavSatFix消息
                json_string = json.dumps(self.pose, indent=4)  # 使用indent参数设置缩进宽度为4
                print("EKF pub time:  ",timestamp_ms," x-y-heading",self.pose["position"]["x"],self.pose["position"]["y"],self.pose["orientation"]["Heading"])
                json_bytes = json_string.encode('utf-8')
                send_output("DoraGnssPose",json_bytes,dora_input["metadata"],)
                

        return DoraStatus.CONTINUE

    
