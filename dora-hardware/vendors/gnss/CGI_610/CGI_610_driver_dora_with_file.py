 
from typing import Callable
from dora import DoraStatus
from Nmea_utils import check_nmea_checksum
from Nmea_utils import parse_nmea_sentence
from DoraNmeaDriver_utils import DoraNMEADriver, Timestamp
from transforms3d._gohlketransforms import quaternion_from_euler
import math
import pickle
import time
import numpy as np
from datetime import datetime


import json
from json import *

"""
 读取目录下的文件 cgi610_outdata ，取其中的数据   
"""

class Operator:

    
    # 打开串口读取数据
    def __init__(self):
        self.f = open("/home/crp/autoware.universe/dora-hardware/vendors/gnss/CGI_610/cgi610_outdata.txt")
        self.cnt1 =0
        self.cnt2 =0
        self.cnt3 =0
        self.driver = DoraNMEADriver()

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
        line = self.f.readline()
        self.data = ''
        if(line) :
        #    print (line)
        #    print(type(line))
            self.data = line 
             
        # 读取一行数据并去除首尾空白字符
        # self.data = self.GPS.readline().strip()
        # self.data = self.GPS.read_all()
        
        # 如果数据非空
        if self.data:
            try:
                if isinstance(self.data, bytes):
                    self.data = self.data.decode("utf-8")
                nmea_sentence = self.data
                
                # 校验NMEA句子
                if not check_nmea_checksum(nmea_sentence):
                    print(
                        "Received a sentence with an invalid checksum. "
                        + "Sentence was: %s" % nmea_sentence
                    )
                    return DoraStatus.CONTINUE
                
                # 按类型解析NMEA数据:return解析的消息类型、解析得到map(对应消息类型各字段的key-value)
                parsed_sentence = parse_nmea_sentence(nmea_sentence)
                
                if not parsed_sentence:
                    print(
                        "Failed to parse NMEA sentence. Sentence was: %s"
                        % nmea_sentence
                    )
                    return DoraStatus.CONTINUE
                # print("flag1")
                # for key, value in parsed_sentence.items():
                #     print(f"Key: {key}, Value: {value}" )
                 
                timestamp = None
                frame_id = self.driver.get_frame_id()

                if timestamp:
                    current_time = timestamp
                else:
                    current_time = Timestamp.now()

                self.driver.current_nmea_string.header.stamp = current_time
                self.driver.current_nmea_string.header.frame_id = frame_id
                self.driver.current_nmea_string.sentence = nmea_sentence
                nmea_Publish_instance = self.driver.current_nmea_string #输出原始字符流
                # 发布DoraSentence消息
                parsed_nmea_sentence = pickle.dumps(nmea_Publish_instance)
                # print(parsed_nmea_sentence)
                send_output(
                    "DoraSentence",
                    parsed_nmea_sentence,
                    dora_input["metadata"],
                )

                if "CHC" in parsed_sentence:
                    data = parsed_sentence["CHC"]

                    Warming = data["Warming"] # 2 表示正常
                    Status = int(data["Status"] )# 系统状态（低半字 节）： 0 初始化 1 卫导模式 2 组合导航模式 3 纯惯导模式 卫星状态
                                            # （  高半字 节）： （0：不定位不定向； 1：单点定位定向； 2：伪距差分定位定 向；3：组合推算； 
                                            #                   4：RTK 稳定解定位定 向；5：RTK 浮点解定 位定向；6：单点定位 不定向；7：伪距差分 定位不定向；8：RTK 稳定解定位不定向； 9-RTK 浮点解定位不 定向）
                    if Warming == 2:
                        self.driver.current_fix.header.stamp = current_time
                        self.driver.current_fix.header.frame_id = frame_id
                        self.driver.current_fix.position_covariance_type = (
                            self.driver.getParameter_DoraNavSatFix.COVARIANCE_TYPE_APPROXIMATED()
                        )
                       
                        # self.driver.current_fix.status.status  
                        if (Status & 0x0f)> 3:
                            self.driver.valid_fix = True
                        else:
                            self.driver.valid_fix = False

                        self.driver.current_fix.latitude = data["Latitude"]
                        self.driver.current_fix.longitude = data["Longitude"]
                        self.driver.current_fix.altitude = data["Altitude"]
                        # 发布解析得到的DoraNavSatFix消息类型
                        #nmea_Publish_instance = self.driver.current_fix
                        #parsed_nmea_sentence = pickle.dumps(nmea_Publish_instance)
                        # send_output(
                        #     "DoraNavSatFix",
                        #     parsed_nmea_sentence,
                        #     dora_input["metadata"],
                        # )
                        # 发布JSON-DoraNavSatFix消息
                        self.cnt1 =self.cnt1+1
                        sentence_dict = {
                            "seq": self.cnt1,
                            "frame_id": self.driver.current_fix.header.frame_id,
                            "sec": self.driver.current_fix.header.stamp.sec,
                            "nanosec": self.driver.current_fix.header.stamp.nanosec,
                            "latitude": self.driver.current_fix.latitude,
                            "longitude": self.driver.current_fix.longitude,
                            "altitude": self.driver.current_fix.altitude,
                            "position_covariance": self.driver.current_fix.position_covariance,
                            "position_covariance_type": self.driver.current_fix.position_covariance_type,
                            "status": self.driver.current_fix.status.status,
                            "service": self.driver.current_fix.status.service
                        }
                        
                        json_string = json.dumps(sentence_dict, indent=4)  # 使用indent参数设置缩进宽度为4
                        #print(json_string)
                        json_bytes = json_string.encode('utf-8')
                        send_output("DoraNavSatFix",json_bytes,dora_input["metadata"],)

                        now = datetime.now()# 获取当前时间
                        timestamp_ms = round(now.timestamp() * 1000)# 转换为毫秒格式
                        print("counter: ",self.cnt1 ,"NavSatFix  pub time: ", timestamp_ms)# 打印时间戳（毫秒）
                    else:
                        print("GPS error, Warming == ",Warming)


                    # 固定解时候提供速度 朝向
                    if self.driver.valid_fix and Warming == 2:
                        self.driver.current_vel.header.stamp = current_time
                        self.driver.current_vel.header.frame_id = frame_id
                        self.driver.current_vel.x_linear = data["Ve"]
                        self.driver.current_vel.y_linear = data["Vn"]
                        #  发布解析得到的DoraTwistStamped消息类型
                        nmea_Publish_instance = self.driver.current_vel
                        parsed_nmea_sentence = pickle.dumps(nmea_Publish_instance)
                        #print(parsed_nmea_sentence)
                        send_output(
                            "DoraTwistStamped",
                            parsed_nmea_sentence,
                            dora_input["metadata"],
                        )

                        print("roll-pitch-yaw:  ",data["Roll"],"  ",data["Pitch"],"  ",data["Heading"])
                        q = quaternion_from_euler(math.radians(np.float32(data["Roll"])), math.radians(np.float32(data["Pitch"])), math.radians(np.float32(data["Heading"])))
                        
                        self.driver.current_heading.header.stamp = current_time
                        self.driver.current_heading.header.frame_id = frame_id
                        self.driver.current_heading.quaternion.x = q[1]
                        self.driver.current_heading.quaternion.y = q[2]
                        self.driver.current_heading.quaternion.z = q[3]
                        self.driver.current_heading.quaternion.w = q[0]
                        # 发布解析得到的DoraQuaternionStamped消息类型
                        # nmea_Publish_instance = self.driver.current_heading
                        # parsed_nmea_sentence = pickle.dumps(nmea_Publish_instance)
                        # # print(parsed_nmea_sentence)
                        # send_output(
                        #     "DoraQuaternionStamped",
                        #     parsed_nmea_sentence,
                        #     dora_input["metadata"],
                        # )
                        self.cnt2 = self.cnt2+1
                        # 以json字符从的形似发布
                        sentence_dict = {
                            "seq": self.cnt2,
                            "frame_id": self.driver.current_heading.header.frame_id,
                            "sec": self.driver.current_heading.header.stamp.sec,
                            "nanosec": self.driver.current_heading.header.stamp.nanosec,
                            "x": self.driver.current_heading.quaternion.x,
                            "y": self.driver.current_heading.quaternion.y,
                            "z": self.driver.current_heading.quaternion.z,
                            "w": self.driver.current_heading.quaternion.w,
                        }
                        json_string = json.dumps(sentence_dict, indent=4)  # 使用indent参数设置缩进宽度为4
                        #print(json_string)
                        json_bytes = json_string.encode('utf-8')
                        send_output("DoraQuaternionStamped",json_bytes,dora_input["metadata"],)
                        now = datetime.now()# 获取当前时间
                        timestamp_ms = round(now.timestamp() * 1000)# 转换为毫秒格式
                        print("counter: ",self.cnt2 ,"QuaternionStamped  pub time: ", timestamp_ms)# 打印时间戳（毫秒）

                return DoraStatus.CONTINUE
            except ValueError as e:
                print(
                    "Value error, likely due to missing fields in the NMEA message. Error was: %s. "
                    % e
                )
            return DoraStatus.CONTINUE
         

    def drop_operator(self):
        self.f.close()
