# ros2 interface show sensor_msgs/msg/NavSatFix
# 查看ROS2消息类型
from typing import Callable, Optional
import pyarrow as pa
from dora import DoraStatus
import dora
import numpy as np
import time

from DoraNmeaDriver_utils import (
    DoraSentence,
    DoraNavSatFix,
    DoraQuaternionStamped,
    DoraTwistStamped,
)
import pickle

class Operator:
    """
    反序列化后，输出各类消息内容
    """

    def __init__(self):
        self.receDoraSentence = DoraSentence()
        self.receDoraNavSatFix = DoraNavSatFix()
        self.receDoraQuaternionStamped = DoraQuaternionStamped()
        self.receDoraTwistStamped = DoraTwistStamped()

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
            "/dora_ros2_bridge/gps",
            "sensor_msgs::NavSatFix",
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
            dora_input = dora_input["value"]
            dora_input_bytes = bytes(dora_input.to_pylist())
            self.receDoraSentence = pickle.loads(dora_input_bytes)
            print(self.receDoraSentence)

        elif "DoraNavSatFix" == dora_input["id"]:
            nmea_sentence_input = dora_input["value"]
            nmea_sentence_bytes = bytes(nmea_sentence_input.to_pylist())
            self.receDoraNavSatFix = pickle.loads(nmea_sentence_bytes)
            print(self.receDoraNavSatFix)
            print("latitude :", self.receDoraNavSatFix.latitude)

        elif "DoraQuaternionStamped" == dora_input["id"]:
            nmea_sentence_input = dora_input["value"]
            nmea_sentence_bytes = bytes(nmea_sentence_input.to_pylist())
            self.receDoraQuaternionStamped = pickle.loads(nmea_sentence_bytes)
            print(self.receDoraQuaternionStamped)

        elif "DoraTwistStamped" == dora_input["id"]:
            nmea_sentence_input = dora_input["value"]
            nmea_sentence_bytes = bytes(nmea_sentence_input.to_pylist())
            self.receDoraTwistStamped = pickle.loads(nmea_sentence_bytes)
            print(self.receDoraTwistStamped)
            
        current_time = time.time()
        sec = (current_time)
        nanosec = ((current_time - sec) * 1e9)
        gps_data_dict = {
            "header": {
                #"seq": np.uint32(1),
                "stamp": {
                    "sec": np.int32(sec),
                    "nanosec": np.uint32(nanosec),
                },
                "frame_id": "gps",
            },
            "status":{
                "status": np.int8(1),
                "service": np.uint16(1),          
            },
            "latitude": np.float64(self.receDoraNavSatFix.latitude),
            "longitude": np.float64(self.receDoraNavSatFix.longitude),
            "altitude": np.float64(self.receDoraNavSatFix.altitude),
            "position_covariance_type":  np.uint8(2)
        }
        #print(pa.array([gps_data_dict]))
        self.gps_data_publisher.publish(pa.array([gps_data_dict]))

        return DoraStatus.CONTINUE