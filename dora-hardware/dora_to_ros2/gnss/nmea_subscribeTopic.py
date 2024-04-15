from typing import Callable
from dora import DoraStatus
from DoraNmeaDriver_utils import (
    DoraSentence,
    DoraNavSatFix,
    DoraQuaternionStamped,
    DoraTwistStamped,
)
import pickle
import json
from json import *
class Operator:
    """
    反序列化后，输出各类消息内容
    """

    def __init__(self):
        self.receDoraSentence = DoraSentence()
        self.receDoraNavSatFix = DoraNavSatFix()
        self.receDoraQuaternionStamped = DoraQuaternionStamped()
        self.receDoraTwistStamped = DoraTwistStamped()

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


            # 发布JSON-DoraNavSatFix消息
            sentence_dict = {
                "frame_id": self.receDoraNavSatFix.header.frame_id,
                "sec": self.receDoraNavSatFix.header.stamp.sec,
                "nanosec": self.receDoraNavSatFix.header.stamp.nanosec,
                "latitude": self.receDoraNavSatFix.latitude,
                "longitude": self.receDoraNavSatFix.longitude,
                "altitude": self.receDoraNavSatFix.altitude,
                "position_covariance": self.receDoraNavSatFix.position_covariance,
                "position_covariance_type": self.receDoraNavSatFix.position_covariance_type,
                "status": self.receDoraNavSatFix.status.status,
                "service": self.receDoraNavSatFix.status.service
            }
            json_string = json.dumps(sentence_dict, indent=4)  # 使用indent参数设置缩进宽度为4
            print(json_string)
            json_bytes = json_string.encode('utf-8')
            send_output("DoraNavSatFix",json_bytes,dora_input["metadata"],)


            # 发布DoraNavSatFix消息
            # print(self.receDoraNavSatFix)
            # parsed_nmea_sentence = pickle.dumps(self.receDoraNavSatFix)
            # send_output("DoraNavSatFix",parsed_nmea_sentence,dora_input["metadata"],)


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

        return DoraStatus.CONTINUE
