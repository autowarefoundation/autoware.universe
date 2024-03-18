from typing import Callable
from dora import DoraStatus
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
