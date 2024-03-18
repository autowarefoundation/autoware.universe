from typing import Callable
from dora import DoraStatus
import os
import cv2
import numpy as np
import pyarrow as pa

pa.array([])

OUTPUT_WIDTH = 1920
OUTPUT_HEIGHT = 1080
DEVICE_INDEX = os.environ.get("DEVICE_INDEX", "0")


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.video_capture = cv2.VideoCapture(int(DEVICE_INDEX))
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, OUTPUT_WIDTH)
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, OUTPUT_HEIGHT)

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
        ret, frame = self.video_capture.read()
        if ret:
            frame = cv2.resize(frame, (OUTPUT_WIDTH, OUTPUT_HEIGHT))
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
            send_output(
                "image",
                pa.array(frame.ravel().view(np.uint8)),
                dora_input["metadata"],
            )
        else:
            print("could not get webcam.")
        return DoraStatus.CONTINUE

    def drop_operator(self):
        self.video_capture.release()
