""" 
# Plot operator

Plot operator takes outputs from the graph and plot it on the camera frame.
"""
import time
from typing import Callable

import cv2
import numpy as np
import pyarrow as pa
from dora import DoraStatus
from dora_utils import (
    LABELS,
    get_extrinsic_matrix,
    get_intrinsic_matrix,
    get_projection_matrix,
    local_points_to_camera_view,
    location_to_camera_view,
)
from scipy.spatial.transform import Rotation as R

pa.array([])  # See: https://github.com/apache/arrow/issues/34994

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
DEPTH_IMAGE_WIDTH = 640
DEPTH_IMAGE_HEIGHT = 480
DEPTH_FOV = 90
SENSOR_POSITION = np.array([3, 0, 1])

VELODYNE_MATRIX = np.array([[0, 0, 1], [1, 0, 0], [0, -1, 0]])
UNREAL_MATRIX = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
INV_VELODYNE_MATRIX = np.linalg.inv(VELODYNE_MATRIX)
INTRINSIC_MATRIX = get_intrinsic_matrix(
    DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_HEIGHT, DEPTH_FOV
)

VERBOSE = True
NO_DISPLAY = False

writer = cv2.VideoWriter(
    "../output01.avi",
    cv2.VideoWriter_fourcc(*"MJPG"),
    30,
    (CAMERA_WIDTH, CAMERA_HEIGHT),
)

font = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10, 30)
fontScale = 0.6
fontColor = (255, 0, 255)
thickness = 2
lineType = 2


class Operator:
    """
    Plot inputs using cv2.imshow
    """

    def __init__(self):
        self.waypoints = []
        self.gps_waypoints = []
        self.obstacles = []
        self.raw_obstacles = []
        self.obstacles_bbox = []
        self.obstacles_id = []
        self.lanes = []
        self.global_lanes = []
        self.drivable_area = []
        self.last_timestamp = time.time()
        self.position = []
        self.last_position = []
        self.camera_frame = []
        self.traffic_sign_bbox = []
        self.point_cloud = np.array([])
        self.control = []
        self.last_time = time.time()
        self.current_speed = []

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
        if "infra2_image" == dora_input["id"]:

            self.camera_frame = np.frombuffer(dora_input['value'].to_numpy(), dtype=np.uint8).reshape((480, 640))



        if "infra2_image" != dora_input["id"] or isinstance(self.camera_frame, list):
            return DoraStatus.CONTINUE

        resized_image = (self.camera_frame.astype(np.uint16) * (65535 // 255))

        writer.write(resized_image)
        resized_image = cv2.resize(resized_image, (480, 640))
        if not NO_DISPLAY:
            cv2.imshow("infra2_image", resized_image)
            cv2.waitKey(1)
        self.last_time = time.time()
        ## send_output("plot_status", b"")
        return DoraStatus.CONTINUE
