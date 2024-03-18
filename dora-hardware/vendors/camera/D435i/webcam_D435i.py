from typing import Callable
from dora import DoraStatus
import os
import cv2
import numpy as np
import pyarrow as pa
import pyrealsense2 as rs
import threading
pa.array([])
OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 480
# DEVICE_INDEX = os.environ.get('DEVICE_INDEX', '0')


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 15)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        self.config.enable_stream(rs.stream.infrared, 1)
        self.config.enable_stream(rs.stream.infrared, 2)
        self.config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
        self.config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        self.lock = threading.Lock()
        self.pipeline.start(self.config)

    def on_event(
        self,
        dora_event: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE

    def normalize_depth_image(depth_image):
        # 检查分母是否为0，如果为0则避免除法运算
        min_depth = np.min(depth_image)
        max_depth = np.max(depth_image)
        if min_depth == max_depth:
            # 分母为0，可能出现了特殊情况，可以设置一个默认值或者直接返回原始深度图像
            # 这里我们设置一个默认值为0的情况，您可以根据实际情况进行修改
            normalized_depth_image = np.zeros_like(depth_image, dtype=np.uint8)
        else:
            # 进行归一化处理和数据类型转换
            normalized_depth_image = ((depth_image - min_depth) / (max_depth - min_depth) * 255).astype(np.uint8)

        return normalized_depth_image

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):
        frames = self.pipeline.wait_for_frames()
        # 使用线程锁来确保只有一个线程能访问数据流
        # 获取各种数据流
        color_frames = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        infrared1_frame = frames.get_infrared_frame(1)
        infrared2_frame = frames.get_infrared_frame(2)
        imu_frame = frames.first_or_default(rs.stream.accel)
        depth_to_color_extrinsics = self.pipeline.get_active_profile().get_stream(
            rs.stream.depth).as_video_stream_profile().get_extrinsics_to(
            self.pipeline.get_active_profile().get_stream(rs.stream.color))

        # 将其转化为np数组
        color_images = np.asanyarray(color_frames.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        infra1_images = np.asanyarray(infrared1_frame.get_data())
        infra2_images = np.asanyarray(infrared2_frame.get_data())

        color_images = cv2.cvtColor(color_images, cv2.COLOR_BGR2RGB)
        color_images = cv2.resize(color_images, (640, 480))

        depth_image = (
                (depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image)) * 255).astype(
            np.uint8)
        infra1_images = ((infra1_images - np.min(infra1_images)) / (
                np.max(infra1_images) - np.min(infra1_images)) * 255).astype(
            np.uint8)

        infra2_images = ((infra2_images - np.min(infra2_images)) / (
                np.max(infra2_images) - np.min(infra2_images)) * 255).astype(
            np.uint8)

        imu_data = imu_frame.as_motion_frame().get_motion_data()
        imu_data_list = [imu_data.x, imu_data.y, imu_data.z]
        imu_data_np = np.array(imu_data_list, dtype=np.uint8)

        extrinsics_array = np.array(depth_to_color_extrinsics.rotation).reshape((3, 3))

        # 将数据转换为 `UInt8` 类型的 numpy 数组
        extrinsics_np = extrinsics_array.astype(np.uint8)

        # 发送彩色数据流
        send_output(
            "color_image",
            pa.array(color_images.ravel().view(np.uint8)),
            dora_input["metadata"]
        )

        # 发布深度数据流
        send_output(
            "depth_image",
            pa.array(depth_image.ravel().view(np.uint8)),
            dora_input["metadata"],
        )

        # 发布左红外数据流
        send_output(
            "infra1_image",
            pa.array(infra1_images.ravel().view(np.uint8)),
            dora_input["metadata"],
        )

        # 发布右红外数据流
        send_output(
            "infra2_image",
            pa.array(infra2_images.ravel().view(np.uint8)),
            dora_input["metadata"],
        )

        # 发布imu数据流
        send_output(
            "imu_data",
            pa.array(imu_data_np),
            dora_input["metadata"],
        )

        # 发布相机外参数据流
        send_output(
            "depth_to_color_extrinsics",
            pa.array(extrinsics_np.ravel().view(np.uint8)),
            dora_input["metadata"],
        )


        return DoraStatus.CONTINUE

    def drop_operator(self):
        self.pipeline.stop()
