#!/usr/bin/env python
# -*- coding: utf-8 -*-

# from dora import Node

# node = Node()

# event = node.next()
# if event["type"] == "INPUT":
#     print(
#         f"""Node received:
#         id: {event["id"]},
#         value: {event["data"]},
#         metadata: {event["metadata"]}"""
#     )

import dora
from typing import Callable
from dora import DoraStatus
# from sensor_msgs.msg import Imu
from dora import Node
import pickle
import pyarrow as pa
import pandas as pd


class Operator:
    """
    反序列化后，输出各类消息内容
    """

    def __init__(self):
        pass
        
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
        # print(dora_input["id"])
        if "Imu100D4" == dora_input["id"]:
            dora_input = dora_input["value"] 
            print(dora_input)
            # dora_input_bytes = bytes(dora_input.to_pylist())
            # self.receDoraSentence = pickle.loads(dora_input_bytes)
            # print(self.receDoraSentence)
            # Create a ROS2 Context
            ros2_context = dora.experimental.ros2_bridge.Ros2Context()
            ros2_node = ros2_context.new_node(
                "turtle_teleop",
                "/ros2_demo",
                dora.experimental.ros2_bridge.Ros2NodeOptions(rosout=True),
            )

            # Define a ROS2 QOS
            topic_qos = dora.experimental.ros2_bridge.Ros2QosPolicies(
                reliable=True, max_blocking_time=0.1
            )

            # Create a publisher to imu topic
            turtle_twist_topic = ros2_node.create_topic(
                "/turtle1/imu", "sensor_msgs::Imu", topic_qos
            )
            twist_writer = ros2_node.create_publisher(turtle_twist_topic)

            # Publish the PyArrow Array
            # imu_dict = dora_input.to_pylist()
            # twist_writer.publish(imu_dict)
            # twist_writer.publish(imu_data_array)          # 发布imu的数据
        return DoraStatus.CONTINUE



