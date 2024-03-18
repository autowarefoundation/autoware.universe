#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dora import Node
import pickle

node = Node()

event = node.next()
if event["type"] == "INPUT":
    print(
        f"""Node received:
    id: {event["id"]},
    value: {event["data"]},
    metadata: {event["metadata"]}""")
    # 你可以将其转换回原来的 Python 对象
    reconstructed_imu_dict = pickle.loads(event["data"])
    # 打印还原后的属性字典
    print(reconstructed_imu_dict)

