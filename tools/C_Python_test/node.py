#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pyarrow as pa
import numpy as np
from dora import Node, DoraStatus

class Operator:
    def __init__(self, expected_messages):
        self.expected_messages = expected_messages
        self.received_messages = 0
        self.received_ids = set()

    def on_event(self, dora_event, send_output):
        if dora_event["type"] == "INPUT":
            self.received_messages += 1
            data = dora_event["value"].to_numpy()
            data_id = dora_event["id"]  # 获取消息ID
            data_len = len(data)

            print(f"Input Data length: {data_len}")
            # 显示消息计数
            print(f"Received messages: {self.received_messages}/{self.expected_messages}")
            print("=======================")

            # 记录接收到的消息ID
            self.received_ids.add(data_id)

        elif dora_event["type"] == "STOP":
            print("[Python node] received stop event")
            print(f"Total messages received: {self.received_messages}/{self.expected_messages}")

            # 计算缺失的消息
            expected_ids = set(map(str, range(1, self.expected_messages + 1)))  # 假设消息ID是从 "1" 到 "expected_messages" 的字符串
            missing_ids = expected_ids - self.received_ids
            missing_count = len(missing_ids)

            if missing_count > 0:
                print(f"Missing message IDs: {sorted(missing_ids)}")
                print(f"Total missing messages: {missing_count}")
            else:
                print("No messages are missing.")

            return DoraStatus.STOP

        return DoraStatus.CONTINUE

if __name__ == "__main__":
    expected_messages = 10000  # 设置预期的消息总数
    operator = Operator(expected_messages)
    node = Node()
    for event in node:
        status = operator.on_event(event, node.send_output)
        if status == DoraStatus.STOP:
            break

