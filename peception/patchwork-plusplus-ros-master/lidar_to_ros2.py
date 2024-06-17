from typing import Callable, Optional
import pyarrow as pa
from dora import DoraStatus
import dora
import numpy as np
import time
class Operator:
    def __init__(self) -> None:
        self.ros2_context = dora.experimental.ros2_bridge.Ros2Context()
        # create ros2 node
        self.ros2_node = self.ros2_context.new_node(
            "lidar2ros",
            "/ros2_bridge",
            dora.experimental.ros2_bridge.Ros2NodeOptions(rosout=True)
        )
        # create ros2 qos
        self.topic_qos = dora.experimental.ros2_bridge.Ros2QosPolicies(
            reliable=True, max_blocking_time=0.1
        )
        # create ros2 topic
        self.lidar_data_topic = self.ros2_node.create_topic(
            "/ros2_bridge/lidar_data",
            "sensor_msgs::PointCloud2",
            self.topic_qos
        )
        # create ros2 publisher
        self.lidar_data_publisher = self.ros2_node.create_publisher(self.lidar_data_topic)
    
    def on_event(
            self,
            dora_event,
            send_output,
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":

            # print(dora_event["value"],"\n")
            # print("msg->seq:", dora_event["value"][0:4])
            # print("empty:", dora_event["value"][4:8])
            # print("timestamp:", dora_event["value"][8:16])
            # print("first point data x:", u82f.uint8_2_float(np.array(dora_event["value"][16:20])))
            # print("first point data y:", u82f.uint8_2_float(np.array(dora_event["value"][20:24])))
            # print("first point data z:", u82f.uint8_2_float(np.array(dora_event["value"][24:28])))
            print("dora event value over, it len:",len(dora_event["value"]), '\n')

            # pointdata_raw = np.array(dora_event["value"][16:]).reshape(-1, 16)
            pointdata_raw = np.array(dora_event["value"][16:]).reshape(-1, 16)
            print(pointdata_raw.shape)
            print(pointdata_raw.size)
            pointdata_raw = pointdata_raw[:, 0:16]
            print(pointdata_raw.shape)
            print(pointdata_raw.size)
            points = pointdata_raw
            # for i in range(0, len(pointdata_raw)):
            #     point_xyz = [u82f.uint8_2_float(pointdata_raw[i][j * 4 : j * 4 + 4]) for j in range(0, 3)]
            #     # point_xyz = u82f.uint8_2_float(po intdata_raw[i])
            #     points.append(point_xyz)

            # points = u82f.uint8_2_float(pointdata_raw)
            # points = np.array(points).reshape(-1, 4)
            # points.append(point_xyz)
            # points = pointdata_raw
            print("==========================================================\n")
            '''
            # create a virtual PointCloud2 message for testing the programm
            points=np.array([[10.0, 10.0, 10.0],[5.0, 5.0, 5.0],[3.0, 3.0, 3.0]])
            # print(np.asarray(points, np.float32).tostring())
            '''
            current_time = time.time()
            sec = (current_time)
            nanosec = ((current_time - sec) * 1e9)
            lidar_data_dict = {
                "header": {
                    "stamp": {
                        "sec": np.int32(sec),
                        "nanosec": np.uint32(nanosec),
                    },
                    "frame_id": "rslidar",
                },
                "height": np.uint32(1),
                "width": np.uint32(len(points)),
                "fields":[{"name": "x", "offset": np.uint32(0), "datatype": np.uint8(7), "count": np.uint32(1)}, 
                          {"name": "y", "offset": np.uint32(4), "datatype": np.uint8(7), "count": np.uint32(1)},
                          {"name": "z", "offset": np.uint32(8), "datatype": np.uint8(7), "count": np.uint32(1)},
                          {"name": "i", "offset": np.uint32(12), "datatype": np.uint8(7), "count": np.uint32(1)},],
                "is_bigendian": False,
                "point_step": np.uint32(16),
                "row_step": np.uint32(len(points)),
                "data": points.ravel().view(np.uint8),#np.asarray(points, np.float32).ravel().view(np.uint8),
                "is_dense": False,
            }
            # print(pa.array([lidar_data_dict]))
            self.lidar_data_publisher.publish(pa.array([lidar_data_dict]))
        return DoraStatus.CONTINUE
