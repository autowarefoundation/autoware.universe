#!/usr/bin/python3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import sys
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
from rosbag2_py import ConverterOptions
import matplotlib.pyplot as plt
import numpy as np


class FixChecker():
    def __init__(self, bag_file):
        self.reader = SequentialReader()
        bag_storage_otions = StorageOptions(uri=bag_file, storage_id="sqlite3")
        bag_converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        self.reader.open(bag_storage_otions, bag_converter_options)

        self.type_map = {}
        for topic_type in self.reader.get_all_topics_and_types():
            self.type_map[topic_type.name] = topic_type.type

    def __checkNavPvt(self):
        vels = []
        first_stamp = None
        while self.reader.has_next():
            (topic, data, stamp) = self.reader.read_next()
            if topic != '/sensing/gnss/ublox/navpvt':
                continue

            if first_stamp is None:
                first_stamp = stamp

            msg_type = get_message(self.type_map[topic])
            msg = deserialize_message(data, msg_type)

            dt = (stamp-first_stamp)*1e-9
            if dt < 300 or dt > 520:
                continue
            vels.append([dt, msg.vel_n*1e-3, msg.vel_e*1e-3, msg.vel_d*1e-3])

        vels = np.array(vels)
        plt.grid()
        plt.plot(vels[:, 0], vels[:, 1], color='r')
        plt.plot(vels[:, 0], vels[:, 2], color='g')
        plt.plot(vels[:, 0], vels[:, 3], color='b')
        plt.show()

    def __call__(self):
        vels = []
        first_stamp = None
        while self.reader.has_next():
            (topic, data, stamp) = self.reader.read_next()
            if topic != '/sensing/gnss/ublox/navpvt':
                continue

            if first_stamp is None:
                first_stamp = stamp

            msg_type = get_message(self.type_map[topic])
            msg = deserialize_message(data, msg_type)

            dt = (stamp-first_stamp)*1e-9
            if dt<300 or dt > 600:
                continue
            vels.append([dt, msg.vel_n, msg.vel_e,msg.vel_d])

        vels = np.array(vels)
        print(vels)
        plt.grid()
        plt.plot(vels[:, 0], vels[:, 1], color='r')
        plt.plot(vels[:, 0], vels[:, 2], color='g')
        plt.plot(vels[:, 0], vels[:, 3], color='b')
        plt.show()


def main():
    if len(sys.argv) == 1:
        return
    bag_file = sys.argv[1]

    checker = FixChecker(bag_file)
    checker()


if __name__ == "__main__":
    main()
