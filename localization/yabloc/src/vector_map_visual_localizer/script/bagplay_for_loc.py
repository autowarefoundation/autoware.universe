#!/usr/bin/python3
import subprocess
import argparse
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
from rosbag2_py import ConverterOptions

TOPICS = [
    "/clock",
    "/sensing/gnss/ublox/fix_velocity",
    "/sensing/gnss/ublox/nav_sat_fix",
    "/sensing/gnss/ublox/navpvt",
    "/sensing/imu/tamagawa/imu_raw",
    "/sensing/lidar/left/velodyne_packets",
    "/sensing/lidar/right/velodyne_packets",
    "/sensing/lidar/rear/velodyne_packets",
    "/sensing/lidar/top/velodyne_packets",
    "/sensing/camera/traffic_light/camera_info",
    "/sensing/camera/traffic_light/image_raw/compressed",
    "/vehicle/status/control_mode",
    "/vehicle/status/gear_status",
    "/vehicle/status/steering_status",
    "/vehicle/status/velocity_status",
    # .iv
    "/vehicle/status/twist",
]


def doesRosbagIncludeClock(rosbag):
    reader = SequentialReader()
    bag_storage_otions = StorageOptions(uri=rosbag, storage_id="sqlite3")
    bag_converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(bag_storage_otions, bag_converter_options)

    for topic_type in reader.get_all_topics_and_types():
        if topic_type.name == '/clock':
            return True
    return False


def printCommand(command):
    idx = command.index('--topics')
    print('\033[33m\033[40m\033[1m')
    for c in command[0:idx]:
        print(c, end=' ')
    print(command[idx])
    for c in command[idx+1:]:
        print('\t', c)
    print('\033[0m')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag')
    parser.add_argument('-r', '--rate', default='1.0')

    args = parser.parse_args()

    command = ['ros2', 'bag', 'play', args.rosbag, '-r', args.rate]
    if not doesRosbagIncludeClock(args.rosbag):
        command.append('--clock')
        command.append('100')

    command.append('--topics')

    for t in TOPICS:
        command.append(t)
    printCommand(command)
    subprocess.run(command)


if __name__ == '__main__':
    main()
