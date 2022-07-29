#!/usr/bin/python3
import os
import subprocess
import argparse
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions
from rosbag2_py import ConverterOptions

TMP_YAML_PATH = './tmp_qos_override.yaml'
TOPICS = [
    '/clock',
    '/sensing/gnss/ublox/fix_velocity',
    '/sensing/gnss/ublox/nav_sat_fix',
    '/sensing/gnss/ublox/navpvt',
    '/sensing/imu/tamagawa/imu_raw',
    '/sensing/lidar/left/velodyne_packets',
    '/sensing/lidar/right/velodyne_packets',
    '/sensing/lidar/rear/velodyne_packets',
    '/sensing/lidar/top/velodyne_packets',
    '/sensing/camera/traffic_light/camera_info',
    '/sensing/camera/traffic_light/image_raw/compressed',
    '/vehicle/status/control_mode',
    '/vehicle/status/gear_status',
    '/vehicle/status/steering_status',
    '/vehicle/status/velocity_status',
    # .iv
    '/vehicle/status/twist',
]

OVERRIDE_TEXT = '''
/sensing/camera/traffic_light/image_raw/compressed:
    reliability: reliable
    history: keep_all
    durability: transient_local
/sensing/camera/traffic_light/camera_info:
    reliability: reliable
    history: keep_all
    durability: transient_local
'''


def doesRosbagIncludeTopics(rosbag):
    reader = SequentialReader()
    bag_storage_otions = StorageOptions(uri=rosbag, storage_id="sqlite3")
    bag_converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader.open(bag_storage_otions, bag_converter_options)

    included = []
    for topic_type in reader.get_all_topics_and_types():
        if topic_type.name in TOPICS:
            included.append(topic_type.name)
    return included


def printCommand(command):
    idx = command.index('--topics')
    print('\033[33m\033[40m\033[1m')
    for c in command[0:idx]:
        print(c, end=' ')
    print(command[idx])
    for c in command[idx+1:]:
        print('\t', c)
    print('\033[0m')

    print('The following topics are not included in rosbag')
    for t in TOPICS:
        if not t in command[idx+1:]:
            print('\t', t)


def removeOverrideYaml(yaml_path):
    print('remove', yaml_path)
    os.remove(yaml_path)


def makeOverrideYaml(yaml_path):
    f = open(yaml_path, 'w')
    f.write(OVERRIDE_TEXT)
    f.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag', help='rosbag file to replay')
    parser.add_argument('-r', '--rate', default='1.0', help='rate at which to play back messages. Valid range > 0.0.')
    parser.add_argument('-o', '--override', action='store_true', help='qos profile overrides')

    args = parser.parse_args()

    command = ['ros2', 'bag', 'play', args.rosbag, '-r', args.rate]
    included_topics = doesRosbagIncludeTopics(args.rosbag)

    if not '/clock' in included_topics:
        command.append('--clock')
        command.append('100')

    if args.override:
        command.append('--qos-profile-overrides-path')
        command.append(TMP_YAML_PATH)

    command.append('--topics')
    for t in TOPICS:
        if t in included_topics:
            command.append(t)

    printCommand(command)
    try:
        makeOverrideYaml(TMP_YAML_PATH)
        subprocess.run(command)
    finally:
        removeOverrideYaml(TMP_YAML_PATH)


if __name__ == '__main__':
    main()
