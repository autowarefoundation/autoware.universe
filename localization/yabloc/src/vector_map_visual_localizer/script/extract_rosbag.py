#!/usr/bin/python3
import glob
import subprocess
import shutil
import re
import argparse

# fmt: off
TOPICS = [
    '/sensing/gnss/ublox/fix_velocity', 
    '/sensing/gnss/ublox/nav_sat_fix', 
    '/sensing/gnss/ublox/navpvt',
    '/sensing/imu/tamagawa/imu_raw',
    '/vehicle/status/actuation_status',
    '/vehicle/status/shift',
    '/vehicle/status/steering',
    '/vehicle/status/control_mode',
    '/vehicle/status/twist',
    '/vehicle/status/velocity_status'
    ]
LIDAR_TOPICS = [
    '/sensing/lidar/top/velodyne_packets',
]
CAMERA_TOPICS = [
    '/sensing/camera/traffic_light/camera_info',
    '/sensing/camera/traffic_light/image_raw/compressed',
]
# fmt: on


def print_highlight(text):
    print('\033[1m\033[93m', text, '\033[0m')


def extractIndex(name):
    pattern = '.*?(\d+).db3'
    result = re.match(pattern, name)
    return int(result.group(1))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--lidar',  action='store_true', help='include lidar topics')
    parser.add_argument('-c', '--camera', action='store_true', help='include camera topics')
    args = parser.parse_args()

    if args.camera:
        TOPICS.extend(CAMERA_TOPICS)
    if args.lidar:
        TOPICS.extend(LIDAR_TOPICS)

    # Sort files
    files = glob.glob('./*.db3')
    indexed_files = []
    for f in files:
        index = extractIndex(f)
        indexed_files.append((index, f))
    indexed_files.sort(key=lambda x: x[0])

    # Filter necessary topics
    print_highlight('Process '+str(len(indexed_files))+' files.')
    dst_files = []
    for index, f in indexed_files:
        print_highlight(str(index)+' '+f)
        tmp_dst = 'tmp'+str(index)
        dst_files.append(tmp_dst)
        command = ['ros2', 'bag', 'filter', f, '-o', tmp_dst, '-i']
        for t in TOPICS:
            command.append(t)
        subprocess.run(command)

    # Merge tmp files
    print_highlight('Merge filtered files.')
    command = ['ros2', 'bag', 'merge',  '-o', 'filtered']
    for f in dst_files:
        command.append(f)
    subprocess.run(command)

    # Remove tmp files
    print_highlight('Remove temporary files.')
    for f in dst_files:
        shutil.rmtree(f)


if __name__ == '__main__':
    main()
