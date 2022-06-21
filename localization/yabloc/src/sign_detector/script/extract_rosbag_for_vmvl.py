#!/usr/bin/python3
import glob
import subprocess
import shutil


def main():
    files = glob.glob('./*.db3')

    # fmt: off
    topics = [
        '/sensing/camera/traffic_light/camera_info', 
        '/sensing/camera/traffic_light/image_raw/compressed',
        '/sensing/gnss/ublox/fix_velocity', 
        '/sensing/gnss/ublox/nav_sat_fix', 
        '/sensing/gnss/ublox/navpvt',
        '/sensing/imu/tamagawa/imu_raw',
        '/vehicle/status/twist',
        ]
    # fmt: on

    # filt necessary topics
    dst_files = []
    for index, f in enumerate(files):
        print(index, f)
        tmp_dst = 'tmp_'+str(index)+'.db3'
        dst_files.append(tmp_dst)
        command = ['ros2', 'bag', 'filter', f, '-o', tmp_dst, '-i']
        for t in topics:
            command.append(t)
        subprocess.run(command)

    # merge tmp files
    command = ['ros2', 'bag', 'merge',  '-o', 'minimum.db3']
    for f in dst_files:
        command.append(f)
    subprocess.run(command)

    # remove tmp files
    for f in dst_files:
        shutil.rmtree(f)


if __name__ == '__main__':
    main()
