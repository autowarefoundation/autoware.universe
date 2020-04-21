#!/bin/sh

usage_exit() {
        echo "Usage: rosrun autoware_rosbag_recorder record.sh [-o filename]" 1>&2
        exit 1
}

while getopts o:h OPT
do
    case $OPT in
        "o") echo "record as $OPTARG";;
        "h") usage_exit;;
        "") usage_exit;;
        \?) usage_exit;;
    esac
done

if [ -n "$OPTARG" ]; then
  rosbag record -e "(.*)/velodyne_packets|/as/(.*)|/pacmod/(.*)|/vehicle/(.*)|/sensing/imu/(.*)|/sensing/gnss/(.*)|/sensing/camera/(.*)/camera_info|/perception/object_recognition/detection/rois(.)|/perception/object_recognition/objects" -o $OPTARG;
else 
  usage_exit;
fi
