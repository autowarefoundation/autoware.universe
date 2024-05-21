#! /bin/zsh

sudo busybox devmem 0x0c303000 32 0x0000c400
sudo busybox devmem 0x0c303008 32 0x0000c458
sudo busybox devmem 0x0c303010 32 0x0000c400
sudo busybox devmem 0x0c303018 32 0x0000c458

sudo busybox devmem 0x0c303000
sudo busybox devmem 0x0c303008
sudo busybox devmem 0x0c303010
sudo busybox devmem 0x0c303018

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo modprobe can_dev

sudo ip link set can0 type can bitrate 500000
sudo ip link set can1 type can bitrate 500000

sudo ip link set up can0
sudo ip link set up can1
