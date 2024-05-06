#!/usr/bin/env bash

# give docker permission to use X
sudo xhost +si:localuser:root

# run container with privilege mode, host network, display, and mount workspace on host
sudo docker restart $1
sudo docker attach $1