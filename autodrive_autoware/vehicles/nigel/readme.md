<p align="center">
<img src="https://github.com/Tinker-Twins/AutoDRIVE-Autoware/blob/main/autodrive_autoware/media/AutoDRIVE-Logo.png" alt="AutoDRIVE" width="478"/> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <img src="https://github.com/Tinker-Twins/AutoDRIVE-Autoware/blob/main/autodrive_autoware/media/Autoware-Logo.png" alt="Autoware" width="478"/>
</p>

## Digital Twin Simulation Demo - AutoDRIVE Simulator

1. Launch AutoDRIVE Simulator for Nigel and establish Autoware API bridge connection in single or distributed computing setting as applicable.
2. Install the `rviz_imu_plugin` package (if not already accomplished) using Ubuntu's [Advanced Packaging Tool (APT)](https://en.wikipedia.org/wiki/APT_(software)).
    ```bash
    user@host-pc:~$ sudo apt install ros-$ROS_DISTRO-rviz-imu-plugin
    ```
3. Install the `slam_toolbox` package (if not already accomplished) using Ubuntu's [Advanced Packaging Tool (APT)](https://en.wikipedia.org/wiki/APT_(software)).
    ```bash
    user@host-pc:~$ sudo apt install ros-$ROS_DISTRO-slam-toolbox
    ```
4. Map the environment (if not already accomplished) by driving (teleoperating) the vehicle around the environment.
    ```bash
    user@host-pc:~$ ros2 launch autodrive_nigel simulator_slam.launch.py
    user@host-pc:~$ ros2 run autodrive_nigel teleop_keyboard
    ```

| <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-Nigel-TinyTown-Simulator/Map-Nigel.gif" width="478"> | <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-Nigel-TinyTown-Simulator/Map-Autoware.gif" width="478"> |
| :-----------------: | :-----------------: |

5. Build and install the [RangeLibc Python wrapper](https://github.com/Tinker-Twins/AutoDRIVE-Autoware/tree/main/autodrive_autoware/perception/pf_localization/range_libc/pywrapper) (if not already accomplished).
    ```bash
    user@host-pc:~$ sudo apt update
    user@host-pc:~$ rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    user@host-pc:~$ cd ~/Autoware_WS/autoware_local/src/universe/autoware.universe/autodrive_autoware/perception/pf_localization/range_libc/pywrapper
    user@host-pc:~$ sudo chmod +x *.sh
    user@host-pc:~$ ./compile_with_cuda.sh
    ```

6. Record waypoints by driving (teleoperating) the vehicle around the environment while localizing against the map.
    ```bash
    user@host-pc:~$ ros2 launch autodrive_nigel simulator_record.launch.py
    user@host-pc:~$ ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/home/<username>/path"}" --feedback
    user@host-pc:~$ ros2 run autodrive_nigel teleop_keyboard
    ```
    > **Note:** Replace `<username>` with your actual username. Feel free to use a different path to save the trajectory file.

| <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-Nigel-TinyTown-Simulator/Record-Nigel.gif" width="478"> | <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-Nigel-TinyTown-Simulator/Record-Autoware.gif" width="478"> |
| :-----------------: | :-----------------: |

7. Engage the vehicle in autonomous mode to track the reference trajectory in real-time.
    ```bash
    user@host-pc:~$ ros2 launch autodrive_nigel simulator_replay.launch.py
    user@host-pc:~$ ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/home/<username>/path"}" --feedback
    ```
    > **Note:** Replace `<username>` with your actual username. Be sure to use the correct path to load the trajectory file.

| <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-Nigel-TinyTown-Simulator/Replay-Nigel.gif" width="478"> | <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-Nigel-TinyTown-Simulator/Replay-Autoware.gif" width="478"> |
| :-----------------: | :-----------------: |

## Physical Testbed Demo - Nigel @ ARMLab CU-ICAR

> **Note:** This demo was set up with [`rosserial`](https://wiki.ros.org/rosserial) and [`rosserial_arduino`](https://wiki.ros.org/rosserial_arduino) (tested with [rosserial_arduino_library 0.7.9](https://www.arduino.cc/reference/en/libraries/rosserial-arduino-library)) to additionally demonstrate the workflow of deploying Autoware stack by interfacing ROS 2 Galactic with ROS 1 Noetic using [`ros1_bridge`](https://github.com/ros2/ros1_bridge) for ROS 2.

1. Start up the vehicle and establish a remote connection with the vehicle's single-board computer (SBC) using [SSH](https://en.wikipedia.org/wiki/Secure_Shell) or [VNC](https://en.wikipedia.org/wiki/X11vnc) ([HDMI](https://en.wikipedia.org/wiki/HDMI) or [DP](https://en.wikipedia.org/wiki/DisplayPort) emulator a.k.a. dummy plug may be required) as applicable.
2. Install the `rviz_imu_plugin` package (if not already accomplished) using Ubuntu's [Advanced Packaging Tool (APT)](https://en.wikipedia.org/wiki/APT_(software)).
    ```bash
    user@host-pc:~$ sudo apt install ros-$ROS_DISTRO-rviz-imu-plugin
    ```
3. Install the `slam_toolbox` package (if not already accomplished) using Ubuntu's [Advanced Packaging Tool (APT)](https://en.wikipedia.org/wiki/APT_(software)).
    ```bash
    user@vehicle-sbc:~$ sudo apt install ros-$ROS_DISTRO-slam-toolbox
    ```
4. Map the environment (if not already accomplished) by driving (teleoperating) the vehicle around the environment.
    ```bash
    user@vehicle-sbc:~$ ros2 launch autodrive_nigel testbed_slam.launch.py
    ```

| <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-Nigel-ARMLab-Testbed/Map-Nigel.gif" width="478"> | <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-Nigel-ARMLab-Testbed/Map-Autoware.gif" width="478"> |
| :-----------------: | :-----------------: |

5. Build and install the [RangeLibc Python wrapper](https://github.com/Tinker-Twins/AutoDRIVE-Autoware/tree/main/autodrive_autoware/perception/pf_localization/range_libc/pywrapper) (if not already accomplished).
    ```bash
    user@vehicle-sbc:~$ sudo apt update
    user@vehicle-sbc:~$ rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    user@vehicle-sbc:~$ cd ~/Autoware_WS/autoware_local/src/universe/autoware.universe/autodrive_autoware/perception/pf_localization/range_libc/pywrapper
    user@vehicle-sbc:~$ sudo chmod +x *.sh
    user@vehicle-sbc:~$ ./compile_with_cuda.sh
    ```

6. Record waypoints by driving (teleoperating) the vehicle around the environment while localizing against the map.
    ```bash
    user@vehicle-sbc:~$ ros2 launch autodrive_nigel testbed_record.launch.py
    user@vehicle-sbc:~$ ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/home/<username>/path"}" --feedback
    ```
    > **Note:** Replace `<username>` with your actual username. Feel free to use a different path to save the trajectory file.

| <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-Nigel-ARMLab-Testbed/Record-Nigel.gif" width="478"> | <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-Nigel-ARMLab-Testbed/Record-Autoware.gif" width="478"> |
| :-----------------: | :-----------------: |

7. Engage the vehicle in autonomous mode to track the reference trajectory in real-time.
    ```bash
    user@vehicle-sbc:~$ ros2 launch autodrive_nigel testbed_replay.launch.py
    user@vehicle-sbc:~$ ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/home/<username>/path"}" --feedback
    ```
    > **Note:** Replace `<username>` with your actual username. Be sure to use the correct path to load the trajectory file.

| <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-Nigel-ARMLab-Testbed/Replay-Nigel.gif" width="478"> | <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-Nigel-ARMLab-Testbed/Replay-Autoware.gif" width="478"> |
| :-----------------: | :-----------------: |

## Citation

We encourage you to read and cite the following paper if you use any part of this work for your research:

#### [AutoDRIVE: A Comprehensive, Flexible and Integrated Digital Twin Ecosystem for Enhancing Autonomous Driving Research and Education](https://arxiv.org/abs/2212.05241)
```bibtex
@article{AutoDRIVE-Ecosystem-2023,
author = {Samak, Tanmay and Samak, Chinmay and Kandhasamy, Sivanathan and Krovi, Venkat and Xie, Ming},
title = {AutoDRIVE: A Comprehensive, Flexible and Integrated Digital Twin Ecosystem for Autonomous Driving Research &amp; Education},
journal = {Robotics},
volume = {12},
year = {2023},
number = {3},
article-number = {77},
url = {https://www.mdpi.com/2218-6581/12/3/77},
issn = {2218-6581},
doi = {10.3390/robotics12030077}
}
```
This work has been published in **MDPI Robotics.** The open-access publication can be found on [MDPI](https://doi.org/10.3390/robotics12030077).
