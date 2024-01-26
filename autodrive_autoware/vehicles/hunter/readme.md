<p align="center">
<img src="https://github.com/Tinker-Twins/AutoDRIVE-Autoware/blob/main/autodrive_autoware/media/AutoDRIVE-Logo.png" alt="AutoDRIVE" width="390"/> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <img src="https://github.com/Tinker-Twins/AutoDRIVE-Autoware/blob/main/autodrive_autoware/media/Autoware-Logo.png" alt="Autoware" width="390"/>
</p>

## 2D Navigation Demo (Digital Twin Simulation - AutoDRIVE Simulator)

1. Launch AutoDRIVE Simulator for Hunter SE and establish Autoware API bridge connection in single or distributed computing setting as applicable.
2. Install the `slam_toolbox` package (if not already accomplished) using Ubuntu's [Advanced Packaging Tool (APT)](https://en.wikipedia.org/wiki/APT_(software)).
    ```bash
    user@host-pc:~$ sudo apt install ros-$ROS_DISTRO-slam-toolbox
    ```
3. Map the environment (if not already accomplished) by driving (teleoperating) the vehicle around the environment.
    ```bash
    user@host-pc:~$ ros2 launch autodrive_hunter simulator_slam_2d.launch.py
    user@host-pc:~$ ros2 run autodrive_hunter teleop_keyboard
    ```

| <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-HunterSE-TinyTown-Simulator/Map-HunterSE.gif" width="478"> | <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-HunterSE-TinyTown-Simulator/Map-Autoware.gif" width="478"> |
| :-----------------: | :-----------------: |

4. Build and install the [RangeLibc Python wrapper](https://github.com/Tinker-Twins/AutoDRIVE-Autoware/tree/main/autodrive_autoware/perception/pf_localization/range_libc/pywrapper) (if not already accomplished).
    ```bash
    user@host-pc:~$ sudo apt update
    user@host-pc:~$ rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    user@host-pc:~$ cd ~/Autoware_WS/autoware_local/src/universe/autoware.universe/autodrive_autoware/perception/pf_localization/range_libc/pywrapper
    user@host-pc:~$ sudo chmod +x *.sh
    user@host-pc:~$ ./compile_with_cuda.sh
    ```
5. Record waypoints by driving (teleoperating) the vehicle around the environment while localizing against the map.
    ```bash
    user@host-pc:~$ ros2 launch autodrive_hunter simulator_record_2d.launch.py
    user@host-pc:~$ ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/home/<username>/path"}" --feedback
    user@host-pc:~$ ros2 run autodrive_hunter teleop_keyboard
    ```
    > **Note:** Replace `<username>` with your actual username. Feel free to use a different path to save the trajectory file.

| <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-HunterSE-TinyTown-Simulator/Record-HunterSE.gif" width="478"> | <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-HunterSE-TinyTown-Simulator/Record-Autoware.gif" width="478"> |
| :-----------------: | :-----------------: |

6. Engage the vehicle in autonomous mode to track the reference trajectory in real-time.
    ```bash
    user@host-pc:~$ ros2 launch autodrive_hunter simulator_replay_2d.launch.py
    user@host-pc:~$ ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/home/<username>/path"}" --feedback
    ```
    > **Note:** Replace `<username>` with your actual username. Be sure to use the correct path to load the trajectory file.

| <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-HunterSE-TinyTown-Simulator/Replay-HunterSE.gif" width="478"> | <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-HunterSE-TinyTown-Simulator/Replay-Autoware.gif" width="478"> |
| :-----------------: | :-----------------: |

## 3D Navigation Demo (Digital Twin Simulation - AutoDRIVE Simulator)

1. Launch AutoDRIVE Simulator for Hunter SE and establish Autoware API bridge connection in single or distributed computing setting as applicable.
2. Map the environment (if not already accomplished) by driving (teleoperating) the vehicle around the environment.
    -  Use the built-in 3D PCD mapping functionality of AutoDRIVE Simulator.
    -  Use standard ROS 2 3D SLAM packages to save a PCD map.

| <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-HunterSE-Forest-Simulator/Map-HunterSE.png" width="478"> | <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-HunterSE-Forest-Simulator/Map-Autoware.png" width="478"> |
| :-----------------: | :-----------------: |

3. Build and install the [RangeLibc Python wrapper](https://github.com/Tinker-Twins/AutoDRIVE-Autoware/tree/main/autodrive_autoware/perception/pf_localization/range_libc/pywrapper) (if not already accomplished).
    ```bash
    user@host-pc:~$ sudo apt update
    user@host-pc:~$ rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    user@host-pc:~$ cd ~/Autoware_WS/autoware_local/src/universe/autoware.universe/autodrive_autoware/perception/pf_localization/range_libc/pywrapper
    user@host-pc:~$ sudo chmod +x *.sh
    user@host-pc:~$ ./compile_with_cuda.sh
    ```
4. Record waypoints by driving (teleoperating) the vehicle around the environment while localizing against the map.
    ```bash
    user@host-pc:~$ ros2 launch autodrive_hunter simulator_record_3d.launch.py
    user@host-pc:~$ ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/home/<username>/path"}" --feedback
    user@host-pc:~$ ros2 run autodrive_hunter teleop_keyboard
    ```
    > **Note:** Replace `<username>` with your actual username. Feel free to use a different path to save the trajectory file.

| <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-HunterSE-Forest-Simulator/Record-HunterSE.gif" width="478"> | <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-HunterSE-Forest-Simulator/Record-Autoware.gif" width="478"> |
| :-----------------: | :-----------------: |

5. Engage the vehicle in autonomous mode to track the reference trajectory in real-time.
    ```bash
    user@host-pc:~$ ros2 launch autodrive_hunter simulator_replay_3d.launch.py
    user@host-pc:~$ ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/home/<username>/path"}" --feedback
    ```
    > **Note:** Replace `<username>` with your actual username. Be sure to use the correct path to load the trajectory file.

| <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-HunterSE-Forest-Simulator/Replay-HunterSE.gif" width="478"> | <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-HunterSE-Forest-Simulator/Replay-Autoware.gif" width="478"> |
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
