<p align="center">
<img src="https://github.com/Tinker-Twins/AutoDRIVE-Autoware/blob/main/autodrive_autoware/media/AutoDRIVE-Logo.png" alt="AutoDRIVE" width="390"/> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <img src="https://github.com/Tinker-Twins/AutoDRIVE-Autoware/blob/main/autodrive_autoware/media/Autoware-Logo.png" alt="Autoware" width="390"/> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <img src="https://github.com/Tinker-Twins/AutoDRIVE-Autoware/blob/main/autodrive_autoware/media/OpenCAV-Logo.png" alt="OpenCAV" width="150"/>
</p>

## 2D Navigation Demo (Digital Twin Simulation - AutoDRIVE Simulator)

| <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-OpenCAV-TinyTown-Simulator/Replay-OpenCAV.gif" width="478"> | <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-OpenCAV-TinyTown-Simulator/Replay-Autoware.gif" width="478"> |
| :-----------------: | :-----------------: |

1. Launch AutoDRIVE Simulator for OpenCAV and establish Autoware API bridge connection in single or distributed computing setting as applicable.
2. Map the environment (if not already accomplished) by driving (teleoperating) the vehicle around the environment.
    ```bash
    user@host-pc:~$ sudo apt install ros-galactic-slam-toolbox
    user@host-pc:~$ ros2 launch autodrive_opencav simulator_slam_2d.launch.py
    user@host-pc:~$ ros2 run autodrive_opencav teleop_keyboard
    ```
3. Record waypoints by driving (teleoperating) the vehicle around the environment while localizing against the map.
    ```bash
    user@host-pc:~$ ros2 launch autodrive_opencav simulator_record_2d.launch.py
    user@host-pc:~$ ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/home/<username>/path"}" --feedback
    user@host-pc:~$ ros2 run autodrive_opencav teleop_keyboard
    ```
    > **Note:** Replace `<username>` with your actual username. Feel free to use a different path to save the trajectory file.
4. Engage the vehicle in autonomous mode to track the reference trajectory in real-time.
    ```bash
    user@host-pc:~$ ros2 launch autodrive_opencav simulator_replay_2d.launch.py
    user@host-pc:~$ ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/home/<username>/path"}" --feedback
    ```
    > **Note:** Replace `<username>` with your actual username. Be sure to use the correct path to load the trajectory file.

## 3D Navigation Demo (Digital Twin Simulation - AutoDRIVE Simulator)

| <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-OpenCAV-City-Simulator/Replay-OpenCAV.gif" width="478"> | <img src="https://github.com/Tinker-Twins/Scaled-Autonomous-Vehicles/blob/main/Project%20Media/AutoDRIVE-OpenCAV-City-Simulator/Replay-Autoware.gif" width="478"> |
| :-----------------: | :-----------------: |

1. Launch AutoDRIVE Simulator for OpenCAV and establish Autoware API bridge connection in single or distributed computing setting as applicable.
2. Map the environment (if not already accomplished) by driving (teleoperating) the vehicle around the environment.
    -  Use the built-in 3D PCD mapping functionality of AutoDRIVE Simulator.
    -  Use standard ROS 2 3D SLAM packages to save a PCD map.
3. Record waypoints by driving (teleoperating) the vehicle around the environment while localizing against the map.
    ```bash
    user@host-pc:~$ ros2 launch autodrive_opencav simulator_record_3d.launch.py
    user@host-pc:~$ ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/home/<username>/path"}" --feedback
    user@host-pc:~$ ros2 run autodrive_opencav teleop_keyboard
    ```
    > **Note:** Replace `<username>` with your actual username. Feel free to use a different path to save the trajectory file.
4. Engage the vehicle in autonomous mode to track the reference trajectory in real-time.
    ```bash
    user@host-pc:~$ ros2 launch autodrive_opencav simulator_replay_3d.launch.py
    user@host-pc:~$ ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/home/<username>/path"}" --feedback
    ```
    > **Note:** Replace `<username>` with your actual username. Be sure to use the correct path to load the trajectory file.

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
