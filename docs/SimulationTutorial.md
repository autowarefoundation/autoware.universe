# Simulation in Autoware

Autoware provides 2 types of simulations. Rosbag is used for testing/validation for `Sensing`, `Localization` and `Perception` stacks. Planning Simulator is mainly used for testing/validation for `Planning` stack by simulating traffic rules, interactions with dynamic objects and control command to vehicle.
![sim](https://user-images.githubusercontent.com/8327598/79709776-0bd47b00-82fe-11ea-872e-d94ef25bc3bf.png)

## How to use rosbag for simulation
Assuming already completed [Autoware setup](https://github.com/tier4/AutowareArchitectureProposal#autoware-setup).

1. Download sample map from [here](https://drive.google.com/open?id=1ovrJcFS5CZ2H51D8xVWNtEvj_oiXW-zk).
2. Download sample rosbag from [here](https://drive.google.com/open?id=1BFcNjIBUVKwupPByATYczv2X4qZtdAeD).

| Sensor                | Topic name                               |
| --------------------- | ---------------------------------------- |
| Velodyne 128 (Top)    | /sensing/velodyne/top/velodyne_packets   |
| Velodyne 16 (Right)   | /sensing/velodyne/right/velodyne_packets |
| Velodyne 16 (Left)    | /sensing/velodyne/left/velodyne_packets  |
| IMU (Tamagawa TAG300) | /sensing/imu/tamagawa/imu_raw            |
| GNSS (Ublox F9P)      | /sensing/gnss/ublox/fix_velocity         |
|                       | /sensing/gnss/ublox/nav_sat_fix          |
|                       | /sensing/gnss/ublox/navpvt               |
| CAN data              | /vehicle/status/control_mode             |
|                       | /vehicle/status/shift                    |
|                       | /vehicle/status/steering                 |
|                       | /vehicle/status/twist                    |
| ~~Camera x 7~~        | ~~/sensing/camera/camera[]/image_raw~~   |

Note: Image data are removed due to privacy concerns.

3. Launch Autoware with rosbag mode.

```
source devel/setup.bash
roslaunch autoware_launch logging_simulator.launch map_path:=[path]
```

4. Play sample rosbag.

```
rosbag play --clock -r 0.2　sample.bag
```

![rosbag_sim](https://user-images.githubusercontent.com/10920881/79726334-9381b000-8325-11ea-9ac6-ebbb29b11f14.png)

##### Note

- sample map : © 2020 TierIV inc.
- rosbag : © 2020 TierIV inc.

## How to use Planning Simulator

Assuming already completed [Autoware setup](https://github.com/tier4/AutowareArchitectureProposal#autoware-setup).

1. Download sample map from [here](https://drive.google.com/open?id=197kgRfSomZzaSbRrjWTx614le2qN-oxx) and extract the zip file.
2. Launch Autoware with Planning Simulator

```
source devel/setup.bash
roslaunch autoware_launch planning_simulator.launch map_path:=[path]
```

![initial](https://user-images.githubusercontent.com/10920881/79816587-8b298380-83be-11ea-967c-8c45772e30f4.png)

3. Set initial position by using `2D Pose Estimate` in rviz.

![start](https://user-images.githubusercontent.com/10920881/79816595-8e247400-83be-11ea-857a-32cf096ac3dc.png)

4. Set goal position by using `2D Nav Goal` in rviz.

![goal](https://user-images.githubusercontent.com/10920881/79816596-8fee3780-83be-11ea-9ee4-caabbef3a385.png)

5. Engage vehicle.
   - a. Go to [autoware_web_controller](http://localhost:8085/autoware_web_controller/index.html).
   - b. Push `Engage` button.

![engage](https://user-images.githubusercontent.com/10920881/79714298-4db7ee00-830b-11ea-9ac4-11e126d7a7c4.png)

### Simulate dummy obstacles

- Set obstacles' position by using `2D Dummy Pedestrian` or `2D Dummy Car` in rviz.
  - Shorcut keys `l` and `k` are assigned respectively.
  - Can adjust obstacles' infomation including velocity, position/orientation error and etc, via `Tool Properties` in rviz.
  - Can delete all the objects by using `Delte All Objects` in rviz.
    ![dummy](https://user-images.githubusercontent.com/10920881/79742437-c9cb2980-833d-11ea-8ad7-7c3ed1a96540.png)

### Simulate parking maneuver

Set goal in parking area.

![parking](https://user-images.githubusercontent.com/10920881/79817389-56b6c700-83c0-11ea-873b-6ec73c8a5c38.png)

##### Note

- sample map : © 2020 TierIV inc.
