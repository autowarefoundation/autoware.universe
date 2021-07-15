# Quick Start

## Rosbag simulation

1. [Download the sample pointcloud and vector maps](https://drive.google.com/open?id=1ovrJcFS5CZ2H51D8xVWNtEvj_oiXW-zk), unpack the zip archive and copy the two map files to the same folder.
2. Download the sample rosbag files and put them into the same folder, e.g., `~/rosbag2/sample/`.

   - [db3](https://drive.google.com/file/d/1wLWyOlfH_-k4VYBgae1KAFlKdwJnH_si/view?usp=sharing)
   - [yaml](https://drive.google.com/file/d/1Arb-QVnNHM-BFdB_icm7J7fWkyuZt7mZ/view?usp=sharing)

3. Open a terminal and launch Autoware

   ```sh
   cd ~/workspace/autoware.proj
   source install/setup.bash
   ros2 launch autoware_launch logging_simulator.launch.xml map_path:=/path/to/map_folder vehicle_model:=lexus sensor_model:=aip_xx1 rosbag:=true perception:=false
   ```

4. Open a second terminal and play the sample rosbag file

   ```sh
   cd ~/workspace/autoware.proj
   source install/setup.bash
   ros2 bag play /path/to/sample.625-2.bag2_0.db3 -r 0.2
   ```

5. Focus the view on the ego vehicle by changing the `Target Frame` in the RViz Views panel from `viewer` to `base_link`.

### Note

- Sample map and rosbag: © 2020 Tier IV, Inc.
  - Due to privacy concerns, the rosbag does not contain image data, and so traffic light recognition functionality cannot be tested with this sample rosbag. As a further consequence, object detection accuracy is decreased.

## Planning Simulator

1. [Download the sample pointcloud and vector maps](https://drive.google.com/open?id=197kgRfSomZzaSbRrjWTx614le2qN-oxx), unpack the zip archive and copy the two map files to the same folder.
2. Open a terminal and launch Autoware

```sh
cd ~/workspace/autoware.proj
source install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/path/to/map_folder vehicle_model:=lexus sensor_model:=aip_xx1
```

3. Set an initial pose for the ego vehicle

   - a) Click the `2D Pose estimate` button in the toolbar, or hit the `P` key
   - b) In the 3D View pane, click and hold the left-mouse button, and then drag to set the direction for the initial pose.

4. Set a goal pose for the ego vehicle

   - a) Click the `2D Nav Goal` button in the toolbar, or hit the `G` key
   - b) In the 3D View pane, click and hold the left-mouse button, and then drag to set the direction for the goal pose.

5. Engage the ego vehicle.

   - a) Open the [autoware_web_controller](http://localhost:8085/autoware_web_controller/) in a browser.
   - b) Click the `Engage` button.

### Note

- Sample map: © 2020 Tier IV, Inc.
