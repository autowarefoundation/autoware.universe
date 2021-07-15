# How to setup for the specific hardware configuration

In order to test Autoware in a real vehicle, it is necessary to setup Autoware for each specific combination of vehicle, drive-by-wire system and sensors as follows:

## 1. Sensor TF

- The sensor TF describes the positional relationship of each sensor to the vehicle's base link (defined as the center of the vehicle's rear axle) and has to be created for each configuration of sensors.
- Please setup following the [TF design document](https://github.com/tier4/AutowareArchitectureProposal.proj/blob/master/design/TF.md).

## 2. Vehicle interface

- The [vehicle interface](https://github.com/tier4/AutowareArchitectureProposal.proj/blob/master/design/Vehicle/Vehicle.md#vehicle-interface) is the Autoware module that communicates with the vehicle's DBW (drive-by-wire) system, and must be created for each specific combination of vehicle and DBW.
- Please create an appropriate vehicle interface following the ["How to design a new vehicle interface"](https://github.com/tier4/AutowareArchitectureProposal.proj/blob/master/design/Vehicle/Vehicle.md#how-to-design-a-new-vehicle-interface) section of the [Vehicle stack design document](https://github.com/tier4/AutowareArchitectureProposal.proj/blob/master/design/Vehicle/Vehicle.md).
- [Sample vehicle interface file](https://github.com/tier4/lexus_description.iv.universe/blob/master/launch/vehicle_interface.launch) (for the Lexus RX 450H vehicle using [AutonomouStuff's PacMod system](https://autonomoustuff.com/products/pacmod))

## 3. Vehicle info

- The `vehicle_info` YAML configuration file contains global parameters for the vehicle's physical configuration (e.g. wheel radius) that are read by Autoware in [rosparam format](http://wiki.ros.org/rosparam) and published to the ROS Parameter Server.
- The required parameters are as follows:

```txt
/vehicle_info/wheel_radius    # wheel radius
/vehicle_info/wheel_width     # wheel width
/vehicle_info/wheel_base      # between front wheel center and rear wheel center
/vehicle_info/wheel_tread     # between left wheel center and right wheel center
/vehicle_info/front_overhang  # between front wheel center and vehicle front
/vehicle_info/rear_overhang   # between rear wheel center and vehicle rear
/vehicle_info/vehicle_height  # from the ground point to the highest point
```

- [Sample vehicle info file](https://github.com/tier4/lexus_description.iv.universe/blob/master/config/vehicle_info.yaml) (for the Lexus RX 450H)

## 4. Sensor launch file

- The `sensor.launch` file defines which sensor driver nodes are launched when running Autoware, and is dependent on the specific sensors (type, OEM and model) that are to be used.
- [Sample sensor.launch file](https://github.com/tier4/autoware_launcher.iv.universe/blob/master/sensing_launch/launch/sensing.launch)
