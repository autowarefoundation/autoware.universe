# How to setup for the specific hardware configulation


In order to test the autoware in the vehicle, you need to setup for the specific hardware configulation. Please make the following settings.

## 1. Sensor TF setting

The sensor tf describes the positional relationship of the sensors and has to be created with the specific sensor hardware configuration. Please setup following [TF.md](https://github.com/tier4/AutowareArchitectureProposal/blob/master/design/TF.md). 

## 2. Vehicle interface setting
The vehicle interface is the module that communicates with the vehicle and has to be created with the specific vehicle configuration. Please setup the vehicle interface following [Vehicle.md](https://github.com/tier4/AutowareArchitectureProposal/blob/master/design/Vehicle/Vehicle.md).

## 3. Vehicle info setting

The `vehicle_info` is a global parameter for the vehicle configurations. that is read by the Autoware modules. These parameters are read by the Autoware modules and has to be published as rosparam format. The sample is [here](https://github.com/tier4/AutowareArchitectureProposal/blob/master/src/vehicle/vehicle_description/vehicle_body_description/lexus_description/config/vehicle_info.yaml).

Required parameters are as follows.
```
/vehicle_info/wheel_radius    # wheel radius
/vehicle_info/wheel_width     # wheel width
/vehicle_info/wheel_base      # between front wheel center and rear wheel center
/vehicle_info/wheel_tread     # between left wheel center and right wheel center
/vehicle_info/front_overhang  # between front wheel center and vehicle front
/vehicle_info/rear_overhang   # between rear wheel center and vehicle rear 
/vehicle_info/vehicle_height  # from the ground point to the highest point
```

## 4. Launch files setting

The following launch files has to be modified for the specific configulation.

**sensor.launch**

The `sensor.launch` defines what or which sensor driver nodes are launched. It is necessary to modify it according to the sensor configuration. The default setting is [here](https://github.com/tier4/AutowareArchitectureProposal/blob/master/src/launcher/sensing_launch/launch/sensing.launch).

<!-- **vehicle.launch**

The `vehicle.launch` starts up the sensor TF or vehicle configs / interface described above. Please modify this file according to your setup. The default setting is [here](). -->