# Vehicle

## Overview

Vehicle stack is an interface between Autoware and vehicle. This layer converts signals from Autoware to vehicle-specific, and vice versa.
This module needs to be designed according to the vehicle to be used. How to implement a new interface is described [below.](#how-to-design-a-new-vehicle-interface)

## Role

There are two main roles of Vehicle stack:

- **Conversion of Autoware commands to a vehicle-specific format**
- **Conversion of vehicle status in a vehicle-specific format to Autoware messages**

## Assumption

It is assumed that the vehicle has one of the following control interfaces.

**Type A. target velocity or acceleration interface.**

**Type B. target throttle and brake pedals interface.**

The use case and requirements change according to this type.

## Use Cases

Vehicle stack supports the following use cases.

- Speed control with desired velocity or acceleration (for type A only)
- Speed control with desired throttle and brake pedals (for type B only)
- Steering control with desired steering angle and/or steering angle velocity (for both)
- Shift control (for both)
- Turn signal control (for both)

## Requirement

To achieve the above use case, the vehicle stack requires the following conditions.

### Speed control with desired velocity or acceleration (for type A)

- The vehicle can be controlled by the target velocity or acceleration.
- The input vehicle command includes target velocity or acceleration.
- The output to the vehicle includes desired velocity or acceleration in a vehicle-specific format.

### Speed control with the desired throttle and brake pedals (for type B)

- The vehicle can be controlled by the target throttle and brake pedals.
- The input vehicle command includes target throttle and brake pedals for the desired speed.
- The output to the vehicle includes desired throttle and brake pedals in a vehicle-specific format.

### Steering control with the desired steering angle and/or steering angle velocity

- The vehicle can be controlled by the target steer angle and/or steering angle velocity.
- The input vehicle command includes the target steering angle and/or target steering angle velocity.
- The output to the vehicle includes the desired steering angle and/or steering angle velocity in a vehicle-specific format.

### Shift control

- The vehicle can be controlled by the target shift mode.
- The input vehicle command includes the desired shift.
- The output to the vehicle includes the desired shift in a vehicle-specific format.

### Turn signal control

- The vehicle can be controlled by the target turn signal mode.
- The input vehicle command includes the desired turn signal.
- The output to the vehicle includes the desired turn signal in a vehicle-specific format.

## Input

The input to Vehicle stack:

| Input           | Topic(Data Type)                                                   | Explanation |
| --------------- | ------------------------------------------------------------------ | ----------- |
| Vehicle Command | `/control/vehicle_cmd`<br>(`autoware_vehicle_msgs/VehicleCommand`) | Table Below |

The detailed contents in Vehicle Command are as follows.

| Input                   | Data Type        | Explanation                            |
| ----------------------- | ---------------- | -------------------------------------- |
| Velocity                | std_msgs/Float64 | Target velocity [m/s]                  |
| Acceleration            | std_msgs/Float64 | Target acceleration [m/s2]             |
| Steering angle          | std_msgs/Float64 | Target steering angle [rad]            |
| Steering angle velocity | std_msgs/Float64 | Target steering angle velocity [rad/s] |
| Gear shifting command   | std_msgs/Int32   | Target Gear shift                      |
| Emergency command       | std_msgs/Int32   | Emergency status of Autoware           |

### Output

There are two types of outputs from Vehicle stack: vehicle status to Autoware and a control command to the vehicle.

The table below summarizes the output from Vehicle stack:

| Output (to Autoware)          | Topic(Data Type)                                                      | Explanation                                  |
| ----------------------------- | --------------------------------------------------------------------- | -------------------------------------------- |
| velocity status               | `/vehicle/status/twist`<br>(`geometry_msgs/TwistStamped`)             | vehicle velocity status to Autoware [m/s]    |
| steering status (optional)    | `/vehicle/status/steering`<br>(`autoware_vehicle_msgs/Steering`)      | vehicle steering status to Autoware [rad]    |
| Shift status (optional)       | `/vehicle/status/Shift`<br>(`autoware_vehicle_msgs/ShiftStamped`)     | vehicle shift to Autoware [-]                |
| Turn signal status (optional) | `/vehicle/status/turn_signal`<br>(`autoware_vehicle_msgs/TurnSignal`) | vehicle turn signal status to Autoware [m/s] |

The output to the vehicle depends on each vehicle interface.

| Output (to vehicle)      | Topic(Data Type)        | Explanation                          |
| ------------------------ | ----------------------- | ------------------------------------ |
| vehicle control messages | Depends on each vehicle | Control signals to drive the vehicle |

## Design

For vehicles of the type controlled by the target velocity or acceleration (type A)

![Vehicle_design_typeA](image/VehicleInterfaceDesign1.png)

For vehicles of the type controlled by the target throttle and brake pedals (type B)

![Vehicle_design_typeB](image/VehicleInterfaceDesign2.png)

## Vehicle Interface

### Role

To convert Autoware control messages to vehicle-specific format, and generate vehicle status messages from vehicle-specific format.

### Input

- Vehicle Command (`autoware_vehicle_msgs/VehicleCommand`) (type A only)
  - includes target velocity, acceleration, steering angle, steering angle velocity, gear shift, and emergency.
- Raw Vehicle Command (`autoware_vehicle_msgs/RawVehicleCommand`) (type B only)
  - includes target throttle pedal, brake pedal, steering angle, steering angle velocity, gear shift, and emergency.
- Turn signal (`autoware_vehicle_msgs/TurnSignal`) (optional)

### Output

- Velocity status (`geometry_msgs/TwistStamped`)
- Steering status (`autoware_vehicle_msgs/Steering`) (optional)
- Shift status (`autoware_vehicle_msgs/ShiftStamped`) (optional)
- Turn signal status (`autoware_vehicle_msgs/TurnSignal`) (optional)

NOTE: Lane driving is possible without the optional part. Design vehicle interface according to the purpose.

## Raw Vehicle Cmd Converter

### Role

To convert the target acceleration to the target throttle and brake pedals with the given acceleration map. This node is used only for the case of vehicle type B.

### Input

- Vehicle Command (`autoware_vehicle_msgs/VehicleCommand`)
- Current velocity (`geometry_msgs/TwistStamped`)

### Output

- Raw Vehicle Command (`autoware_vehicle_msgs/RawVehicleCommand`)
  - includes target throttle pedal, brake pedal, steering angle, steering angle velocity, gear shift, and emergency.

### How to design a new vehicle interface

#### For type A

Create a module that satisfies the following two requirements

- Receives `autoware_vehicle_msg/VehicleCommand` and sends control commands to the vehicle.
- Converts the information from the vehicle, publishes vehicle speed to Autoware with `geometry_msgs/TwistStamped`.

For example, if the vehicle has an interface to be controlled with a target velocity, the velocity in `autoware_vehicle_msg/VehicleCommand` is sent to the vehicle as the target velocity. If the vehicle control interface is steering wheel angle, it is necessary to convert steering angle to steering wheel angle in this vehicle_interface.

#### For type B

Since `autoware_vehicle_msg/VehicleCommand` contains only the target velocity and acceleration, you need to convert these values for the throttle and brake pedal interface vehicles. In this case, use the `RawVehicleCmdConverter`. The `RawVehicleCmdConverter` converts the target acceleration to the target throttle/brake pedal based on the given acceleration map. You need to create this acceleration map in advance from vehicle data sheets and experiments.

With the use of `RawVehicleCmdConverter`, you need to create a module that satisfies the following two requirements

- Receives `autoware_vehicle_msg/RawVehicleCommand` and sends control commands to the vehicle.
- Converts the information from the vehicle, publishes vehicle speed to Autoware with `geometry_msgs/TwistStamped`.

#### How to make an acceleration map (for type B)

When using the `RawVehicleCmdConverter` described above, it is necessary to create an acceleration map for each vehicle. The acceleration map is data in CSV format that describes how much acceleration is produced when the pedal pressed in each vehicle speed range. You can find the default acceleration map data in `src/vehicle/raw_vehicle_cmd_converter/data` as a reference. In the CSV data, the horizontal axis is the current velocity [m/s], the vertical axis is the vehicle-specific pedal value [-], and the element is the acceleration [m/ss] as described below.

![Vehicle_accel_map_description](image/VehicleAccelMapDescription.png)

This is the reference data created by TierIV with the following steps.

- Press the pedal to a constant value on a flat road to accelerate/decelerate the vehicle.
- Save IMU acceleration and vehicle velocity data during acceleration/deceleration.
- Create a CSV file with the relationship between pedal values and acceleration at each vehicle speed.

After your acceleration map is created, load it when `RawVehicleCmdConverter` is launched (the file path is defined at the launch file).

#### Control of additional elements, such as turn signals

If you need to control parts that are not related to the vehicle drive (turn signals, doors, window opening and closing, headlights, etc.), the vehicle interface will handle them separately. The current Autoware supports and implements only turn signals.
