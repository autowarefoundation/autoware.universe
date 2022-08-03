# Vehicle Interface Package

## Purpose of Package

`vehicle_interface` provides an API to bridge Autoware and any vehicle platform, whether or not it is part of an offical Autoware ODD. It converts Autoware control commands and vehicle commands to vehicle-specific messages, and convert the vehicle status (steering angle report, headlight state, etc.) to Autoware messages.

While the Autoware stack speaks ROS2 messages, the vehicle side of the interface could be in many forms. In most cases the vehicle interface simply publishes a different set of messages to a vehicle ROS2 driver, who then handles the communication with the actual vehicle via CAN, TCP/IP, USB, etc. In a few cases, the vehicle interface itself may include an instance of the vehicle platform's C++ SDK, who then communicates directly with the vehicle without a separate ROS2 driver.

`vehicle_interface` provides an abstract ROS2 node `vehicle_interface_node`, which any custom vehicle interface should inherit from. The developer should implement callbacks triggered by Autoware commands, and provide status reports that Autoware is subscribing to.

## Design

The package contains these classes:

- `InterfaceFeature` in `base_interface.hpp`: defines feature enums that an interface may have.
- `BaseInterface` in `base_interface.hpp`: defines the API messages to and from Autoware.
- `VehicleInterfaceNode` in `vehicle_interface_node.hpp`: inherits from `BaseInterface` and creates pubs and subs to/from Autoware.

<div align="center">
  <img src="image/vehicle_interface_design.svg" width=80%>
</div>

### Publishers

These publishers of `VehicleInterfaceNode` fetch a member variable from `BaseInterface` and publish at a constant interval.

| Topic Name             | Message Type                                        | `BaseInterface` Member   |
| ---------------------- | --------------------------------------------------- | ------------------------ |
| gear_report            | autoware_auto_vehicle_msgs/msg/GearReport           | m_gear_report            |
| hand_brake_report      | autoware_auto_vehicle_msgs/msg/HandBrakeReport      | m_hand_brake_report      |
| hazard_lights_report   | autoware_auto_vehicle_msgs/msg/HazardLightsReport   | m_hazard_lights_report   |
| headlights_report      | autoware_auto_vehicle_msgs/msg/HeadlightsReport     | m_headlights_report      |
| horn_report            | autoware_auto_vehicle_msgs/msg/HornReport           | m_horn_report            |
| wipers_report          | autoware_auto_vehicle_msgs/msg/WipersReport         | m_wipers_report          |
| turn_indicators_report | autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport | m_turn_indicators_report |
| odom                   | nav_msgs/msg/Odometry                               | m_odometry               |

### Subscribers

These publishers of `VehicleInterfaceNode` trigger a callback in `BaseInterface`.

| Topic Name           | Message Type                                           | `BaseInterface` Method       |
| -------------------- | ------------------------------------------------------ | ---------------------------- |
| control_cmd          | autoware_auto_vehicle_msgs/msg/AckermannControlCommand | send_control_command         |
| gear_cmd_            | autoware_auto_vehicle_msgs/msg/GearReport              | send_gear_command            |
| hand_brake_cmd_      | autoware_auto_vehicle_msgs/msg/HandBrakeReport         | send_hand_brake_command      |
| hazard_lights_cmd_   | autoware_auto_vehicle_msgs/msg/HazardLightsReport      | send_hazard_lights_command   |
| headlights_cmd_      | autoware_auto_vehicle_msgs/msg/HeadlightsReport        | send_headlights_command      |
| horn_cmd_            | autoware_auto_vehicle_msgs/msg/HornReport              | send_horn_command            |
| wipers_cmd_          | autoware_auto_vehicle_msgs/msg/WipersReport            | send_wipers_command          |
| turn_indicators_cmd_ | autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport    | send_turn_indicators_command |

### Service

These services of `VehicleInterfaceNode` trigger a callback in `BaseInterface`.

| Topic Name    | Service Type                                           | `BaseInterface` Method     |
| ------------- | ------------------------------------------------------ | -------------------------- |
| autonomy_mode | autoware_auto_vehicle_msgs/msg/AckermannControlCommand | handle_mode_change_request |

### Parameters

| Parameter Name      | Type   | Description                            |
| ------------------- | ------ | -------------------------------------- |
| report_interval_sec | double | Interval of publishing vehicle reports |

## How to Use

To start developing your own interface:

1. Create a new package that depends on `vehicle_interface`.
2. Create a ROS2 node that inherits from `vehicle_interface_node`.
3. Declare and implement required callbacks `send_control_command` and `handle_mode_change_request`.
4. Declare and implement optional callbacks.

Difference vehicle interfaces may have different features. For example, some may provide headlight operations, while some do not. To handle this, the `vehicle_interface_node` constructor takes a `FeatureSet` object, and selectively initializes only publishers and subscribers that are present in the feature set. For example, by passing `InterfaceFeature::GEAR`, the publisher of `GearReport` and subscriber of `GearCommand` are declared, and the developer needs to implement `send_gear_command` which would otherwise throw exception.

An example of implementation can be found in `vesc_interface` package, used by Autoware racing ODD on F1TENTH vehicles.

### Related Issue

An analysis of the Autoware.Auto's approach to `vehicle_interface` can be found in this [issue](https://github.com/autowarefoundation/autoware.universe/issues/1451).
