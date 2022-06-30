# simulator_compatibility_test

## Purpose

Test procedures (e.g. test codes) to check whether a certain simulator is compatible with Autoware


## Test

```bash
source install/setup.bash
colcon build --packages-select simulator_compatibility_test
cd src/universe/autoware.universe/tools/simulator_test/simulator_compatibility_test/test_another_sim
```
To run each test case manually
```bash
python -m pytest test_01_control_mode_and_report.py
python -m pytest test_02_change_gear_and_report.py
python -m pytest test_03_longitudinal_command_and_report.py
python -m pytest test_04_lateral_command_and_report.py
python -m pytest test_05_turn_indicators_cmd_and_report.py
python -m pytest test_06_hazard_lights_cmd_and_report.py
```
To run all test cases automatically just invoke pytest.
```bash
pytest
```
If you are interested in testing with MORAI SIM: Drive, move to the folder named test_moraisim.
And then, you can use the commands above as the same way.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                        | Type                                           | Description       |
| --------------------------- | ---------------------------------------------- | ----------------- |
| `/vehicle/status/control_mode` | `autoware_auto_vehicle_msgs::msg::ControlModeReport` | for [Test Case #1] |
| `/vehicle/status/gear_status` | `autoware_auto_vehicle_msgs::msg::GearReport` | for [Test Case #2] |
| `/vehicle/status/velocity_status` | `autoware_auto_vehicle_msgs::msg::VelocityReport` | for [Test Case #3] |
| `/vehicle/status/steering_status` | `autoware_auto_vehicle_msgs::msg::SteeringReport` | for [Test Case #4] |
| `/vehicle/status/turn_indicators_status` | `autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport` | for [Test Case #5] |
| `/vehicle/status/hazard_lights_status` | `autoware_auto_vehicle_msgs::msg::HazardLightsReport` | for [Test Case #6] |

### Output

| Name                        | Type                                           | Description       |
| --------------------------- | ---------------------------------------------- | ----------------- |
| `/control/command/control_mode_cmd` | `autoware_auto_vehicle_msgs/ControlModeCommand` | for [Test Case #1] |
| `/control/command/gear_cmd` | `autoware_auto_vehicle_msgs/GearCommand` | for [Test Case #2] |
| `/control/command/control_cmd` | `autoware_auto_vehicle_msgs/AckermannControlCommand` | for [Test Case #3, #4] |
| `/vehicle/status/steering_status` | `autoware_auto_vehicle_msgs/TurnIndicatorsCommand` | for [Test Case #5] |
| `/control/command/turn_indicators_cmd` | `autoware_auto_vehicle_msgs/HazardLightsCommand` | for [Test Case #6] |


## Parameters

None.

### Node Parameters

None.

### Core Parameters

None.

## Assumptions / Known limits

None.