# simple_planning_simulator

## Purpose / Use cases

This node simulates the vehicle motion for a vehicle command in 2D using a simple vehicle model.

## Design

The purpose of this simulator is for the integration test of planning and control modules. This does not simulate sensing or perception, but is implemented in pure c++ only and works without GPU.

## Assumptions / Known limits

- It simulates only in 2D motion.
- It does not perform physical operations such as collision and sensing, but only calculates the integral results of vehicle dynamics.

## Inputs / Outputs / API

### input

- input/initialpose [`geometry_msgs/msg/PoseWithCovarianceStamped`] : for initial pose
- input/ackermann_control_command [`autoware_auto_msgs/msg/AckermannControlCommand`] : target command to drive a vehicle
- input/manual_ackermann_control_command [`autoware_auto_msgs/msg/AckermannControlCommand`] : manual target command to drive a vehicle (used when control_mode_request = Manual)
- input/gear_command [`autoware_auto_vehicle_msgs/msg/GearCommand`] : target gear command.
- input/manual_gear_command [`autoware_auto_vehicle_msgs/msg/GearCommand`] : target gear command (used when control_mode_request = Manual)
- input/turn_indicators_command [`autoware_auto_vehicle_msgs/msg/TurnIndicatorsCommand`] : target turn indicator command
- input/hazard_lights_command [`autoware_auto_vehicle_msgs/msg/HazardLightsCommand`] : target hazard lights command
- input/control_mode_request [`tier4_vehicle_msgs::srv::ControlModeRequest`] : mode change for Auto/Manual driving

### output

- /tf [`tf2_msgs/msg/TFMessage`] : simulated vehicle pose (base_link)
- /output/odometry [`nav_msgs/msg/Odometry`] : simulated vehicle pose and twist
- /output/steering [`autoware_auto_vehicle_msgs/msg/SteeringReport`] : simulated steering angle
- /output/control_mode_report [`autoware_auto_vehicle_msgs/msg/ControlModeReport`] : current control mode (Auto/Manual)
- /output/gear_report [`autoware_auto_vehicle_msgs/msg/ControlModeReport`] : simulated gear
- /output/turn_indicators_report [`autoware_auto_vehicle_msgs/msg/ControlModeReport`] : simulated turn indicator status
- /output/hazard_lights_report [`autoware_auto_vehicle_msgs/msg/ControlModeReport`] : simulated hazard lights status

## Inner-workings / Algorithms

### Common Parameters

| Name                  | Type   | Description                                                                                                                                | Default value        |
| :-------------------- | :----- | :----------------------------------------------------------------------------------------------------------------------------------------- | :------------------- |
| simulated_frame_id    | string | set to the child_frame_id in output tf                                                                                                     | "base_link"          |
| origin_frame_id       | string | set to the frame_id in output tf                                                                                                           | "odom"               |
| initialize_source     | string | If "ORIGIN", the initial pose is set at (0,0,0). If "INITIAL_POSE_TOPIC", node will wait until the `input/initialpose` topic is published. | "INITIAL_POSE_TOPIC" |
| add_measurement_noise | bool   | If true, the Gaussian noise is added to the simulated results.                                                                             | true                 |
| pos_noise_stddev      | double | Standard deviation for position noise                                                                                                      | 0.01                 |
| rpy_noise_stddev      | double | Standard deviation for Euler angle noise                                                                                                   | 0.0001               |
| vel_noise_stddev      | double | Standard deviation for longitudinal velocity noise                                                                                         | 0.0                  |
| angvel_noise_stddev   | double | Standard deviation for angular velocity noise                                                                                              | 0.0                  |
| steer_noise_stddev    | double | Standard deviation for steering angle noise                                                                                                | 0.0001               |

### Vehicle Model Parameters

#### vehicle_model_type options

- `IDEAL_STEER_VEL`
- `IDEAL_STEER_ACC`
- `IDEAL_STEER_ACC_GEARED`
- `DELAY_STEER_VEL`
- `DELAY_STEER_ACC`
- `DELAY_STEER_ACC_GEARED`
- `DELAY_STEER_MAP_ACC_GEARED`

The `IDEAL` model moves ideally as commanded, while the `DELAY` model moves based on a 1st-order with time delay model. The `STEER` means the model receives the steer command. The `VEL` means the model receives the target velocity command, while the `ACC` model receives the target acceleration command. The `GEARED` suffix means that the motion will consider the gear command: the vehicle moves only one direction following the gear.

The table below shows which models correspond to what parameters. The model names are written in abbreviated form (e.g. IDEAL_STEER_VEL = I_ST_V).

| Name                       | Type   | Description                                                                                                 | I_ST_V | I_ST_A | I_ST_A_G | D_ST_V | D_ST_A | D_ST_A_G | D_ST_M_ACC_G | Default value | unit    |
| :------------------------- | :----- | :---------------------------------------------------------------------------------------------------------- | :----- | :----- | :------- | :----- | :----- | :------- | :----------- | :------------ | :------ |
| acc_time_delay             | double | dead time for the acceleration input                                                                        | x      | x      | x        | x      | o      | o        | o            | 0.1           | [s]     |
| steer_time_delay           | double | dead time for the steering input                                                                            | x      | x      | x        | o      | o      | o        | o            | 0.24          | [s]     |
| vel_time_delay             | double | dead time for the velocity input                                                                            | x      | x      | x        | o      | x      | x        | x            | 0.25          | [s]     |
| acc_time_constant          | double | time constant of the 1st-order acceleration dynamics                                                        | x      | x      | x        | x      | o      | o        | o            | 0.1           | [s]     |
| steer_time_constant        | double | time constant of the 1st-order steering dynamics                                                            | x      | x      | x        | o      | o      | o        | o            | 0.27          | [s]     |
| steer_dead_band            | double | dead band for steering angle                                                                                | x      | x      | x        | o      | o      | o        | x            | 0.0           | [rad]   |
| vel_time_constant          | double | time constant of the 1st-order velocity dynamics                                                            | x      | x      | x        | o      | x      | x        | x            | 0.5           | [s]     |
| vel_lim                    | double | limit of velocity                                                                                           | x      | x      | x        | o      | o      | o        | o            | 50.0          | [m/s]   |
| vel_rate_lim               | double | limit of acceleration                                                                                       | x      | x      | x        | o      | o      | o        | o            | 7.0           | [m/ss]  |
| steer_lim                  | double | limit of steering angle                                                                                     | x      | x      | x        | o      | o      | o        | o            | 1.0           | [rad]   |
| steer_rate_lim             | double | limit of steering angle change rate                                                                         | x      | x      | x        | o      | o      | o        | o            | 5.0           | [rad/s] |
| debug_acc_scaling_factor   | double | scaling factor for accel command                                                                            | x      | x      | x        | x      | o      | o        | x            | 1.0           | [-]     |
| debug_steer_scaling_factor | double | scaling factor for steer command                                                                            | x      | x      | x        | x      | o      | o        | x            | 1.0           | [-]     |
| acceleration_map_path      | string | path to csv file for acceleration map which converts velocity and ideal acceleration to actual acceleration | x      | x      | x        | x      | x      | x        | o            | -             | [-]     |

`acceleration_map` is usend only for `DELAY_STEER_MAP_ACC_GEARED` and it shows the acceleration command on the vertical axis and the current velocity on the horizontal axis, with each cell representing the converted acceleration command that is actually used in the simulator's motion calculation. Values in between are linearly interpolated.

example of `acceleration_map.csv`

```csv
default,0.00,1.39,2.78,4.17,5.56,6.94,8.33,9.72,11.11,12.50,13.89,15.28,16.67
-4,-4.4,-4.363636551,-4.377800754,-4.123122297,-4.200251976,-3.941588901,-3.983020054,-3.8,-3.767618565,-3.76158025,-3.588141709,-3.5,-3.4
-3.5,-4,-3.912946297,-3.855195605,-3.637975596,-3.675459,-3.548710083,-3.417860611,-3.24471272,-3.246114944,-3.004011648,-3.039347414,-2.92900558,-2.8
-3,-3.4,-3.373812496,-3.331230923,-3,-2.997811863,-2.9,-2.875177095,-2.649806805,-2.433249009,-2.444935612,-2.431940746,-2.393829973,-2.3
-2.5,-2.8,-2.723184726,-2.719015619,-2.62231722,-2.41277305,-2.428188471,-2.258351491,-2.178778835,-2.113338781,-2.028335512,-1.964596576,-1.913704611,-1.85
-2,-2.3,-2.238080454,-2.124425664,-2.024509438,-1.916697062,-1.81361914,-1.66606418,-1.577847558,-1.511356594,-1.490684556,-1.398017631,-1.35,-1.3
-1.5,-1.7,-1.610613371,-1.470852729,-1.461088567,-1.404871706,-1.368495381,-1.286898193,-1.238396629,-1.1,-0.9943455959,-0.8298192234,-0.8,-0.78
-1,-1.3,-1.275235376,-1.103445843,-1.088406302,-1.039321358,-1.02105369,-0.9751710693,-0.8854352235,-0.8210113976,-0.607258157,-0.5174264392,-0.5412350047,-0.5610285835
-0.8,-0.9590832786,-0.9,-0.8186387583,-0.7447984172,-0.7023011258,-0.6545403693,-0.6320942295,-0.5935860191,-0.55048343,-0.4442914804,-0.3901021846,-0.3919597965,-0.35
-0.6,-0.7658249026,-0.7066297769,-0.6748573388,-0.6454624801,-0.5800721935,-0.5176519761,-0.5081073889,-0.4988131589,-0.4004919454,-0.3257258364,-0.3018138626,-0.3110086312,-0.3
-0.4,-0.45,-0.3955078655,-0.45,-0.4420551681,-0.378263005,-0.3483463745,-0.3051486357,-0.2979650452,-0.2632949777,-0.2982640387,-0.2920397922,-0.3068711635,-0.25
-0.2,-0.236347113,-0.24,-0.2466631395,-0.2166629978,-0.2289237699,-0.2494739762,-0.2681446089,-0.2930905187,-0.24,-0.2246614573,-0.1669471084,-0.177229732,-0.1213663697
0,0,0,-0.05,-0.05,-0.05,-0.05,-0.08,-0.08,-0.08,-0.08,-0.1,-0.1,-0.1
0.2,0.16,0.12,0.0208379152,0.02,0,0,-0.05,-0.05,-0.05,-0.05,-0.08,-0.08,-0.08
0.4,0.3750641135,0.3047092516,0.221616677,0.2471973044,0.2350857222,0.231864649,0.2018410901,0.1562028944,0.1616276318,0.14,0.1,0.05,0.05
0.6,0.52,0.5193656045,0.5070327645,0.4865393415,0.4329989451,0.3958019322,0.354683087,0.3307829017,0.3291051391,0.3347948202,0.3177862573,0.3439621507,0.34
0.8,0.8239214061,0.8064813329,0.7847310427,0.6810651129,0.6338313891,0.5583948098,0.5275341787,0.4842021213,0.4326073185,0.4077661307,0.3723376985,0.38,0.3985055042
1,1.1,1.081666906,1.010666969,0.8781165326,0.762716663,0.6873783468,0.6553170752,0.5790677856,0.5361829552,0.48682048,0.45491258,0.3999384688,0.4
1.5,1.52,1.501457265,1.381911528,1.257342524,1.139809219,1.033006839,0.9103484248,0.8172015502,0.6687928801,0.6055502227,0.5058744415,0.4088174437,0.41
2,1.8,1.7975859,1.638490281,1.426616009,1.251194668,1.113771502,0.9618652599,0.8098033929,0.6953401352,0.5896689884,0.51189749,0.4180952453,0.42
```

![accerlation_map](https://private-user-images.githubusercontent.com/39142679/286099389-97a77043-86af-4384-8612-65b2b81fa360.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTEiLCJleHAiOjE3MDExNDA1NDMsIm5iZiI6MTcwMTE0MDI0MywicGF0aCI6Ii8zOTE0MjY3OS8yODYwOTkzODktOTdhNzcwNDMtODZhZi00Mzg0LTg2MTItNjViMmI4MWZhMzYwLnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFJV05KWUFYNENTVkVINTNBJTJGMjAyMzExMjglMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjMxMTI4VDAyNTcyM1omWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPWRhMTM1YTMyYzBhNDEzNDA4MTU4MGEyMTdlMDhlNDUzYmMxYTUwOGRjOWJkYmU5NWQyNjg5OGMzNDIyOGNiOTEmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0JmFjdG9yX2lkPTAma2V5X2lkPTAmcmVwb19pZD0wIn0.D7rTJhrKAz0J8FmlYOQ1QNbYjIEpMiPlBq9JHCbu1PY)

<!-- deadzone_delta_steer | double | dead zone for the steering dynamics                  | x      | x      | x        | o      | o      | 0.0      | [rad]         |         | -->

_Note_: The steering/velocity/acceleration dynamics is modeled by a first order system with a deadtime in a _delay_ model. The definition of the _time constant_ is the time it takes for the step response to rise up to 63% of its final value. The _deadtime_ is a delay in the response to a control input.

### Default TF configuration

Since the vehicle outputs `odom`->`base_link` tf, this simulator outputs the tf with the same frame_id configuration.
In the simple_planning_simulator.launch.py, the node that outputs the `map`->`odom` tf, that usually estimated by the localization module (e.g. NDT), will be launched as well. Since the tf output by this simulator module is an ideal value, `odom`->`map` will always be 0.

### (Caveat) Pitch calculation

Ego vehicle pitch angle is calculated in the following manner.

![pitch calculation](./media/pitch-calculation.drawio.svg)

NOTE: driving against the line direction (as depicted in image's bottom row) is not supported and only shown for illustration purposes.

## Error detection and handling

The only validation on inputs being done is testing for a valid vehicle model type.

## Security considerations

<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->

## References / External links

This is originally developed in the Autoware.AI. See the link below.

<https://github.com/Autoware-AI/simulation/tree/master/wf_simulator>

## Future extensions / Unimplemented parts

- Improving the accuracy of vehicle models (e.g., adding steering dead zones and slip behavior)
- Cooperation with modules that output pseudo pointcloud or pseudo perception results
