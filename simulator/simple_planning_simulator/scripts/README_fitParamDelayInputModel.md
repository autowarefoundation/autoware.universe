# fitParamDelayInputModel.py scripts
## How to use
```
python fitParamDelayInputModel.py --bag_file [bagfile] (--cutoff_time [cutoff_time] --cutoff_freq [cutoff_freq] --min_delay [min_delay] --max_delay [max_delay] --delay_incr [delay_incr]) 
# in round brakets is optional arguments
python fitParamDelayInputModel.py --help # for more information
```

## Requirements python packages:
* numpy
* pandas

## Required topics
* autoware_msgs::VehicleCmd /vehicle_cmd: assuming
  * vehicle_cmd/ctrl_cmd/steering_angle is the steering angle input [rad]
  * vehicle_cmd/ctrl_cmd/linear_velocity is the vehicle velocity input [m/s]
* autoware_msgs::VehicleStatus /vehicle_status : assuming
  * vehicle_status/angle is the measured steering angle [rad]
  * vehicle_status/speed is the measured vehicle speed [km/h]

## Description 
* Paramter fitting for Input Delay Model (First Order System with Dead Time) with rosbag file input
* Arguments explaining:
  * CUTOFF_TIME: Cutoff time[sec]. Rosbag file often was start recording before autoware was run. Time before autoware was run should be cut off. This script will only consider data from t=cutoff_time to the end of the bag file (default is 1.0)
  * CUTOFF_FREQ: Cutoff freq for low-pass filter[Hz], negative value will disable low-pass filter (default is 0.1)
  * MIN_DELAY: Min value for searching delay loop (default is 0.1)
  * MAX_DELAY: Max value for searching delay loop (default is 1.0)
  * DELAY_INCR: Step value for searching delay loop (default is 0.01)
