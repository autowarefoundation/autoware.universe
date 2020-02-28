# Representation of a trajectory point for the controller
builtin_interfaces/Duration time_from_start
float32 x 0.0
float32 y 0.0
Complex32 heading
float32 longitudinal_velocity_mps 0.0
float32 lateral_velocity_mps 0.0
float32 acceleration_mps2 0.0
float32 heading_rate_rps 0.0
float32 front_wheel_angle_rad 0.0
float32 rear_wheel_angle_rad 0.0

# heading (1, 0) corresponds to 0 rad, corresponding to the +x direction in the appropriate frame
