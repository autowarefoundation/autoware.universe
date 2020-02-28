# Diagnostic information for the controller
DiagnosticHeader header
# Controller specific information
bool new_trajectory
string<=256 trajectory_source
string<=256 pose_source
# the error between the current vehicle and the nearest neighbor point
float32 lateral_error_m
float32 longitudinal_error_m
float32 velocity_error_mps
float32 acceleration_error_mps2
float32 yaw_error_rad
float32 yaw_rate_error_rps
