# VehicleKinematicState.msg
# Representation of a trajectory point with timestamp for the controller
std_msgs/Header header
TrajectoryPoint state
geometry_msgs/Transform delta

# state::time_from_start should contain the time since the last published message
# delta should contain the position update relative to the last published message
# state::x and state::y should correspond to the vehicle's position in a fixed frame, optional
# If header::frame_id is a dynamic frame (e.g. /base_link), then state::x and state::y are not valid
