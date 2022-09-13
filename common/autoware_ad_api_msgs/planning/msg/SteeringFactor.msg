# constants for common use
uint16 UNKNOWN = 0

# constants for type
uint16 INTERSECTION = 1
uint16 LANE_CHANGE = 2
uint16 AVOIDANCE_PATH_CHANGE = 3
uint16 AVOIDANCE_PATH_RETURN = 4
uint16 STATION = 5
uint16 PULL_OUT = 6
uint16 PULL_OVER = 7
uint16 EMERGENCY_OPERATION = 8

# constants for direction
uint16 LEFT = 1
uint16 RIGHT = 2
uint16 STRAIGHT = 3

# constants for status
uint16 APPROACHING = 1
uint16 TRYING = 2
uint16 TURNING = 3

# variables
geometry_msgs/Pose[2] pose
float32[2] distance
uint16 type
uint16 direction
uint16 status
string detail
