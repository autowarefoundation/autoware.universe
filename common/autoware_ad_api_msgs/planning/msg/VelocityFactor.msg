# constants for common use
uint16 UNKNOWN = 0

# constants for type
uint16 SURROUNDING_OBSTACLE = 1
uint16 ROUTE_OBSTACLE = 2
uint16 INTERSECTION = 3
uint16 CROSSWALK = 4
uint16 REAR_CHECK = 5
uint16 USER_DEFINED_DETECTION_AREA = 6
uint16 NO_STOPPING_AREA = 7
uint16 STOP_SIGN = 8
uint16 TRAFFIC_SIGNAL = 9
uint16 V2I_GATE_CONTROL_ENTER = 10
uint16 V2I_GATE_CONTROL_LEAVE = 11
uint16 MERGE = 12
uint16 SIDEWALK = 13
uint16 LANE_CHANGE = 14
uint16 AVOIDANCE = 15
uint16 EMERGENCY_STOP_OPERATION = 16

# constants for status
uint16 APPROACHING = 1
uint16 STOPPED = 2

# variables
geometry_msgs/Pose pose
float32 distance
uint16 type
uint16 status
string detail
