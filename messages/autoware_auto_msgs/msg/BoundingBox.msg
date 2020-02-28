# Oriented bounding box representation
geometry_msgs/Point32 centroid
geometry_msgs/Point32 size
Quaternion32 orientation
float32 velocity 0.0
float32 heading 0.0
float32 heading_rate 0.0
geometry_msgs/Point32[4] corners
float32[8] variance
# can hold arbitrary value, e.g. likelihood, area, perimeter
float32 value
# can hold one of the vehicle constants defined below
uint8 vehicle_label 0  # NO_LABEL as default value
# can hold one of the signal constants defined below
uint8 signal_label 0   # NO_SIGNAL as default value
# Likelihood of vehicle label
float32 class_likelihood

# vehicle constants
uint8 NO_LABEL=0
uint8 CAR=1
uint8 PEDESTRIAN=2
uint8 CYCLIST=3
uint8 MOTORCYCLE=4

# Signal constants
uint8 NO_SIGNAL=0
uint8 LEFT_SIGNAL=1
uint8 RIGHT_SIGNAL=2
uint8 BRAKE=3

## Indexing constants
# Index for x position variance in `variance` array
uint32 POSE_X=0
# Index for y position variance in `variance` array
uint32 POSE_Y=1
# Index for velocity variance in `variance` array
uint32 VELOCITY=2
# Index for heading position variance in `variance` array
uint32 HEADING=3
# Index for turn rate position variance in `variance` array
uint32 TURN_RATE=4
# Index for x size (length) variance in `variance` array
uint32 SIZE_X=5
# Index for y size (width) variance in `variance` array
uint32 SIZE_Y=6
# Index for acceleration variance in `variance` array
uint32 ACCELERATION=7
