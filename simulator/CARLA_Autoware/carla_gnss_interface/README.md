# gnss Interface Node

Convert GNSS CARLA messages to pose and pose with covariance

1. Receive GNSS Messages: Subscribe to the GNSS messages published by CARLA `carla/ego_vehicle/gnss`. These messages typically include latitude, longitude, and altitude.

2. Convert to ROS Pose and Pose with Covariance: Extract the position components (latitude, longitude, altitude) from the GNSS messages and create a ROS geometry_msgs/Pose and geometry_msgs/PoseWithCovariance message. Set the position fields (x, y, z) of the ROS Pose message by converting the corresponding latitude, longitude, and altitude values using proj.
