# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_msgs)

# Generate messages
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  # Helper types
  "msg/BoundingBox.msg"
  "msg/Complex32.msg"
  "msg/DiagnosticHeader.msg"
  "msg/Quaternion32.msg"
  "msg/TrajectoryPoint.msg"
  # Interfaces
  "msg/BoundingBoxArray.msg"
  "msg/ControlDiagnostic.msg"
  "msg/HighLevelControlCommand.msg"
  "msg/RawControlCommand.msg"
  "msg/Trajectory.msg"
  "msg/VehicleControlCommand.msg"
  "msg/VehicleKinematicState.msg"
  "msg/VehicleOdometry.msg"
  "msg/VehicleStateCommand.msg"
  "msg/VehicleStateReport.msg"
  # Implementation-specific messages
  "msg/PointClusters.msg"
  DEPENDENCIES "std_msgs" "geometry_msgs" "sensor_msgs")

ament_auto_package()
