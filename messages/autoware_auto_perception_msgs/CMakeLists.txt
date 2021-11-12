# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_perception_msgs)

# Generate messages
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox.idl"
  "msg/BoundingBoxArray.idl"
  "msg/ClassifiedRoi.idl"
  "msg/ClassifiedRoiArray.idl"
  "msg/DetectedObject.idl"
  "msg/DetectedObjectKinematics.idl"
  "msg/DetectedObjects.idl"
  "msg/LookingTrafficSignal.idl"
  "msg/ObjectClassification.idl"
  "msg/PointClusters.idl"
  "msg/PointXYZIF.idl"
  "msg/PredictedObject.idl"
  "msg/PredictedObjectKinematics.idl"
  "msg/PredictedObjects.idl"
  "msg/PredictedPath.idl"
  "msg/Shape.idl"
  "msg/TrackedObject.idl"
  "msg/TrackedObjectKinematics.idl"
  "msg/TrackedObjects.idl"
  "msg/TrafficLight.idl"
  "msg/TrafficLightRoi.idl"
  "msg/TrafficLightRoiArray.idl"
  "msg/TrafficSignal.idl"
  "msg/TrafficSignalArray.idl"
  "msg/TrafficSignalStamped.idl"
  "msg/TrafficSignalWithJudge.idl"
  DEPENDENCIES
    "autoware_auto_geometry_msgs"
    "geometry_msgs"
    "sensor_msgs"
    "std_msgs"
    "unique_identifier_msgs"
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
