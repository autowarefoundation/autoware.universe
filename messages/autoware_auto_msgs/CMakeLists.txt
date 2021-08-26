# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_msgs)

# Generate messages
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlanTrajectory.idl"
  "action/RecordTrajectory.idl"
  "action/ReplayTrajectory.idl"
  "msg/AckermannControlCommand.idl"
  "msg/AckermannLateralCommand.idl"
  "msg/AutowareState.idl"
  "msg/BoundingBox.idl"
  "msg/BoundingBoxArray.idl"
  "msg/ClassifiedRoi.idl"
  "msg/ClassifiedRoiArray.idl"
  "msg/Complex32.idl"
  "msg/ControlDiagnostic.idl"
  "msg/DetectedObject.idl"
  "msg/DetectedObjectKinematics.idl"
  "msg/DetectedObjects.idl"
  "msg/DiagnosticHeader.idl"
  "msg/DrivingCapability.idl"
  "msg/EmergencyState.idl"
  "msg/Engage.idl"
  "msg/Float32MultiArrayDiagnostic.idl"
  "msg/GearReport.idl"
  "msg/HADMapBin.idl"
  "msg/HADMapRoute.idl"
  "msg/HADMapSegment.idl"
  "msg/HandBrakeCommand.idl"
  "msg/HandBrakeReport.idl"
  "msg/HazardLightsCommand.idl"
  "msg/HazardLightsReport.idl"
  "msg/HazardStatus.idl"
  "msg/HazardStatusStamped.idl"
  "msg/HeadlightsCommand.idl"
  "msg/HeadlightsReport.idl"
  "msg/HighLevelControlCommand.idl"
  "msg/HornCommand.idl"
  "msg/HornReport.idl"
  "msg/LongitudinalCommand.idl"
  "msg/MapPrimitive.idl"
  "msg/ObjectClassification.idl"
  "msg/Path.idl"
  "msg/PathPoint.idl"
  "msg/PathPointWithLaneId.idl"
  "msg/PathWithLaneId.idl"
  "msg/PointClusters.idl"
  "msg/PointXYZIF.idl"
  "msg/PredictedObject.idl"
  "msg/PredictedObjectKinematics.idl"
  "msg/PredictedObjects.idl"
  "msg/PredictedPath.idl"
  "msg/Quaternion32.idl"
  "msg/RawControlCommand.idl"
  "msg/RelativePositionWithCovarianceStamped.idl"
  "msg/Route.idl"
  "msg/RoutePoint.idl"
  "msg/Shape.idl"
  "msg/StationaryLockingCommand.idl"
  "msg/TrackedObject.idl"
  "msg/TrackedObjectKinematics.idl"
  "msg/TrackedObjects.idl"
  "msg/Trajectory.idl"
  "msg/TrajectoryPoint.idl"
  "msg/TurnIndicatorsCommand.idl"
  "msg/TurnIndicatorsReport.idl"
  "msg/VehicleControlCommand.idl"
  "msg/VehicleKinematicState.idl"
  "msg/VehicleOdometry.idl"
  "msg/VehicleStateCommand.idl"
  "msg/VehicleStateReport.idl"
  "msg/WheelEncoder.idl"
  "msg/WipersCommand.idl"
  "msg/WipersReport.idl"
  "srv/AutonomyModeChange.idl"
  "srv/HADMapService.idl"
  "srv/ModifyTrajectory.idl"
  DEPENDENCIES
    "builtin_interfaces"
    "diagnostic_msgs"
    "geometry_msgs"
    "nav_msgs"
    "sensor_msgs"
    "shape_msgs"
    "std_msgs"
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
