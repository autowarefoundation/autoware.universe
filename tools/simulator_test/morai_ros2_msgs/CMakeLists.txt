cmake_minimum_required(VERSION 3.5)
project(morai_msgs)


find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/CollisionData.msg"
  "msg/CtrlCmd.msg"
  "msg/EgoVehicleStatus.msg"
  "msg/ERP42Info.msg"
  "msg/EventInfo.msg"
  "msg/GetTrafficLightStatus.msg"
  "msg/GhostMessage.msg"
  "msg/GPSMessage.msg"
  "msg/IntersectionControl.msg"
  "msg/IntersectionStatus.msg"
  "msg/IntscnTL.msg"
  "msg/Lamps.msg"
  "msg/MapSpec.msg"
  "msg/MapSpecIndex.msg"
  "msg/MoraiSimProcHandle.msg"
  "msg/MoraiSimProcStatus.msg"
  "msg/MoraiSrvResponse.msg"
  "msg/MoraiTLIndex.msg"
  "msg/MoraiTLInfo.msg"
  "msg/MultiEgoSetting.msg"
  "msg/NpcGhostCmd.msg"
  "msg/NpcGhostInfo.msg"
  "msg/ObjectStatus.msg"
  "msg/ObjectStatusList.msg"
  "msg/PRCtrlCmd.msg"
  "msg/PREvent.msg"
  "msg/PRStatus.msg"
  "msg/RadarTrack.msg"
  "msg/RadarTracks.msg"
  "msg/ReplayInfo.msg"
  "msg/SaveSensorData.msg"
  "msg/ScenarioLoad.msg"
  "msg/SensorPosControl.msg"
  "msg/SetTrafficLight.msg"
  "msg/SyncModeAddObj.msg"
  "msg/SyncModeCmd.msg"
  "msg/SyncModeCmdResponse.msg"
  "msg/SyncModeCtrlCmd.msg"
  "msg/SyncModeInfo.msg"
  "msg/SyncModeRemoveObj.msg"
  "msg/SyncModeResultResponse.msg"
  "msg/SyncModeScenarioLoad.msg"
  "msg/SyncModeSetGear.msg"
  "msg/TrafficLight.msg"
  "msg/VehicleCollision.msg"
  "msg/VehicleCollisionData.msg"
  "msg/VehicleSpec.msg"
  "msg/VehicleSpecIndex.msg"
  "msg/WaitForTick.msg"
  "msg/WaitForTickResponse.msg"
)

set(srv_files
  "srv/MoraiEventCmdSrv.srv"
  "srv/MoraiMapSpecSrv.srv"
  "srv/MoraiScenarioLoadSrv.srv"
  "srv/MoraiSimProcSrv.srv"
  "srv/MoraiSyncModeCmdSrv.srv"
  "srv/MoraiSyncModeCtrlCmdSrv.srv"
  "srv/MoraiSyncModeSetGearSrv.srv"
  "srv/MoraiSyncModeSLSrv.srv"
  "srv/MoraiTLInfoSrv.srv"
  "srv/MoraiVehicleSpecSrv.srv"
  "srv/MoraiWaitForTickSrv.srv"
  "srv/PREventSrv.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
