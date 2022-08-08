cmake_minimum_required(VERSION 3.14)
project(autoware_ad_api_msgs)

find_package(autoware_cmake REQUIRED)
autoware_package()

rosidl_generate_interfaces(${PROJECT_NAME}
  common/msg/ResponseStatus.msg
  interface/srv/InterfaceVersion.srv
  operation_mode/srv/ChangeOperationMode.srv
  operation_mode/msg/OperationMode.msg
  operation_mode/msg/OperationModeState.msg
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

ament_auto_package()
