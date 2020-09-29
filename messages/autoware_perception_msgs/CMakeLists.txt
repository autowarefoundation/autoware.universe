cmake_minimum_required(VERSION 3.5)
project(autoware_perception_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(uuid_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/object_recognition/DynamicObject.msg"
  "msg/object_recognition/DynamicObjectArray.msg"
  "msg/object_recognition/Shape.msg"
  "msg/object_recognition/Semantic.msg"
  "msg/object_recognition/State.msg"
  "msg/object_recognition/Feature.msg"
  "msg/object_recognition/PredictedPath.msg"
  "msg/object_recognition/DynamicObjectWithFeature.msg"
  "msg/object_recognition/DynamicObjectWithFeatureArray.msg"
  "msg/object_recognition/PredictedPath.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs uuid_msgs
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/traffic_light_recognition/LampState.msg"
#   "msg/traffic_light_recognition/TrafficLightRoi.msg"
#   "msg/traffic_light_recognition/TrafficLightRoiArray.msg"
#   "msg/traffic_light_recognition/TrafficLightState.msg"
#   "msg/traffic_light_recognition/TrafficLightStateArray.msg"
#   "msg/traffic_light_recognition/TrafficLightStateStamped.msg"
#   DEPENDENCIES std_msgs geometry_msgs sensor_msgs uuid_msgs
# )

ament_package()
