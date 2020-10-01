project(autoware_perception_msgs)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

set(OBJECT_RECOGNITION_MSG_FILES
  "DynamicObject.msg"
  "DynamicObjectArray.msg"
  "DynamicObjectWithFeature.msg"
  "DynamicObjectWithFeatureArray.msg"
  "Feature.msg"
  "PredictedPath.msg"
  "Semantic.msg"
  "Shape.msg"
  "State.msg"
)

set(TRAFFIC_LIGHT_RECOGNITION_MSG_FILES
  "LampState.msg"
  "TrafficLightRoi.msg"
  "TrafficLightRoiArray.msg"
  "TrafficLightState.msg"
  "TrafficLightStateArray.msg"
)

if(${ROS_VERSION} EQUAL 1)

  cmake_minimum_required(VERSION 3.0.2)

  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  set(UUID_MSG_TYPE uuid_msgs/UniqueID)
  configure_file(msg/object_recognition/DynamicObject.msg.in
    ${CMAKE_CURRENT_SOURCE_DIR}/msg/object_recognition/DynamicObject.msg
  )

  find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    sensor_msgs
    uuid_msgs
  )

  ## Generate messages in the 'msg' folder
  add_message_files(FILES
    ${OBJECT_RECOGNITION_MSG_FILES}
    DIRECTORY msg/object_recognition
  )

  add_message_files(FILES
    ${TRAFFIC_LIHIT_RECOGNITION_MSG_FILES}
    DIRECTORY msg/traffic_light_recognition
  )

  ## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
      std_msgs
      geometry_msgs
      sensor_msgs
      uuid_msgs
  )

  catkin_package(
    CATKIN_DEPENDS message_runtime
  )

elseif(${ROS_VERSION} EQUAL 2)

  cmake_minimum_required(VERSION 3.5)

  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  set(UUID_MSG_TYPE "unique_identifier_msgs/UUID")
  configure_file(msg/object_recognition/DynamicObject.msg.in
    ${CMAKE_CURRENT_SOURCE_DIR}/msg/object_recognition/DynamicObject.msg
  )

  find_package(ament_cmake REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(unique_identifier_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  # Append "msg/" to each file name
  set(TEMP_LIST "")
  foreach(MSG_FILE ${OBJECT_RECOGNITION_MSG_FILES})
    list(APPEND TEMP_LIST "msg/object_recognition/${MSG_FILE}")
  endforeach()

  foreach(MSG_FILE ${TRAFFIC_LIGHT_RECOGNITION_MSG_FILES})
    list(APPEND TEMP_LIST "msg/traffic_light_recognition/${MSG_FILE}")
  endforeach()
  set(MSG_FILES ${TEMP_LIST})

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${MSG_FILES}
    DEPENDENCIES
      builtin_interfaces
      std_msgs
      geometry_msgs
      sensor_msgs
      unique_identifier_msgs
    ADD_LINTER_TESTS
  )

  ament_export_dependencies(rosidl_default_runtime)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()

endif()
