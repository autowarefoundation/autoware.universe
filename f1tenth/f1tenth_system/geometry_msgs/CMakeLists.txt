cmake_minimum_required(VERSION 3.5)

project(geometry_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/Accel.msg"
  "msg/AccelStamped.msg"
  "msg/AccelWithCovariance.msg"
  "msg/AccelWithCovarianceStamped.msg"
  "msg/Inertia.msg"
  "msg/InertiaStamped.msg"
  "msg/Point.msg"
  "msg/Point32.msg"
  "msg/PointStamped.msg"
  "msg/Polygon.msg"
  "msg/PolygonStamped.msg"
  "msg/Pose.msg"
  "msg/Pose2D.msg"
  "msg/PoseArray.msg"
  "msg/PoseStamped.msg"
  "msg/PoseWithCovariance.msg"
  "msg/PoseWithCovarianceStamped.msg"
  "msg/Quaternion.msg"
  "msg/QuaternionStamped.msg"
  "msg/Transform.msg"
  "msg/TransformStamped.msg"
  "msg/Twist.msg"
  "msg/TwistStamped.msg"
  "msg/TwistWithCovariance.msg"
  "msg/TwistWithCovarianceStamped.msg"
  "msg/Vector3.msg"
  "msg/Vector3Stamped.msg"
  "msg/Wrench.msg"
  "msg/WrenchStamped.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
