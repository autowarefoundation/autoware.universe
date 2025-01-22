// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE_PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_HPP_
#define AUTOWARE_PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_HPP_

// since ASSERT_NO_THROW in gtest masks the exception message, redefine it.
#define ASSERT_NO_THROW_WITH_ERROR_MSG(statement)                                                \
  try {                                                                                          \
    statement;                                                                                   \
    SUCCEED();                                                                                   \
  } catch (const std::exception & e) {                                                           \
    FAIL() << "Expected: " << #statement                                                         \
           << " doesn't throw an exception.\nActual: it throws. Error message: " << e.what()     \
           << std::endl;                                                                         \
  } catch (...) {                                                                                \
    FAIL() << "Expected: " << #statement                                                         \
           << " doesn't throw an exception.\nActual: it throws. Error message is not available." \
           << std::endl;                                                                         \
  }

#include <autoware/component_interface_specs_universe/planning.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ctime>
#include <memory>
#include <string>

namespace autoware::planning_test_manager
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;
using tf2_msgs::msg::TFMessage;
using tier4_planning_msgs::msg::PathWithLaneId;
using tier4_planning_msgs::msg::Scenario;
using tier4_planning_msgs::msg::VelocityLimit;

enum class ModuleName {
  UNKNOWN = 0,
  START_PLANNER,
};

class PlanningInterfaceTestManager
{
public:
  PlanningInterfaceTestManager();

  void publishOdometry(
    rclcpp::Node::SharedPtr target_node, std::string topic_name, const double shift = 0.0);

  void publishInitialPose(
    rclcpp::Node::SharedPtr target_node, std::string topic_name, const double shift = 0.0,
    ModuleName module_name = ModuleName::UNKNOWN);

  void publishMaxVelocity(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishPointCloud(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishAcceleration(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishPredictedObjects(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishOccupancyGrid(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishCostMap(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishMap(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishLaneDrivingScenario(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishParkingScenario(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishParkingState(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishTrajectory(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishRoute(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishTF(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishInitialPoseTF(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishOperationModeState(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishTrafficSignals(rclcpp::Node::SharedPtr target_node, std::string topic_name);

  void setTrajectoryInputTopicName(std::string topic_name);
  void setParkingTrajectoryInputTopicName(std::string topic_name);
  void setLaneDrivingTrajectoryInputTopicName(std::string topic_name);
  void setRouteInputTopicName(std::string topic_name);
  void setPathInputTopicName(std::string topic_name);
  void setPathWithLaneIdTopicName(std::string topic_name);
  void setPathTopicName(std::string topic_name);

  void setTrajectorySubscriber(std::string topic_name);
  void setScenarioSubscriber(std::string topic_name);
  void setPathWithLaneIdSubscriber(std::string topic_name);
  void setRouteSubscriber(std::string topic_name);
  void setPathSubscriber(std::string topic_name);

  void setInitialPoseTopicName(std::string topic_name);
  void setOdometryTopicName(std::string topic_name);

  void testWithNominalTrajectory(rclcpp::Node::SharedPtr target_node);
  void testWithAbnormalTrajectory(rclcpp::Node::SharedPtr target_node);

  void testWithNominalRoute(rclcpp::Node::SharedPtr target_node);
  void testWithAbnormalRoute(rclcpp::Node::SharedPtr target_node);

  void testWithBehaviorNominalRoute(
    rclcpp::Node::SharedPtr target_node, ModuleName module_name = ModuleName::UNKNOWN);

  void testWithNominalPathWithLaneId(rclcpp::Node::SharedPtr target_node);
  void testWithAbnormalPathWithLaneId(rclcpp::Node::SharedPtr target_node);

  void testWithNominalPath(rclcpp::Node::SharedPtr target_node);
  void testWithAbnormalPath(rclcpp::Node::SharedPtr target_node);

  // for invalid ego poses, contains some tests inside.
  void testRouteWithInvalidEgoPose(rclcpp::Node::SharedPtr target_node);
  void testPathWithInvalidEgoPose(rclcpp::Node::SharedPtr target_node);
  void testPathWithLaneIdWithInvalidEgoPose(rclcpp::Node::SharedPtr target_node);
  void testTrajectoryWithInvalidEgoPose(rclcpp::Node::SharedPtr target_node);

  // ego vehicle is located far from trajectory
  void testOffTrackFromRoute(rclcpp::Node::SharedPtr target_node);
  void testOffTrackFromPath(rclcpp::Node::SharedPtr target_node);
  void testOffTrackFromPathWithLaneId(rclcpp::Node::SharedPtr target_node);
  void testOffTrackFromTrajectory(rclcpp::Node::SharedPtr target_node);

  int getReceivedTopicNum();
  rclcpp::Node::SharedPtr getTestNode() const;

private:
  // Publisher (necessary for node running)
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr max_velocity_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr acceleration_pub_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr predicted_objects_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr cost_map_pub_;
  rclcpp::Publisher<LaneletMapBin>::SharedPtr map_pub_;
  rclcpp::Publisher<Scenario>::SharedPtr scenario_pub_;
  rclcpp::Publisher<Scenario>::SharedPtr parking_scenario_pub_;
  rclcpp::Publisher<Scenario>::SharedPtr lane_driving_scenario_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parking_state_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<LaneletRoute>::SharedPtr route_pub_;
  rclcpp::Publisher<TFMessage>::SharedPtr TF_pub_;
  rclcpp::Publisher<TFMessage>::SharedPtr initial_pose_tf_pub_;
  rclcpp::Publisher<OperationModeState>::SharedPtr operation_mode_state_pub_;
  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr traffic_signals_pub_;

  // Subscriber
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr route_sub_;
  rclcpp::Subscription<Scenario>::SharedPtr scenario_sub_;
  rclcpp::Subscription<PathWithLaneId>::SharedPtr path_with_lane_id_sub_;
  rclcpp::Subscription<Path>::SharedPtr path_sub_;

  // Publisher for testing(trajectory)
  rclcpp::Publisher<Trajectory>::SharedPtr normal_trajectory_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr abnormal_trajectory_pub_;

  // Publisher for testing(route)
  rclcpp::Publisher<LaneletRoute>::SharedPtr normal_route_pub_;
  rclcpp::Publisher<LaneletRoute>::SharedPtr abnormal_route_pub_;

  // Publisher for testing(route)
  rclcpp::Publisher<LaneletRoute>::SharedPtr behavior_normal_route_pub_;

  // Publisher for testing(PathWithLaneId)
  rclcpp::Publisher<PathWithLaneId>::SharedPtr normal_path_with_lane_id_pub_;
  rclcpp::Publisher<PathWithLaneId>::SharedPtr abnormal_path_with_lane_id_pub_;

  // Publisher for testing(Path)
  rclcpp::Publisher<Path>::SharedPtr normal_path_pub_;
  rclcpp::Publisher<Path>::SharedPtr abnormal_path_pub_;

  std::string input_trajectory_name_ = "";
  std::string input_parking_trajectory_name_ = "";
  std::string input_lane_driving_trajectory_name_ = "";
  std::string input_route_name_ = "";
  std::string input_path_name_ = "";
  std::string input_path_with_lane_id_name_ = "";
  std::string input_initial_pose_name_ = "";  // for the map based pose
  std::string input_odometry_name_ = "";

  // Node
  rclcpp::Node::SharedPtr test_node_;

  std::string map_frame_ = "map";
  size_t count_{0};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  void publishNominalTrajectory(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishAbnormalTrajectory(
    rclcpp::Node::SharedPtr target_node, const Trajectory & abnormal_trajectory);

  void publishNominalRoute(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishAbnormalRoute(
    rclcpp::Node::SharedPtr target_node, const LaneletRoute & abnormal_route);

  void publishBehaviorNominalRoute(
    rclcpp::Node::SharedPtr target_node, std::string topic_name,
    ModuleName module_name = ModuleName::UNKNOWN);
  void publishNominalPathWithLaneId(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishAbNominalPathWithLaneId(rclcpp::Node::SharedPtr target_node, std::string topic_name);

  void publishNominalPath(rclcpp::Node::SharedPtr target_node, std::string topic_name);
  void publishAbnormalPath(rclcpp::Node::SharedPtr target_node, std::string topic_name);
};  // class PlanningInterfaceTestManager

}  // namespace autoware::planning_test_manager

#endif  // AUTOWARE_PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_HPP_
