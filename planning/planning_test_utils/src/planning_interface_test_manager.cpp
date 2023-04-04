// Copyright 2023 Tier IV, Inc.
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

#include <planning_interface_test_manager/planning_interface_test_manager.hpp>
#include <planning_interface_test_manager/planning_interface_test_manager_utils.hpp>

namespace planning_test_utils
{

PlanningIntefaceTestManager::PlanningIntefaceTestManager()
{
  test_node_ = std::make_shared<rclcpp::Node>("planning_interface_test_node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(test_node_->get_clock());
}

void PlanningIntefaceTestManager::declareVehicleInfoParams(rclcpp::NodeOptions & node_options)
{
  // for vehicle info
  node_options.append_parameter_override("wheel_radius", 0.5);
  node_options.append_parameter_override("wheel_width", 0.2);
  node_options.append_parameter_override("wheel_base", 3.0);
  node_options.append_parameter_override("wheel_tread", 2.0);
  node_options.append_parameter_override("front_overhang", 1.0);
  node_options.append_parameter_override("rear_overhang", 1.0);
  node_options.append_parameter_override("left_overhang", 0.5);
  node_options.append_parameter_override("right_overhang", 0.5);
  node_options.append_parameter_override("vehicle_height", 1.5);
  node_options.append_parameter_override("max_steer_angle", 0.7);
}

void PlanningIntefaceTestManager::declareNearestSearchDistanceParams(
  rclcpp::NodeOptions & node_options)
{
  node_options.append_parameter_override("ego_nearest_dist_threshold", 3.0);
  node_options.append_parameter_override("ego_nearest_yaw_threshold", 1.046);
}

void PlanningIntefaceTestManager::publishOdometry(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<Odometry>(test_node_, target_node, topic_name, odom_pub_);
}

void PlanningIntefaceTestManager::publishMaxVelocity(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<VelocityLimit>(test_node_, target_node, topic_name, max_velocity_pub_);
}

void PlanningIntefaceTestManager::publishPointCloud(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<PointCloud2>(test_node_, target_node, topic_name, point_cloud_pub_);
}

void PlanningIntefaceTestManager::publishAcceleration(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<AccelWithCovarianceStamped>(
    test_node_, target_node, topic_name, acceleration_pub_);
}

void PlanningIntefaceTestManager::publishPredictedObjects(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<PredictedObjects>(
    test_node_, target_node, topic_name, predicted_objects_pub_);
}

void PlanningIntefaceTestManager::publishExpandStopRange(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<ExpandStopRange>(
    test_node_, target_node, topic_name, expand_stop_range_pub_);
}

void PlanningIntefaceTestManager::publishOccupancyGrid(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<OccupancyGrid>(test_node_, target_node, topic_name, occupancy_grid_pub_);
}

void PlanningIntefaceTestManager::publishCostMap(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<OccupancyGrid>(test_node_, target_node, topic_name, cost_map_pub_);
}

void PlanningIntefaceTestManager::publishMap(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<HADMapBin>(test_node_, target_node, topic_name, map_pub_);
}

void PlanningIntefaceTestManager::publishLaneDrivingScenario(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishScenarioData(
    test_node_, target_node, topic_name, lane_driving_scenario_pub_, Scenario::LANEDRIVING);
}

void PlanningIntefaceTestManager::publishParkingScenario(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishScenarioData(
    test_node_, target_node, topic_name, parking_scenario_pub_, Scenario::PARKING);
}

void PlanningIntefaceTestManager::publishParkingState(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<std_msgs::msg::Bool>(
    test_node_, target_node, topic_name, parking_state_pub_);
}

void PlanningIntefaceTestManager::publishTrajectory(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<Trajectory>(test_node_, target_node, topic_name, trajectory_pub_);
}

void PlanningIntefaceTestManager::publishRoute(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<LaneletRoute>(test_node_, target_node, topic_name, route_pub_);
}

void PlanningIntefaceTestManager::publishTF(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<TFMessage>(test_node_, target_node, topic_name, TF_pub_);
}

void PlanningIntefaceTestManager::publishLateralOffset(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<LateralOffset>(test_node_, target_node, topic_name, lateral_offset_pub_);
}

void PlanningIntefaceTestManager::publishOperationModeState(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::publishData<OperationModeState>(
    test_node_, target_node, topic_name, operation_mode_state_pub_);
}

void PlanningIntefaceTestManager::setTrajectoryInputTopicName(std::string topic_name)
{
  input_trajectory_name_ = topic_name;
}

void PlanningIntefaceTestManager::setParkingTrajectoryInputTopicName(std::string topic_name)
{
  input_parking_trajectory_name_ = topic_name;
}

void PlanningIntefaceTestManager::setLaneDrivingTrajectoryInputTopicName(std::string topic_name)
{
  input_lane_driving_trajectory_name_ = topic_name;
}

void PlanningIntefaceTestManager::setRouteInputTopicName(std::string topic_name)
{
  input_route_name_ = topic_name;
}

void PlanningIntefaceTestManager::publishNominalTrajectory(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::setPublisher(test_node_, topic_name, normal_trajectory_pub_);
  normal_trajectory_pub_->publish(test_utils::generateTrajectory<Trajectory>(10, 1.0));
  test_utils::spinSomeNodes(test_node_, target_node);
}

void PlanningIntefaceTestManager::publishNominalRoute(
  rclcpp::Node::SharedPtr target_node, std::string topic_name)
{
  test_utils::setPublisher(test_node_, topic_name, normal_route_pub_);
  normal_route_pub_->publish(test_utils::makeNormalRoute());
  test_utils::spinSomeNodes(test_node_, target_node);
}

void PlanningIntefaceTestManager::setTrajectorySubscriber(std::string topic_name)
{
  test_utils::setSubscriber(test_node_, topic_name, traj_sub_, count_);
}

void PlanningIntefaceTestManager::setRouteSubscriber(std::string topic_name)
{
  test_utils::setSubscriber(test_node_, topic_name, route_sub_, count_);
}
void PlanningIntefaceTestManager::setScenarioSubscriber(std::string topic_name)
{
  test_utils::setSubscriber(test_node_, topic_name, scenario_sub_, count_);
}

void PlanningIntefaceTestManager::setPathWithLaneIdSubscriber(std::string topic_name)
{
  test_utils::setSubscriber(test_node_, topic_name, path_with_lane_id_sub_, count_);
}

// test for normal working
void PlanningIntefaceTestManager::testWithNominalTrajectory(rclcpp::Node::SharedPtr target_node)
{
  publishNominalTrajectory(target_node, input_trajectory_name_);
  test_utils::spinSomeNodes(test_node_, target_node, 2);
}

// check to see if target node is dead.
void PlanningIntefaceTestManager::testWithAbnormalTrajectory(rclcpp::Node::SharedPtr target_node)
{
  ASSERT_NO_THROW(publishAbnormalTrajectory(target_node, Trajectory{}));
  ASSERT_NO_THROW(
    publishAbnormalTrajectory(target_node, test_utils::generateTrajectory<Trajectory>(1, 0.0)));
  ASSERT_NO_THROW(publishAbnormalTrajectory(
    target_node, test_utils::generateTrajectory<Trajectory>(10, 0.0, 0.0, 0.0, 0.0, 1)));
}

void PlanningIntefaceTestManager::publishAbnormalTrajectory(
  rclcpp::Node::SharedPtr target_node, const Trajectory & abnormal_trajectory)
{
  test_utils::setPublisher(test_node_, input_trajectory_name_, abnormal_trajectory_pub_);
  abnormal_trajectory_pub_->publish(abnormal_trajectory);
  test_utils::spinSomeNodes(test_node_, target_node);
}

// test for normal working
void PlanningIntefaceTestManager::testWithNominalRoute(rclcpp::Node::SharedPtr target_node)
{
  publishNominalRoute(target_node, input_route_name_);
  test_utils::spinSomeNodes(test_node_, target_node, 5);
}

// check to see if target node is dead.
void PlanningIntefaceTestManager::testWithAbnormalRoute(rclcpp::Node::SharedPtr target_node)
{
  ASSERT_NO_THROW(publishAbnormalRoute(target_node, LaneletRoute{}));
  // ASSERT_NO_THROW(
  //   publishAbnormalRoute(target_node, test_utils::generateRoute<LaneletRoute>(1, 0.0)));
  // ASSERT_NO_THROW(publishAbnormalRoute(
  //   target_node, test_utils::generateRoute<LaneletRoute>(10, 0.0, 0.0, 0.0, 0.0, 1)));
}

void PlanningIntefaceTestManager::publishAbnormalRoute(
  rclcpp::Node::SharedPtr target_node, const LaneletRoute & abnormal_route)
{
  test_utils::setPublisher(test_node_, input_route_name_, abnormal_route_pub_);
  abnormal_route_pub_->publish(abnormal_route);
  test_utils::spinSomeNodes(test_node_, target_node);
}

int PlanningIntefaceTestManager::getReceivedTopicNum()
{
  return count_;
}

}  // namespace planning_test_utils
