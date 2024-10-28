// Copyright 2024 TIER IV, Inc.
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

#include <autoware/behavior_path_planner_common/data_manager.hpp>
#include <autoware/behavior_path_planner_common/planner_data_yaml.hpp>
#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <std_srvs/srv/empty.hpp>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>

class PlannerDataServer : public rclcpp::Node
{
public:
  PlannerDataServer() : Node("planner_data_server")
  {
    server_ = this->create_service<std_srvs::srv::Empty>(
      "/planning/planner_data_server",
      std::bind(
        &PlannerDataServer::on_service, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void take_data()
  {
    // odometry
    {
      const auto msg = velocity_subscriber_.takeData();
      if (msg) {
        self_odometry_ = msg;
      }
    }

    // acceleration
    {
      const auto msg = acceleration_subscriber_.takeData();
      if (msg) {
        self_acceleration_ = msg;
      }
    }

    // perception
    {
      const auto msg = perception_subscriber_.takeData();
      if (msg) {
        dynamic_object_ = msg;
      }
    }

    // occupancy_grid
    {
      const auto msg = occupancy_grid_subscriber_.takeData();
      if (msg) {
        occupancy_grid_ = msg;
      }
    }

    // costmap
    {
      const auto msg = costmap_subscriber_.takeData();
      if (msg) {
        costmap_ = msg;
      }
    }

    // lateral_offset
    {
      const auto msg = lateral_offset_subscriber_.takeData();
      if (msg) {
        lateral_offset_ = msg;
      }
    }

    // operation_mode
    {
      const auto msg = operation_mode_subscriber_.takeData();
      if (msg) {
        operation_mode_ = msg;
      }
    }

    // route
    {
      const auto msg = route_subscriber_.takeData();
      if (msg) {
        if (msg->segments.empty()) {
          RCLCPP_ERROR(this->get_logger(), "input route is empty, ignore");
        }
        route_ptr_ = msg;
      }
    }

    // traffic_signal
    {
      const auto msg = traffic_signals_subscriber_.takeData();
      if (msg) {
        traffic_signal_ = msg;
      }
    }

    /*
    // external_velocity_limiter
    {
      const auto msg = external_limit_max_velocity_subscriber_.takeData();
      if (msg) {
        planner_data_.external_limit_max_velocity = msg;
      }
    }
    */
  }

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_;

  // subscriber
  nav_msgs::msg::Odometry::ConstSharedPtr self_odometry_;
  autoware::universe_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry>
    velocity_subscriber_{this, "/localization/kinematic_state"};

  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr self_acceleration_;
  autoware::universe_utils::InterProcessPollingSubscriber<
    geometry_msgs::msg::AccelWithCovarianceStamped>
    acceleration_subscriber_{this, "/localization/acceleration"};

  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr dynamic_object_;
  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::PredictedObjects>
    perception_subscriber_{this, "/perception/object_recognition/objects"};

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid_;
  autoware::universe_utils::InterProcessPollingSubscriber<nav_msgs::msg::OccupancyGrid>
    occupancy_grid_subscriber_{this, "/perception/occupancy_grid_map/map"};

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap_;
  autoware::universe_utils::InterProcessPollingSubscriber<nav_msgs::msg::OccupancyGrid>
    costmap_subscriber_{
      this, "/planning/scenario_planning/parking/costmap_generator/occupancy_grid"};

  tier4_planning_msgs::msg::LateralOffset::ConstSharedPtr lateral_offset_;
  autoware::universe_utils::InterProcessPollingSubscriber<tier4_planning_msgs::msg::LateralOffset>
    lateral_offset_subscriber_{
      this,
      "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/input/"
      "lateral_offset"};

  autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr operation_mode_;
  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_adapi_v1_msgs::msg::OperationModeState>
    operation_mode_subscriber_{
      this, "/system/operation_mode/state", rclcpp::QoS{1}.transient_local()};

  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route_ptr_;
  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_planning_msgs::msg::LaneletRoute, autoware::universe_utils::polling_policy::Newest>
    route_subscriber_{this, "/planning/mission_planning/route", rclcpp::QoS{1}.transient_local()};

  /*
  autoware::universe_utils::InterProcessPollingSubscriber<tier4_planning_msgs::msg::Scenario>
    scenario_subscriber_{this, "/planning/scenario_planning/scenario"};
  */

  autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr traffic_signal_;
  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>
    traffic_signals_subscriber_{this, "/perception/traffic_light_recognition/traffic_signals"};

  /*
  autoware::universe_utils::InterProcessPollingSubscriber<tier4_planning_msgs::msg::VelocityLimit>
    external_limit_max_velocity_subscriber_{this, "/planning/scenario_planning/max_velocity"};
  */

  void on_service(
    [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Received planner_data save request");
    take_data();

    if (!self_odometry_) {
      RCLCPP_INFO(this->get_logger(), "self_odometry message is not ready yet, not saving");
      return;
    }

    if (!self_acceleration_) {
      RCLCPP_INFO(this->get_logger(), "self_acceleration message is not ready yet, not saving");
      return;
    }

    if (!dynamic_object_) {
      RCLCPP_INFO(this->get_logger(), "dynamic_object message is not ready yet, not saving");
      return;
    }

    if (!lateral_offset_) {
      RCLCPP_INFO(
        this->get_logger(),
        "lateral offset message is not ready yet, but is is optional, skipping");
    }

    if (!operation_mode_) {
      RCLCPP_INFO(this->get_logger(), "operation_mode message is not ready yet, not saving");
      return;
    }

    if (!route_ptr_) {
      RCLCPP_INFO(this->get_logger(), "route message is not ready yet, not saving");
      return;
    }

    if (!traffic_signal_) {
      RCLCPP_INFO(
        this->get_logger(),
        "traffic_signal message is not ready yet, but it is optional, skipping");
    }

    const auto [planner_data, desc] = autoware::behavior_path_planner::get_planner_data_yaml(
      *self_odometry_, *self_acceleration_, *dynamic_object_, *occupancy_grid_, *costmap_,
      lateral_offset_, *operation_mode_, *route_ptr_, traffic_signal_);

    std::ofstream ofs("planner_data.yaml");
    ofs << desc;
    ofs << planner_data;

    RCLCPP_INFO(this->get_logger(), "saved planner_data");
  };
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  auto node = std::make_shared<PlannerDataServer>();

  exec.add_node(node);

  exec.spin();

  rclcpp::shutdown();

  return 0;
}
