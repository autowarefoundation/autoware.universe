// Copyright 2023 The Autoware Foundation
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

#ifndef PLANNING_INTERFACE_TEST_MANAGER__PLANNING_INTERFACE_TEST_MANAGER_UTILS_HPP_
#define PLANNING_INTERFACE_TEST_MANAGER__PLANNING_INTERFACE_TEST_MANAGER_UTILS_HPP_

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

#include <component_interface_specs/planning.hpp>
#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/optional.hpp>

#include <lanelet2_io/Io.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace test_utils
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using planning_interface::Route;
using tf2_msgs::msg::TFMessage;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::createQuaternionFromRPY;
using tier4_planning_msgs::msg::Scenario;
using unique_identifier_msgs::msg::UUID;

geometry_msgs::msg::Pose createPose(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position = createPoint(x, y, z);
  p.orientation = createQuaternionFromRPY(roll, pitch, yaw);
  return p;
}

template <class T>
T generateTrajectory(
  const size_t num_points, const double point_interval, const double velocity = 0.0,
  const double init_theta = 0.0, const double delta_theta = 0.0,
  const size_t overlapping_point_index = std::numeric_limits<size_t>::max())
{
  using Point = typename T::_points_type::value_type;

  T traj;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    Point p;
    p.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.longitudinal_velocity_mps = velocity;
    traj.points.push_back(p);

    if (i == overlapping_point_index) {
      Point value_to_insert = traj.points[overlapping_point_index];
      traj.points.insert(traj.points.begin() + overlapping_point_index + 1, value_to_insert);
    }
  }

  return traj;
}

geometry_msgs::msg::Pose createPose(const std::array<double, 4> & pose3d)
{
  return createPose(pose3d[0], pose3d[1], pose3d[2], 0.0, 0.0, pose3d[3]);
}

Route::Message makeNormalRoute()
{
  const double pi = 3.1415926;
  const std::array<double, 4> start_pose{5.5, 4., 0., pi * 0.5};
  const std::array<double, 4> goal_pose{8.0, 26.3, 0, 0};
  Route::Message route;
  route.header.frame_id = "map";
  route.start_pose = createPose(start_pose);
  route.goal_pose = createPose(goal_pose);
  return route;
}

LaneletSegment createLaneletSegment(int id)
{
  LaneletPrimitive primitive;
  primitive.id = id;
  primitive.primitive_type = "lane";
  LaneletSegment segment;
  segment.preferred_primitive.id = id;
  segment.primitives.push_back(primitive);
  return segment;
}

Route::Message makeBehaviorNormalRoute()
{
  Route::Message route;
  route.header.frame_id = "map";
  route.start_pose =
    createPose({3722.16015625, 73723.515625, 0.233112560494183, 0.9724497591854532});
  route.goal_pose =
    createPose({3778.362060546875, 73721.2734375, -0.5107480274693206, 0.8597304533609347});

  std::vector<int> primitive_ids = {9102, 9178, 54, 112};
  for (int id : primitive_ids) {
    route.segments.push_back(createLaneletSegment(id));
  }

  std::array<uint8_t, 16> uuid_bytes{210, 87,  16,  126, 98,  151, 58, 28,
                                     252, 221, 230, 92,  122, 170, 46, 6};
  route.uuid.uuid = uuid_bytes;

  route.allow_modification = false;
  return route;
}

OccupancyGrid constructCostMap(size_t width, size_t height, double resolution)
{
  nav_msgs::msg::OccupancyGrid costmap_msg{};

  // create info
  costmap_msg.info.width = width;
  costmap_msg.info.height = height;
  costmap_msg.info.resolution = resolution;

  // create data
  const size_t n_elem = width * height;
  for (size_t i = 0; i < n_elem; ++i) {
    costmap_msg.data.push_back(0.0);
  }
  return costmap_msg;
}

HADMapBin constructCostMap(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename, const rclcpp::Time & now)
{
  std::string format_version{}, map_version{};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  HADMapBin map_bin_msg;
  map_bin_msg.header.stamp = now;
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.format_version = format_version;
  map_bin_msg.map_version = map_version;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  return map_bin_msg;
}

void spinSomeNodes(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node,
  const int repeat_count = 1)
{
  for (int i = 0; i < repeat_count; i++) {
    rclcpp::spin_some(test_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(target_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
}

lanelet::LaneletMapPtr load_map(const std::string & lanelet2_filename)
{
  lanelet::ErrorMessages errors{};
  lanelet::projection::MGRSProjector projector{};
  const lanelet::LaneletMapPtr map = lanelet::load(lanelet2_filename, projector, &errors);
  if (errors.empty()) {
    return map;
  }

  for (const auto & error : errors) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("map_loader"), error);
  }
  return nullptr;
}

HADMapBin constructCostMap(
  const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename, const rclcpp::Time & now)
{
  std::string format_version{}, map_version{};
  lanelet::io_handlers::AutowareOsmParser::parseVersions(
    lanelet2_filename, &format_version, &map_version);

  HADMapBin map_bin_msg;
  map_bin_msg.header.stamp = now;
  map_bin_msg.header.frame_id = "map";
  map_bin_msg.format_version = format_version;
  map_bin_msg.map_version = map_version;
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  return map_bin_msg;
}

template <typename T>
void setPublisher(
  rclcpp::Node::SharedPtr test_node, std::string topic_name,
  std::shared_ptr<rclcpp::Publisher<T>> & publisher)
{
  publisher = rclcpp::create_publisher<T>(test_node, topic_name, 1);
}

template <>
void setPublisher(
  rclcpp::Node::SharedPtr test_node, std::string topic_name,
  std::shared_ptr<rclcpp::Publisher<LaneletRoute>> & publisher)
{
  rclcpp::QoS custom_qos_profile{rclcpp::KeepLast(1)};
  custom_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  custom_qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  publisher = rclcpp::create_publisher<LaneletRoute>(test_node, topic_name, custom_qos_profile);
}

template <>
void setPublisher(
  rclcpp::Node::SharedPtr test_node, std::string topic_name,
  std::shared_ptr<rclcpp::Publisher<HADMapBin>> & publisher)
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();
  qos.transient_local();
  publisher = rclcpp::create_publisher<HADMapBin>(test_node, topic_name, qos);
}

template <typename T>
void publishData(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, std::string topic_name,
  typename rclcpp::Publisher<T>::SharedPtr publisher)
{
  setPublisher(test_node, topic_name, publisher);
  publisher->publish(T{});
  spinSomeNodes(test_node, target_node);
}

template <>
void publishData<HADMapBin>(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, std::string topic_name,
  typename rclcpp::Publisher<HADMapBin>::SharedPtr publisher)
{
  setPublisher(test_node, topic_name, publisher);
  const auto planning_test_utils_dir =
    ament_index_cpp::get_package_share_directory("planning_test_utils");

  const auto lanelet2_path = planning_test_utils_dir + "/map/lanelet2_map.osm";
  double center_line_resolution = 5.0;
  // load map from file
  const auto map = load_map(lanelet2_path);
  if (!map) {
    return;
  }

  // overwrite centerline
  lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution, false);

  // create map bin msg
  const auto map_bin_msg = constructCostMap(map, lanelet2_path, rclcpp::Clock(RCL_ROS_TIME).now());
  publisher->publish(map_bin_msg);
  spinSomeNodes(test_node, target_node);
}

template <>
void publishData<Trajectory>(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, std::string topic_name,
  typename rclcpp::Publisher<Trajectory>::SharedPtr publisher)
{
  setPublisher(test_node, topic_name, publisher);
  std::shared_ptr<Trajectory> trajectory = std::make_shared<Trajectory>();
  trajectory->header.stamp = target_node->now();
  publisher->publish(*trajectory);
  spinSomeNodes(test_node, target_node);
}

template <>
void publishData<Odometry>(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, std::string topic_name,
  typename rclcpp::Publisher<Odometry>::SharedPtr publisher)
{
  setPublisher(test_node, topic_name, publisher);

  std::shared_ptr<Odometry> current_odometry = std::make_shared<Odometry>();
  const double pi = 3.1415926;
  const std::array<double, 4> start_pose{5.5, 4., pi * 0.5};
  current_odometry->pose.pose = create_pose_msg(start_pose);
  current_odometry->header.frame_id = "map";
  publisher->publish(*current_odometry);
  spinSomeNodes(test_node, target_node);
}

template <>
void publishData<OccupancyGrid>(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, std::string topic_name,
  typename rclcpp::Publisher<OccupancyGrid>::SharedPtr publisher)
{
  setPublisher(test_node, topic_name, publisher);
  std::shared_ptr<OccupancyGrid> current_occupancy_grid = std::make_shared<OccupancyGrid>();
  auto costmap_msg = constructCostMap(150, 150, 0.2);
  costmap_msg.header.frame_id = "map";
  publisher->publish(costmap_msg);
  spinSomeNodes(test_node, target_node);
}

template <>
void publishData<LaneletRoute>(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, std::string topic_name,
  typename rclcpp::Publisher<LaneletRoute>::SharedPtr publisher)
{
  setPublisher(test_node, topic_name, publisher);
  publisher->publish(makeNormalRoute());
  spinSomeNodes(test_node, target_node);
}

template <>
void publishData<TFMessage>(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, std::string topic_name,
  typename rclcpp::Publisher<TFMessage>::SharedPtr publisher)
{
  TransformStamped tf;
  tf.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  tf.header.frame_id = "base_link";
  tf.child_frame_id = "map";

  TFMessage tf_msg{};
  tf_msg.transforms.emplace_back(std::move(tf));
  setPublisher(test_node, topic_name, publisher);
  publisher->publish(tf_msg);
  spinSomeNodes(test_node, target_node);
}

void publishScenarioData(
  rclcpp::Node::SharedPtr test_node, rclcpp::Node::SharedPtr target_node, std::string topic_name,
  rclcpp::Publisher<Scenario>::SharedPtr publisher, const std::string scenario)
{
  auto scenario_msg = std::make_shared<Scenario>();
  scenario_msg->current_scenario = scenario;
  scenario_msg->activating_scenarios = {scenario};
  setPublisher(test_node, topic_name, publisher);
  publisher->publish(*scenario_msg);

  spinSomeNodes(test_node, target_node);
}

template <typename T>
void setSubscriber(
  rclcpp::Node::SharedPtr test_node, std::string topic_name,
  std::shared_ptr<rclcpp::Subscription<T>> & subscriber, size_t & count)
{
  // Count the number of topic received.
  subscriber = test_node->create_subscription<T>(
    topic_name, 10, [&count](const typename T::SharedPtr) { count++; });
}

template <>
void setSubscriber<Trajectory>(
  rclcpp::Node::SharedPtr test_node, std::string topic_name,
  std::shared_ptr<rclcpp::Subscription<Trajectory>> & subscriber, size_t & count)
{
  subscriber = test_node->create_subscription<Trajectory>(
    topic_name, rclcpp::QoS{1},
    [&count](const typename Trajectory::SharedPtr msg [[maybe_unused]]) { count++; });
}

}  // namespace test_utils

#endif  // PLANNING_INTERFACE_TEST_MANAGER__PLANNING_INTERFACE_TEST_MANAGER_UTILS_HPP_
