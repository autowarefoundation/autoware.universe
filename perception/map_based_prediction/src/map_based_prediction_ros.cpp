// Copyright 2018-2019 Autoware Foundation
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

// headers in STL
#include <chrono>
#include <cmath>
#include <unordered_map>

#include "unique_identifier_msgs/msg/uuid.hpp"

// headers in ROS
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"

// lanelet
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_core/geometry/BoundingBox.h"
#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_core/geometry/Point.h"
#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/utilities.hpp"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"

// headers in local files
#include "map_based_prediction.hpp"
#include "map_based_prediction_ros.hpp"

std::string toHexString(const unique_identifier_msgs::msg::UUID & id){
  std::stringstream ss;
    for (auto i = 0; i < 16; ++i) {
        ss << std::hex << std::setfill('0') << std::setw(2) << +id.uuid[i];
    }
    return ss.str();
}

bool MapBasedPredictionROS::getClosestLanelets(
  const autoware_perception_msgs::msg::DynamicObject & object,
  const lanelet::LaneletMapPtr & lanelet_map_ptr_, std::vector<lanelet::Lanelet> & closest_lanelets,
  std::string uuid_string)
{
  std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
  lanelet::BasicPoint2d search_point(
    object.state.pose_covariance.pose.position.x, object.state.pose_covariance.pose.position.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 10);
  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  debug_accumulated_time_ += time.count() / (1000.0 * 1000.0);

  if (nearest_lanelets.empty()) {
    return false;
  }

  if (uuid2laneids_.size() == 0 || uuid2laneids_.count(uuid_string) == 0) {
    bool is_found_target_closest_lanelet = false;
    const double max_delta_yaw_threshold = M_PI / 4.;
    const double max_dist_for_searching_lanelet = 3;
    lanelet::Lanelet target_closest_lanelet;
    for (const auto & lanelet : nearest_lanelets) {
      double object_yaw = 0;
      if (object.state.orientation_reliable) {
        object_yaw = tf2::getYaw(object.state.pose_covariance.pose.orientation);
      } else {
        geometry_msgs::msg::Pose object_frame_pose;
        geometry_msgs::msg::Pose map_frame_pose;
        object_frame_pose.position.x = object.state.twist_covariance.twist.linear.x * 0.1;
        object_frame_pose.position.y = object.state.twist_covariance.twist.linear.y * 0.1;
        tf2::Transform tf_object2future;
        tf2::Transform tf_map2object;
        tf2::Transform tf_map2future;

        tf2::fromMsg(object.state.pose_covariance.pose, tf_map2object);
        tf2::fromMsg(object_frame_pose, tf_object2future);
        tf_map2future = tf_map2object * tf_object2future;
        tf2::toMsg(tf_map2future, map_frame_pose);
        double dx = map_frame_pose.position.x - object.state.pose_covariance.pose.position.x;
        double dy = map_frame_pose.position.y - object.state.pose_covariance.pose.position.y;
        object_yaw = std::atan2(dy, dx);
      }

      if (lanelet.second.centerline().size() <= 1) {
        continue;
      }
      double lane_yaw =
        lanelet::utils::getLaneletAngle(lanelet.second, object.state.pose_covariance.pose.position);
      double delta_yaw = object_yaw - lane_yaw;
      double normalized_delta_yaw = std::atan2(std::sin(delta_yaw), std::cos(delta_yaw));
      double abs_norm_delta = std::fabs(normalized_delta_yaw);
      if (
        lanelet.first < max_dist_for_searching_lanelet &&
        abs_norm_delta < max_delta_yaw_threshold) {
        target_closest_lanelet = lanelet.second;
        is_found_target_closest_lanelet = true;
        closest_lanelets.push_back(target_closest_lanelet);
      }
    }
    if (is_found_target_closest_lanelet) {
      return true;
    }
  } else {
    bool is_found_target_closest_lanelet = false;
    const double max_delta_yaw_threshold = M_PI / 4.;
    const double max_dist_for_searching_lanelet = 3;
    lanelet::Lanelet target_closest_lanelet;
    for (const auto & laneid : uuid2laneids_.at(uuid_string)) {
      for (const auto & lanelet : nearest_lanelets) {
        if (laneid != lanelet.second.id()) {
          continue;
        }
        double object_yaw = 0;
        if (object.state.orientation_reliable) {
          object_yaw = tf2::getYaw(object.state.pose_covariance.pose.orientation);
        } else {
          geometry_msgs::msg::Pose object_frame_pose;
          geometry_msgs::msg::Pose map_frame_pose;
          object_frame_pose.position.x = object.state.twist_covariance.twist.linear.x * 0.1;
          object_frame_pose.position.y = object.state.twist_covariance.twist.linear.y * 0.1;
          tf2::Transform tf_object2future;
          tf2::Transform tf_map2object;
          tf2::Transform tf_map2future;

          tf2::fromMsg(object.state.pose_covariance.pose, tf_map2object);
          tf2::fromMsg(object_frame_pose, tf_object2future);
          tf_map2future = tf_map2object * tf_object2future;
          tf2::toMsg(tf_map2future, map_frame_pose);
          double dx = map_frame_pose.position.x - object.state.pose_covariance.pose.position.x;
          double dy = map_frame_pose.position.y - object.state.pose_covariance.pose.position.y;
          object_yaw = std::atan2(dy, dx);
        }

        if (lanelet.second.centerline().size() <= 1) {
          continue;
        }
        double lane_yaw = lanelet::utils::getLaneletAngle(
          lanelet.second, object.state.pose_covariance.pose.position);
        double delta_yaw = object_yaw - lane_yaw;
        double normalized_delta_yaw = std::atan2(std::sin(delta_yaw), std::cos(delta_yaw));
        double abs_norm_delta = std::fabs(normalized_delta_yaw);
        if (
          lanelet.first < max_dist_for_searching_lanelet &&
          abs_norm_delta < max_delta_yaw_threshold) {
          target_closest_lanelet = lanelet.second;
          is_found_target_closest_lanelet = true;
          closest_lanelets.push_back(target_closest_lanelet);
        }
      }
    }
    if (is_found_target_closest_lanelet) {
      return true;
    }
  }

  return false;
}

double calculateDistance(const geometry_msgs::msg::Point & point1, const geometry_msgs::msg::Point & point2)
{
  double dx = point1.x - point2.x;
  double dy = point1.y - point2.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  return distance;
}

MapBasedPredictionROS::MapBasedPredictionROS() 
: Node("map_based_prediction"), interpolating_resolution_(0.5)
{
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(clock);
  tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);
  has_subscribed_map_ = declare_parameter("map_based_prediction/has_subscribed_map", false);
  prediction_time_horizon_ = declare_parameter("prediction_time_horizon", 10.0);
  prediction_sampling_delta_time_ = declare_parameter("prediction_sampling_delta_time", 0.5);
  map_based_prediction_ = std::make_shared<MapBasedPrediction>(
    interpolating_resolution_, prediction_time_horizon_, prediction_sampling_delta_time_);

  sub_objects_ = this->create_subscription<autoware_perception_msgs::msg::DynamicObjectArray>(
    "/perception/object_recognition/tracking/objects", 1, 
    std::bind(&MapBasedPredictionROS::objectsCallback, this, std::placeholders::_1));
  sub_map_ = this->create_subscription<autoware_lanelet2_msgs::msg::MapBin>(
    "/vector_map", 10, 
    std::bind(&MapBasedPredictionROS::mapCallback, this, std::placeholders::_1));

  pub_objects_ = this->create_publisher<autoware_perception_msgs::msg::DynamicObjectArray>("objects", 1);
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("objects_path_markers", 1);
}

void MapBasedPredictionROS::objectsCallback(
  const autoware_perception_msgs::msg::DynamicObjectArray::ConstSharedPtr in_objects)
{
  debug_accumulated_time_ = 0.0;
  std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

  if (!lanelet_map_ptr_) {
    return;
  }

  geometry_msgs::msg::TransformStamped world2map_transform;
  geometry_msgs::msg::TransformStamped map2world_transform;
  geometry_msgs::msg::TransformStamped debug_map2lidar_transform;
  try {
    world2map_transform = tf_buffer_ptr_->lookupTransform(
      "map",                        // target
      in_objects->header.frame_id,  // src
      in_objects->header.stamp, rclcpp::Duration::from_seconds(0.1));
    map2world_transform = tf_buffer_ptr_->lookupTransform(
      in_objects->header.frame_id,  // target
      "map",                        // src
      in_objects->header.stamp, rclcpp::Duration::from_seconds(0.1));
    debug_map2lidar_transform = tf_buffer_ptr_->lookupTransform(
      "base_link",  // target
      "map",        // src
      rclcpp::Time(), rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & ex) {
    return;
  }

  autoware_perception_msgs::msg::DynamicObjectArray tmp_objects_whitout_map;
  tmp_objects_whitout_map.header = in_objects->header;
  DynamicObjectWithLanesArray prediction_input;
  prediction_input.header = in_objects->header;

  for (const auto & object : in_objects->objects) {
    DynamicObjectWithLanes tmp_object;
    tmp_object.object = object;
    if (in_objects->header.frame_id != "map") {
      geometry_msgs::msg::PoseStamped pose_in_map;
      geometry_msgs::msg::PoseStamped pose_orig;
      pose_orig.pose = object.state.pose_covariance.pose;
      tf2::doTransform(pose_orig, pose_in_map, world2map_transform);
      tmp_object.object.state.pose_covariance.pose = pose_in_map.pose;
    }

    if (
      object.semantic.type != autoware_perception_msgs::msg::Semantic::CAR &&
      object.semantic.type != autoware_perception_msgs::msg::Semantic::BUS &&
      object.semantic.type != autoware_perception_msgs::msg::Semantic::TRUCK) {
      tmp_objects_whitout_map.objects.push_back(tmp_object.object);
      continue;
    }

    // generate non redundant lanelet vector
    std::vector<lanelet::Lanelet> start_lanelets;
    geometry_msgs::msg::Point closest_point;
    std::vector<geometry_msgs::msg::Pose> path_points;
    std::vector<geometry_msgs::msg::Pose> second_path_points;
    std::vector<geometry_msgs::msg::Pose> right_path_points;
    std::string uuid_string = toHexString(object.id);
    if (!getClosestLanelets(tmp_object.object, lanelet_map_ptr_, start_lanelets, uuid_string)) {
      geometry_msgs::msg::PointStamped debug_point;
      geometry_msgs::msg::PointStamped point_orig;
      point_orig.point = tmp_object.object.state.pose_covariance.pose.position;
      tf2::doTransform(
        point_orig, debug_point,
        debug_map2lidar_transform);
      tmp_objects_whitout_map.objects.push_back(object);
      continue;
    }

    // lanelet vector
    std::vector<lanelet::ConstLanelet> valid_lanelets;
    for (const auto & start_lanelet : start_lanelets) {
      lanelet::ConstLanelets prev_lanelets = routing_graph_ptr_->previous(start_lanelet);
      bool is_skip = false;
      for (const auto & valid_lanelet : valid_lanelets) {
        if (valid_lanelet.id() == start_lanelet.id()) {
          is_skip = true;
        }
        for (const auto & prev_lanelet : prev_lanelets) {
          if (valid_lanelet.id() == prev_lanelet.id()) {
            is_skip = true;
          }
        }
      }
      if (is_skip) {
        continue;
      }
      valid_lanelets.push_back(start_lanelet);
    }

    lanelet::routing::LaneletPaths paths;
    for (const auto & start_lanelet : valid_lanelets) {
      lanelet::ConstLanelet origin_lanelet;
      origin_lanelet = start_lanelet;

      auto opt_right = routing_graph_ptr_->right(origin_lanelet);
      lanelet::routing::LaneletPaths right_paths;
      if (!!opt_right) {
        right_paths = routing_graph_ptr_->possiblePaths(*opt_right, 20, 0, false);
      }
      auto opt_left = routing_graph_ptr_->left(origin_lanelet);
      lanelet::routing::LaneletPaths left_paths;
      if (!!opt_left) {
        left_paths = routing_graph_ptr_->possiblePaths(*opt_left, 20, 0, false);
      }

      lanelet::routing::LaneletPaths center_paths;
      double delta_sampling_meters = 20;
      for (double minimum_dist_for_route_search = 100; minimum_dist_for_route_search >= 20;
           minimum_dist_for_route_search -= delta_sampling_meters) {
        lanelet::routing::LaneletPaths tmp_paths = routing_graph_ptr_->possiblePaths(
          origin_lanelet, minimum_dist_for_route_search, 0, false);
        for (const auto & tmp_path : tmp_paths) {
          bool already_searched = false;
          for (const auto & path : center_paths) {
            for (const auto & llt : path) {
              if (tmp_path.back().id() == llt.id()) {
                already_searched = true;
              }
            }
          }
          if (!already_searched) {
            center_paths.push_back(tmp_path);
          }
        }
      }
      paths.insert(paths.end(), center_paths.begin(), center_paths.end());
      paths.insert(paths.end(), right_paths.begin(), right_paths.end());
      paths.insert(paths.end(), left_paths.begin(), left_paths.end());
    }
    if (paths.size() == 0) {
      geometry_msgs::msg::PointStamped debug_point;
      geometry_msgs::msg::PointStamped point_orig;
      point_orig.point = tmp_object.object.state.pose_covariance.pose.position;
      tf2::doTransform(
        point_orig, debug_point,
        debug_map2lidar_transform);
      tmp_objects_whitout_map.objects.push_back(object);
      continue;
    }

    std::vector<int> lanelet_ids;
    for (const auto & lanelets : paths) {
      for (const auto & lanelet : lanelets) {
        lanelet_ids.push_back(lanelet.id());
      }
    }

    std::string uid_string = toHexString(object.id);
    if (uuid2laneids_.count(uid_string) == 0) {
      uuid2laneids_.emplace(uid_string, lanelet_ids);
    } else {
      // add if not yet having lanelet_id
      for (const auto & current_uid : lanelet_ids) {
        bool is_redundant = false;
        for (const auto & chached_uid : uuid2laneids_.at(uid_string)) {
          if (chached_uid == current_uid) {
            is_redundant = true;
            break;
          }
        }
        if (is_redundant) {
          continue;
        } else {
          uuid2laneids_.at(uid_string).push_back(current_uid);
        }
      }
    }

    std::vector<std::vector<geometry_msgs::msg::Pose>> tmp_paths;
    for (const auto & path : paths) {
      std::vector<geometry_msgs::msg::Pose> tmp_path;
      if (!path.empty()) {
        lanelet::ConstLanelets prev_lanelets = routing_graph_ptr_->previous(path.front());
        if (!prev_lanelets.empty()) {
          lanelet::ConstLanelet prev_lanelet = prev_lanelets.front();
          for (const auto & point : prev_lanelet.centerline()) {
            geometry_msgs::msg::Pose tmp_pose;
            tmp_pose.position.x = point.x();
            tmp_pose.position.y = point.y();
            tmp_pose.position.z = point.z();
            tmp_path.push_back(tmp_pose);
          }
        }
      }
      for (const auto & lanelet : path) {
        for (const auto & point : lanelet.centerline()) {
          geometry_msgs::msg::Pose tmp_pose;
          tmp_pose.position.x = point.x();
          tmp_pose.position.y = point.y();
          tmp_pose.position.z = point.z();
          tmp_path.push_back(tmp_pose);
        }
      }
      tmp_paths.push_back(tmp_path);
    }
    tmp_object.lanes = tmp_paths;
    prediction_input.objects.push_back(tmp_object);
  }

  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);

  std::vector<autoware_perception_msgs::msg::DynamicObject> out_objects_in_map;
  std::vector<geometry_msgs::msg::Point> interpolated_points;
  map_based_prediction_->doPrediction(prediction_input, out_objects_in_map, interpolated_points);
  autoware_perception_msgs::msg::DynamicObjectArray output;
  output.header = in_objects->header;
  output.header.frame_id = "map";
  output.objects = out_objects_in_map;

  std::vector<autoware_perception_msgs::msg::DynamicObject> out_objects_without_map;
  map_based_prediction_->doLinearPrediction(tmp_objects_whitout_map, out_objects_without_map);
  output.objects.insert(
    output.objects.begin(), out_objects_without_map.begin(), out_objects_without_map.end());
  pub_objects_->publish(output);
}

void MapBasedPredictionROS::mapCallback(const autoware_lanelet2_msgs::msg::MapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  RCLCPP_INFO(get_logger(), "Map is loaded");
}
