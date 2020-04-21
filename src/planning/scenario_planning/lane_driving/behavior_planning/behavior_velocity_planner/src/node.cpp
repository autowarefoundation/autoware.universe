/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <behavior_velocity_planner/node.h>

#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_routing/Route.h>

#include <visualization_msgs/MarkerArray.h>

#include <utilization/path_utilization.h>

// Scene modules
#include <scene_module/blind_spot/manager.h>
#include <scene_module/crosswalk/manager.h>
#include <scene_module/intersection/manager.h>
#include <scene_module/stop_line/manager.h>
#include <scene_module/traffic_light/manager.h>

namespace
{
template <class T>
T getParam(const ros::NodeHandle & nh, const std::string & key, const T & default_value)
{
  T value;
  nh.param<T>(key, value, default_value);
  return value;
}

template <class T>
T waitForParam(const ros::NodeHandle & nh, const std::string & key)
{
  T value;

  ros::Rate rate(1.0);
  while (ros::ok()) {
    const auto result = nh.getParam(key, value);
    if (result) {
      return value;
    }

    ROS_INFO("waiting for parameter `%s` ...", key.c_str());
    rate.sleep();
  }

  return {};
}

std::shared_ptr<geometry_msgs::TransformStamped> getTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & from, const std::string & to,
  const ros::Time & time = ros::Time(0), const ros::Duration & duration = ros::Duration(0.1))
{
  try {
    return std::make_shared<geometry_msgs::TransformStamped>(
      tf_buffer.lookupTransform(from, to, time, duration));
  } catch (tf2::TransformException & ex) {
    return {};
  }
}

geometry_msgs::TransformStamped waitForTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & from, const std::string & to,
  const ros::Time & time = ros::Time(0), const ros::Duration & duration = ros::Duration(0.1))
{
  ros::Rate rate(1.0);
  while (ros::ok()) {
    const auto transform = getTransform(tf_buffer, from, to, time, duration);
    if (transform) {
      return *transform;
    }

    ROS_INFO(
      "waiting for transform from `%s` to `%s` ... (time = %f, now = %f)", from.c_str(), to.c_str(),
      time.toSec(), ros::Time::now().toSec());
    rate.sleep();
  }
}

geometry_msgs::PoseStamped transform2pose(const geometry_msgs::TransformStamped & transform)
{
  geometry_msgs::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

autoware_planning_msgs::Path to_path(const autoware_planning_msgs::PathWithLaneId & path_with_id)
{
  autoware_planning_msgs::Path path;
  for (const auto & path_point : path_with_id.points) {
    path.points.push_back(path_point.point);
  }
  return path;
}
}  // namespace

BehaviorVelocityPlannerNode::BehaviorVelocityPlannerNode()
: nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
  // Trigger Subscriber
  trigger_sub_path_with_lane_id_ =
    pnh_.subscribe("input/path_with_lane_id", 1, &BehaviorVelocityPlannerNode::onTrigger, this);

  // Subscribers
  sub_dynamic_objects_ = pnh_.subscribe(
    "input/dynamic_objects", 1, &BehaviorVelocityPlannerNode::onDynamicObjects, this);
  sub_no_ground_pointcloud_ = pnh_.subscribe(
    "input/no_ground_pointcloud", 1, &BehaviorVelocityPlannerNode::onNoGroundPointCloud, this);
  sub_vehicle_velocity_ = pnh_.subscribe(
    "input/vehicle_velocity", 1, &BehaviorVelocityPlannerNode::onVehicleVelocity, this);
  sub_lanelet_map_ =
    pnh_.subscribe("input/vector_map", 10, &BehaviorVelocityPlannerNode::onLaneletMap, this);
  sub_traffic_light_states_ = pnh_.subscribe(
    "input/traffic_light_states", 10, &BehaviorVelocityPlannerNode::onTrafficLightStates, this);

  // Publishers
  path_pub_ = pnh_.advertise<autoware_planning_msgs::Path>("output/path", 1);
  debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("debug/path", 1);

  // Parameters
  pnh_.param("forward_path_length", forward_path_length_, 1000.0);
  pnh_.param("backward_path_length", backward_path_length_, 5.0);

  // Vehicle Parameters
  planner_data_.wheel_base = waitForParam<double>(pnh_, "/vehicle_info/wheel_base");
  planner_data_.front_overhang = waitForParam<double>(pnh_, "/vehicle_info/front_overhang");
  planner_data_.vehicle_width = waitForParam<double>(pnh_, "/vehicle_info/vehicle_width");
  // Additional Vehicle Parameters
  pnh_.param(
    "max_accel", planner_data_.max_stop_acceleration_threshold_,
    -5.0);  // TODO read min_acc in velocity_controller_param.yaml?
  // TODO(Kenji Miyake): get from additional vehicle_info?
  planner_data_.base_link2front = planner_data_.front_overhang + planner_data_.wheel_base;

  // Initialize PlannerManager
  if (getParam<bool>(pnh_, "launch_stop_line", true))
    planner_manager_.launchSceneModule(std::make_shared<StopLineModuleManager>());
  if (getParam<bool>(pnh_, "launch_crosswalk", true))
    planner_manager_.launchSceneModule(std::make_shared<CrosswalkModuleManager>());
  if (getParam<bool>(pnh_, "launch_traffic_light", true))
    planner_manager_.launchSceneModule(std::make_shared<TrafficLightModuleManager>());
  if (getParam<bool>(pnh_, "launch_intersection", true))
    planner_manager_.launchSceneModule(std::make_shared<IntersectionModuleManager>());
  if (getParam<bool>(pnh_, "launch_blind_spot", true))
    planner_manager_.launchSceneModule(std::make_shared<BlindSpotModuleManager>());
}

geometry_msgs::PoseStamped BehaviorVelocityPlannerNode::getCurrentPose()
{
  return transform2pose(waitForTransform(tf_buffer_, "map", "base_link"));
}

bool BehaviorVelocityPlannerNode::isDataReady()
{
  const auto & d = planner_data_;

  // from tf
  if (d.current_pose.header.frame_id == "") return false;

  // from callbacks
  if (!d.current_velocity) return false;
  if (!d.dynamic_objects) return false;
  if (!d.no_ground_pointcloud) return false;
  if (!d.lanelet_map) return false;

  return true;
}

void BehaviorVelocityPlannerNode::onDynamicObjects(
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr & msg)
{
  planner_data_.dynamic_objects = msg;
}

void BehaviorVelocityPlannerNode::onNoGroundPointCloud(
  const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  const auto transform =
    getTransform(tf_buffer_, "map", msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));
  if (!transform) {
    ROS_WARN("no transform found for no_ground_pointcloud");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Matrix4f affine = tf2::transformToEigen(transform->transform).matrix().cast<float>();
  sensor_msgs::PointCloud2 transformed_msg;
  pcl_ros::transformPointCloud(affine, *msg, transformed_msg);

  pcl::fromROSMsg(transformed_msg, *no_ground_pointcloud);

  planner_data_.no_ground_pointcloud = no_ground_pointcloud;
}

void BehaviorVelocityPlannerNode::onVehicleVelocity(
  const geometry_msgs::TwistStamped::ConstPtr & msg)
{
  planner_data_.current_velocity = msg;
}

void BehaviorVelocityPlannerNode::onLaneletMap(const autoware_lanelet2_msgs::MapBin::ConstPtr & msg)
{
  // Load map
  planner_data_.lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, planner_data_.lanelet_map, &planner_data_.traffic_rules, &planner_data_.routing_graph);

  // Build graph
  {
    using lanelet::Locations;
    using lanelet::Participants;
    using lanelet::routing::RoutingGraph;
    using lanelet::routing::RoutingGraphConstPtr;
    using lanelet::routing::RoutingGraphContainer;
    using lanelet::traffic_rules::TrafficRulesFactory;

    const auto traffic_rules =
      TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle);
    const auto pedestrian_rules =
      TrafficRulesFactory::create(Locations::Germany, Participants::Pedestrian);

    RoutingGraphConstPtr vehicle_graph =
      RoutingGraph::build(*planner_data_.lanelet_map, *traffic_rules);
    RoutingGraphConstPtr pedestrian_graph =
      RoutingGraph::build(*planner_data_.lanelet_map, *pedestrian_rules);
    RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});

    planner_data_.overall_graphs = std::make_shared<const RoutingGraphContainer>(overall_graphs);
  }
}

void BehaviorVelocityPlannerNode::onTrafficLightStates(
  const autoware_perception_msgs::TrafficLightStateArray::ConstPtr & msg)
{
  for (const auto & state : msg->states) {
    planner_data_.traffic_light_id_map_[state.id] = {msg->header, state};
  }
}

void BehaviorVelocityPlannerNode::onTrigger(
  const autoware_planning_msgs::PathWithLaneId & input_path_msg)
{
  // Check ready
  planner_data_.current_pose = getCurrentPose();
  if (!isDataReady()) {
    return;
  }

  // Plan path velocity
  const auto velocity_planned_path = planner_manager_.planPathVelocity(
    std::make_shared<const PlannerData>(planner_data_), input_path_msg);

  // screening
  const auto filtered_path = filterLitterPathPoint(to_path(velocity_planned_path));

  // interpolation
  const auto interpolated_path_msg = interpolatePath(filtered_path, forward_path_length_);

  // check stop point
  auto output_path_msg = filterStopPathPoint(interpolated_path_msg);
  output_path_msg.header.frame_id = "map";
  output_path_msg.header.stamp = ros::Time::now();

  // TODO: This must be updated in each scene module, but copy from input message for now.
  output_path_msg.drivable_area = input_path_msg.drivable_area;

  path_pub_.publish(output_path_msg);
  publishDebugMarker(output_path_msg, debug_viz_pub_);

  return;
};

void BehaviorVelocityPlannerNode::publishDebugMarker(
  const autoware_planning_msgs::Path & path, const ros::Publisher & pub)
{
  if (pub.getNumSubscribers() < 1) return;

  visualization_msgs::MarkerArray output_msg;
  for (size_t i = 0; i < path.points.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header = path.header;
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.pose = path.points.at(i).pose;
    marker.scale.y = marker.scale.z = 0.05;
    marker.scale.x = 0.25;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.5);
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    output_msg.markers.push_back(marker);
  }
  pub.publish(output_msg);
}
