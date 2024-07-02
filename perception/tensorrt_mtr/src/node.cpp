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

#include "tensorrt_mtr/node.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>

namespace trt_mtr
{
namespace
{
// Return the Lanelet subtype name. If input Lanelet has no attribute named `type` return empty
// string `""`.
std::string getLaneletSubtype(const lanelet::Lanelet & lanelet)
{
  if (!lanelet.hasAttribute("subtype")) {
    return "";
  } else {
    const auto subtype = lanelet.attribute("subtype").as<std::string>();
    return subtype ? subtype.get() : "";
  }
}

// Return the LineString type name. If input LineString has no attribute named `type` return empty
// string `""`.
std::string getLineStringType(const lanelet::ConstLineString3d & linestring)
{
  if (!linestring.hasAttribute("type")) {
    return "";
  } else {
    const auto type = linestring.attribute("type").as<std::string>();
    return type ? type.get() : "";
  }
}

// Return the LineString subtype name. If input LineString has no attribute named `subtype` return
// empty string `""`.
std::string getLineStringSubtype(const lanelet::ConstLineString3d & linestring)
{
  if (!linestring.hasAttribute("subtype")) {
    return "";
  } else {
    const auto subtype = linestring.attribute("subtype").as<std::string>();
    return subtype ? subtype.get() : "";
  }
}

// Check whether lanelet has an attribute named `turn_direction`.
bool isTurnIntersection(const lanelet::Lanelet & lanelet)
{
  if (lanelet.hasAttribute("turn_direction")) {
    return true;
  } else {
    return false;
  }
}

// Insert source `LanePoints` into target `LanePoints`.
void insertLanePoints(const std::vector<LanePoint> & src, std::vector<LanePoint> & dst)
{
  dst.reserve(dst.size() * 2);
  dst.insert(dst.end(), src.cbegin(), src.cend());
}

// Convert `TrackedObject` to `AgentState`.
AgentState trackedObjectToAgentState(const TrackedObject & object, const bool is_valid)
{
  const auto & pose = object.kinematics.pose_with_covariance.pose;
  const auto & twist = object.kinematics.twist_with_covariance.twist;
  const auto & accel = object.kinematics.acceleration_with_covariance.accel;
  const auto & dimensions = object.shape.dimensions;
  const auto yaw = tf2::getYaw(pose.orientation);
  const auto valid = is_valid ? 1.0f : 0.0f;

  return {
    static_cast<float>(pose.position.x),
    static_cast<float>(pose.position.y),
    static_cast<float>(pose.position.z),
    static_cast<float>(dimensions.x),
    static_cast<float>(dimensions.y),
    static_cast<float>(dimensions.z),
    static_cast<float>(yaw),
    static_cast<float>(twist.linear.x),
    static_cast<float>(twist.linear.y),
    static_cast<float>(accel.linear.x),
    static_cast<float>(accel.linear.y),
    valid};
}

// Get the label index corresponding to AgentLabel. If the label of tracked object is not * defined
// in AgentLabel returns `-1`.
int getLabelIndex(const TrackedObject & object)
{
  const auto classification = object_recognition_utils::getHighestProbLabel(object.classification);
  if (object_recognition_utils::isCarLikeVehicle(classification)) {
    return AgentLabel::VEHICLE;
  } else if (classification == ObjectClassification::PEDESTRIAN) {
    return AgentLabel::PEDESTRIAN;
  } else if (
    classification == ObjectClassification::MOTORCYCLE ||
    classification == ObjectClassification::BICYCLE) {
    return AgentLabel::CYCLIST;
  } else {
    return -1;  // other labels
  }
}

// Return corresponding PrecisionType from string.
PrecisionType getPrecisionType(const std::string & name)
{
  if (name == "FP32") {
    return PrecisionType::FP32;
  } else if (name == "FP16") {
    return PrecisionType::FP16;
  } else if (name == "INT8") {
    return PrecisionType::INT8;
  } else {
    throw std::invalid_argument("Invalid precision name.");
  }
}

// Return corresponding CalibrationType from string.
CalibrationType getCalibrationType(const std::string & name)
{
  if (name == "ENTROPY") {
    return CalibrationType::ENTROPY;
  } else if (name == "LEGACY") {
    return CalibrationType::LEGACY;
  } else if (name == "PERCENTILE") {
    return CalibrationType::PERCENTILE;
  } else if (name == "MINMAX") {
    return CalibrationType::MINMAX;
  } else {
    throw std::invalid_argument("Invalid calibration name.");
  }
}
}  // namespace

MTRNode::MTRNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("tensorrt_mtr", node_options), transform_listener_(this), polyline_type_map_(this)
{
  // TODO(ktro2828)
  {
    // Build MTR
    // Model config
    const auto model_path = declare_parameter<std::string>("model_params.model_path");
    const auto target_labels =
      declare_parameter<std::vector<std::string>>("model_params.target_labels");
    const auto num_past = static_cast<size_t>(declare_parameter<int>("model_params.num_past"));
    const auto num_mode = static_cast<size_t>(declare_parameter<int>("model_params.num_mode"));
    const auto num_future = static_cast<size_t>(declare_parameter<int>("model_params.num_future"));
    const auto max_num_polyline =
      static_cast<size_t>(declare_parameter<int>("model_params.max_num_polyline"));
    const auto max_num_point =
      static_cast<size_t>(declare_parameter<int>("model_params.max_num_point"));
    const auto point_break_distance =
      static_cast<float>(declare_parameter<double>("model_params.point_break_distance"));
    const auto intention_point_filepath =
      declare_parameter<std::string>("model_params.intention_point_filepath");
    const auto num_intention_point_cluster =
      static_cast<size_t>(declare_parameter<int>("model_params.num_intention_point_cluster"));
    config_ptr_ = std::make_unique<MTRConfig>(
      target_labels, num_past, num_mode, num_future, max_num_polyline, max_num_point,
      point_break_distance, intention_point_filepath, num_intention_point_cluster);
    // Build config
    const auto is_dynamic = declare_parameter<bool>("build_params.is_dynamic");
    const auto precision_str = declare_parameter<std::string>("build_params.precision");
    const auto calibration_str = declare_parameter<std::string>("build_params.calibration");
    const auto precision = getPrecisionType(precision_str);
    const auto calibration = getCalibrationType(calibration_str);
    build_config_ptr_ = std::make_unique<BuildConfig>(is_dynamic, precision, calibration);
    model_ptr_ = std::make_unique<TrtMTR>(model_path, *config_ptr_.get(), *build_config_ptr_.get());
  }

  sub_objects_ = create_subscription<TrackedObjects>(
    "~/input/objects", rclcpp::QoS{1}, std::bind(&MTRNode::callback, this, std::placeholders::_1));
  sub_map_ = create_subscription<HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MTRNode::onMap, this, std::placeholders::_1));
  sub_ego_ = create_subscription<Odometry>(
    "~/input/ego", rclcpp::QoS{1}, std::bind(&MTRNode::onEgo, this, std::placeholders::_1));

  pub_objects_ = create_publisher<PredictedObjects>("~/output/objects", rclcpp::QoS{1});

  if (declare_parameter<bool>("build_only")) {
    RCLCPP_INFO(get_logger(), "TensorRT engine file is built and exit.");
    rclcpp::shutdown();
  }
}

void MTRNode::callback(const TrackedObjects::ConstSharedPtr object_msg)
{
  if (!polyline_ptr_) {
    RCLCPP_WARN(get_logger(), "No polyline");
    return;
  }

  const auto current_time = static_cast<float>(rclcpp::Time(object_msg->header.stamp).seconds());

  timestamps_.emplace_back(current_time);
  // TODO(ktro2828): update timestamps
  if (timestamps_.size() < config_ptr_->num_past) {
    RCLCPP_WARN(get_logger(), "Not enough timestamp");
    return;  // Not enough timestamps
  } else if (config_ptr_->num_past < timestamps_.size()) {
    timestamps_.erase(timestamps_.begin(), timestamps_.begin());
  }

  removeAncientAgentHistory(current_time, object_msg);
  updateAgentHistory(current_time, object_msg);

  std::vector<std::string> object_ids;
  std::vector<AgentHistory> histories;
  std::vector<size_t> label_indices;
  histories.reserve(agent_history_map_.size());
  object_ids.reserve(agent_history_map_.size());
  label_indices.reserve(agent_history_map_.size());
  int sdc_index = -1;
  for (const auto & [object_id, history] : agent_history_map_) {
    object_ids.emplace_back(object_id);
    histories.emplace_back(history);
    label_indices.emplace_back(history.label_index());
    if (object_id == EGO_ID) {
      sdc_index = histories.size() - 1;
    }
  }

  if (sdc_index == -1) {
    RCLCPP_WARN(get_logger(), "No EGO");
    return;
  }

  const auto target_indices = extractTargetAgent(histories);
  if (target_indices.empty()) {
    RCLCPP_WARN(get_logger(), "No target agents");
    return;
  }

  const auto relative_timestamps = getRelativeTimestamps();
  AgentData agent_data(
    histories, static_cast<size_t>(sdc_index), target_indices, label_indices, relative_timestamps);

  std::vector<PredictedTrajectory> trajectories;
  if (!model_ptr_->doInference(agent_data, *polyline_ptr_.get(), trajectories)) {
    RCLCPP_WARN(get_logger(), "Inference failed");
    return;
  }

  PredictedObjects output;
  output.header = object_msg->header;
  output.objects.reserve(target_indices.size());
  for (size_t i = 0; i < target_indices.size(); ++i) {
    const auto & trajectory = trajectories.at(i);
    const auto & target_idx = target_indices.at(i);
    const auto & object_id = object_ids.at(target_idx);
    const auto & object = object_msg_map_.at(object_id);
    auto predicted_object = generatePredictedObject(object, trajectory);
    output.objects.emplace_back(predicted_object);
  }

  // Publish results
  pub_objects_->publish(output);
}

void MTRNode::onMap(const HADMapBin::ConstSharedPtr map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);

  RCLCPP_DEBUG(get_logger(), "[TensorRT MTR]: Start converting lanelet to polyline");
  if (convertLaneletToPolyline()) {
    RCLCPP_DEBUG(get_logger(), "[TensorRT MTR]: Success to convert lanelet to polyline");
  } else {
    RCLCPP_WARN(get_logger(), "[TensorRT MTR]: Fail to convert lanelet to polyline");
  }
}

void MTRNode::onEgo(const Odometry::ConstSharedPtr ego_msg)
{
  const auto current_time = static_cast<float>(rclcpp::Time(ego_msg->header.stamp).seconds());
  const auto & position = ego_msg->pose.pose.position;
  const auto & twist = ego_msg->twist.twist;
  const auto yaw = static_cast<float>(tf2::getYaw(ego_msg->pose.pose.orientation));
  float ax = 0.0f, ay = 0.0f;
  if (!ego_states_.empty()) {
    const auto & latest_state = ego_states_.back();
    const auto time_diff = current_time - latest_state.first;
    ax = (static_cast<float>(twist.linear.x) - latest_state.second.vx()) / (time_diff + 1e-10f);
    ay = static_cast<float>(twist.linear.y) - latest_state.second.vy() / (time_diff + 1e-10f);
  }

  // TODO(ktro2828): use received ego size topic
  ego_states_.emplace_back(std::make_pair(
    current_time,
    AgentState(
      static_cast<float>(position.x), static_cast<float>(position.y),
      static_cast<float>(position.z), EGO_LENGTH, EGO_WIDTH, EGO_HEIGHT, yaw,
      static_cast<float>(twist.linear.x), static_cast<float>(twist.linear.y), ax, ay, true)));

  constexpr size_t max_buffer_size = 100;
  if (max_buffer_size < ego_states_.size()) {
    ego_states_.erase(ego_states_.begin(), ego_states_.begin());
  }
}

bool MTRNode::convertLaneletToPolyline()
{
  if (!lanelet_map_ptr_) {
    return false;
  }

  std::vector<LanePoint> all_points;
  for (const auto & lanelet : lanelet_map_ptr_->laneletLayer) {
    const auto lanelet_subtype = getLaneletSubtype(lanelet);
    if (
      lanelet_subtype == "road" || lanelet_subtype == "highway" ||
      lanelet_subtype == "road_shoulder" || lanelet_subtype == "pedestrian_lane" ||
      lanelet_subtype == "bicycle_lane" || lanelet_subtype == "walkway") {
      if (
        lanelet_subtype == "road" || lanelet_subtype == "highway" ||
        lanelet_subtype == "road_shoulder") {
        const auto & type_id = polyline_type_map_.getTypeID(lanelet_subtype);
        auto points = getLanePointFromLineString(lanelet.centerline3d(), type_id);
        insertLanePoints(points, all_points);
      }
      if (!isTurnIntersection(lanelet)) {
        const auto & left = lanelet.leftBound3d();
        const auto left_type = getLineStringType(left);
        if (left_type == "line_thin" || left_type == "line_thick") {
          const auto left_subtype = getLineStringSubtype(left);
          const auto & type_id = polyline_type_map_.getTypeID(left_subtype);
          if (type_id != -1) {
            auto points = getLanePointFromLineString(left, type_id);
            insertLanePoints(points, all_points);
          }
        }
        const auto & right = lanelet.rightBound3d();
        const auto right_type = getLineStringType(right);
        if (right_type == "line_thin" || right_type == "line_thick") {
          const auto right_subtype = getLineStringSubtype(right);
          const auto & type_id = polyline_type_map_.getTypeID(right_subtype);
          if (type_id != -1) {
            auto points = getLanePointFromLineString(right, type_id);
            insertLanePoints(points, all_points);
          }
        }
      }
    } else if (lanelet_subtype == "crosswalk") {
      const auto & type_id = polyline_type_map_.getTypeID(lanelet_subtype);
      auto points = getLanePointFromPolygon(lanelet.polygon3d(), type_id);
      insertLanePoints(points, all_points);
    }
  }

  for (const auto & linestring : lanelet_map_ptr_->lineStringLayer) {
    const auto linestring_type = getLineStringType(linestring);
    if (linestring_type != "road_boarder" && linestring_type != "traffic_sign") {
      continue;
    }
    const auto & type_id = polyline_type_map_.getTypeID(linestring_type);
    auto points = getLanePointFromLineString(linestring, type_id);
    insertLanePoints(points, all_points);
  }

  if (all_points.size() == 0) {
    return false;
  } else {
    polyline_ptr_ = std::make_shared<PolylineData>(
      all_points, config_ptr_->max_num_polyline, config_ptr_->max_num_point,
      config_ptr_->point_break_distance);
    return true;
  }
}

void MTRNode::removeAncientAgentHistory(
  const float current_time, const TrackedObjects::ConstSharedPtr objects_msg)
{
  constexpr float time_threshold = 1.0f;  // TODO(ktro2828): use parameter
  for (const auto & object : objects_msg->objects) {
    const auto & object_id = tier4_autoware_utils::toHexString(object.object_id);
    if (agent_history_map_.count(object_id) == 0) {
      continue;
    }

    const auto & history = agent_history_map_.at(object_id);
    if (history.is_ancient(current_time, time_threshold)) {
      agent_history_map_.erase(object_id);
    }
  }

  if (
    agent_history_map_.count(EGO_ID) != 0 &&
    agent_history_map_.at(EGO_ID).is_ancient(current_time, time_threshold)) {
    agent_history_map_.erase(EGO_ID);
  }
}

void MTRNode::updateAgentHistory(
  const float current_time, const TrackedObjects::ConstSharedPtr objects_msg)
{
  // TODO(ktro2828): use ego info
  std::vector<std::string> observed_ids;
  for (const auto & object : objects_msg->objects) {
    auto label_index = getLabelIndex(object);
    if (label_index == -1) {
      continue;
    }

    const auto & object_id = tier4_autoware_utils::toHexString(object.object_id);
    observed_ids.emplace_back(object_id);
    object_msg_map_.emplace(object_id, object);
    auto state = trackedObjectToAgentState(object, true);

    if (agent_history_map_.count(object_id) == 0) {
      AgentHistory history(object_id, label_index, config_ptr_->num_past);
      history.update(current_time, state);
      agent_history_map_.emplace(object_id, history);
    } else {
      agent_history_map_.at(object_id).update(current_time, state);
    }
  }

  auto ego_state = extractNearestEgo(current_time);
  if (agent_history_map_.count(EGO_ID) == 0) {
    AgentHistory history(EGO_ID, AgentLabel::VEHICLE, config_ptr_->num_past);
    history.update(current_time, ego_state);
    agent_history_map_.emplace(EGO_ID, history);
  } else {
    agent_history_map_.at(EGO_ID).update(current_time, ego_state);
  }
  observed_ids.emplace_back(EGO_ID);

  // update unobserved histories with empty
  for (auto & [object_id, history] : agent_history_map_) {
    if (std::find(observed_ids.cbegin(), observed_ids.cend(), object_id) != observed_ids.cend()) {
      continue;
    }
    history.update_empty();
  }
}

AgentState MTRNode::extractNearestEgo(const float current_time) const
{
  auto state = std::min_element(
    ego_states_.cbegin(), ego_states_.cend(), [&](const auto & s1, const auto & s2) {
      return std::abs(s1.first - current_time) < std::abs(s2.first - current_time);
    });
  return state->second;
}

std::vector<size_t> MTRNode::extractTargetAgent(const std::vector<AgentHistory> & histories)
{
  std::vector<std::pair<size_t, float>> distances;
  for (size_t i = 0; i < histories.size(); ++i) {
    const auto & history = histories.at(i);
    if (!history.is_valid_latest() || history.object_id() == EGO_ID) {
      distances.emplace_back(std::make_pair(i, INFINITY));
    } else {
      auto map2ego = transform_listener_.getTransform(
        "base_link",  // target
        "map",        // src
        rclcpp::Time(), rclcpp::Duration::from_seconds(0.1));
      if (!map2ego) {
        RCLCPP_WARN(get_logger(), "Failed to get transform from map to base_link.");
        return {};
      }
      const auto state = history.get_latest_state();
      geometry_msgs::msg::PoseStamped pose_in_map;
      pose_in_map.pose.position.x = state.x();
      pose_in_map.pose.position.y = state.y();
      pose_in_map.pose.position.z = state.z();
      pose_in_map.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(state.yaw());

      geometry_msgs::msg::PoseStamped pose_in_ego;
      tf2::doTransform(pose_in_map, pose_in_ego, *map2ego);

      const auto dist = std::hypot(
        pose_in_ego.pose.position.x, pose_in_ego.pose.position.y, pose_in_ego.pose.position.z);
      distances.emplace_back(std::make_pair(i, dist));
    }
  }

  std::sort(distances.begin(), distances.end(), [](const auto & item1, const auto & item2) {
    return item1.second < item2.second;
  });

  constexpr size_t max_target_size = 15;  // TODO(ktro2828): use a parameter
  std::vector<size_t> target_indices;
  target_indices.reserve(max_target_size);
  for (const auto & [idx, _] : distances) {
    target_indices.emplace_back(idx);
    if (max_target_size <= target_indices.size()) {
      break;
    }
  }

  return target_indices;
}

std::vector<float> MTRNode::getRelativeTimestamps() const
{
  auto output = timestamps_;
  for (auto & t : output) {
    t -= timestamps_.at(0);
  }
  return output;
}

PredictedObject MTRNode::generatePredictedObject(
  const TrackedObject & object, const PredictedTrajectory & trajectory)
{
  const auto & init_pose_with_cov = object.kinematics.pose_with_covariance;
  const auto & init_twist_with_cov = object.kinematics.twist_with_covariance;
  const auto & init_accel_with_cov = object.kinematics.acceleration_with_covariance;

  PredictedObject predicted_object;
  predicted_object.kinematics.initial_pose_with_covariance = init_pose_with_cov;
  predicted_object.kinematics.initial_twist_with_covariance = init_twist_with_cov;
  predicted_object.kinematics.initial_acceleration_with_covariance = init_accel_with_cov;
  predicted_object.classification = object.classification;
  predicted_object.shape = object.shape;
  predicted_object.object_id = object.object_id;

  float max_existence_probability = 0.0f;
  for (const auto & mode : trajectory.get_modes()) {
    PredictedPath waypoints;
    waypoints.confidence = mode.score();
    waypoints.time_step = rclcpp::Duration::from_seconds(0.1);  // TODO(ktro282): use a parameter
    waypoints.path.reserve(mode.num_future());
    if (max_existence_probability < mode.score()) {
      max_existence_probability = mode.score();
    }
    for (const auto & state : mode.get_waypoints()) {
      geometry_msgs::msg::Pose predicted_pose;
      predicted_pose.position.x = static_cast<double>(state.x());
      predicted_pose.position.y = static_cast<double>(state.y());
      predicted_pose.position.z = init_pose_with_cov.pose.position.z;
      predicted_pose.orientation = init_pose_with_cov.pose.orientation;
      waypoints.path.emplace_back(predicted_pose);
      if (waypoints.path.size() >= waypoints.path.max_size()) {
        break;
      }
    }
    predicted_object.kinematics.predicted_paths.emplace_back(waypoints);
  }
  predicted_object.existence_probability = max_existence_probability;

  return predicted_object;
}
}  // namespace trt_mtr

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(trt_mtr::MTRNode);
