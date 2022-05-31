// Copyright 2020 Tier IV, Inc.
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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#include <chrono>
#include <unordered_map>
// #include <tf2_sensor_msgs/msg/tf2_sensor_msgs.hpp>
#include <object_association_merger/node.hpp>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace
{
boost::optional<geometry_msgs::msg::Transform> getTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time)
{
  try {
    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer.lookupTransform(
      /*target*/ target_frame_id, /*src*/ source_frame_id, time,
      rclcpp::Duration::from_seconds(0.5));
    return self_transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("multi_object_tracker"), ex.what());
    return boost::none;
  }
}

bool transformDetectedObjects(
  const autoware_auto_perception_msgs::msg::DetectedObjects & input_msg,
  const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
  autoware_auto_perception_msgs::msg::DetectedObjects & output_msg)
{
  output_msg = input_msg;

  /* transform to world coordinate */
  if (input_msg.header.frame_id != target_frame_id) {
    output_msg.header.frame_id = target_frame_id;
    tf2::Transform tf_target2objects_world;
    tf2::Transform tf_target2objects;
    tf2::Transform tf_objects_world2objects;
    {
      const auto ros_target2objects_world =
        getTransform(tf_buffer, input_msg.header.frame_id, target_frame_id, input_msg.header.stamp);
      if (!ros_target2objects_world) {
        return false;
      }
      tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
    }
    for (auto & object : output_msg.objects) {
      tf2::fromMsg(object.kinematics.pose_with_covariance.pose, tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      tf2::toMsg(tf_target2objects, object.kinematics.pose_with_covariance.pose);
      // TODO(yukkysaito) transform covariance
    }
  }
  return true;
}
}  // namespace

namespace object_association
{
ObjectAssociationMergerNode::ObjectAssociationMergerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("object_merger_node", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  object0_sub_(this, "input/object0", rclcpp::QoS{1}.get_rmw_qos_profile()),
  object1_sub_(this, "input/object1", rclcpp::QoS{1}.get_rmw_qos_profile()),
  sync_(SyncPolicy(10), object0_sub_, object1_sub_)
{
  // Create publishers and subscribers
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_.registerCallback(std::bind(&ObjectAssociationMergerNode::objectsCallback, this, _1, _2));
  merged_object_pub_ = create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "output/object", rclcpp::QoS{1});

  // Parameters
    base_link_frame_id_ = declare_parameter<std::string>("base_link_frame_id", "base_link");

  const auto tmp = this->declare_parameter<std::vector<int64_t>>("can_assign_matrix");
  const std::vector<int> can_assign_matrix(tmp.begin(), tmp.end());
  const auto max_dist_matrix = this->declare_parameter<std::vector<double>>("max_dist_matrix");
  const auto max_area_matrix = this->declare_parameter<std::vector<double>>("max_area_matrix");
  const auto min_area_matrix = this->declare_parameter<std::vector<double>>("min_area_matrix");
  const auto max_rad_matrix = this->declare_parameter<std::vector<double>>("max_rad_matrix");
  const auto min_iou_matrix = this->declare_parameter<std::vector<double>>("min_iou_matrix");
  data_association_ = std::make_unique<DataAssociation>(
    can_assign_matrix, max_dist_matrix, max_area_matrix, min_area_matrix, max_rad_matrix,
    min_iou_matrix);
}

void ObjectAssociationMergerNode::objectsCallback(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_object0_msg,
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_object1_msg)
{
  // Guard
  if (merged_object_pub_->get_subscription_count() < 1) {
    return;
  }

  /* transform to base_link coordinate */
  autoware_auto_perception_msgs::msg::DetectedObjects transformed_object0,transformed_object1;
  if (
    !transformDetectedObjects(
      *input_object0_msg, base_link_frame_id_, tf_buffer_, transformed_object0) ||
    !transformDetectedObjects(
      *input_object1_msg, base_link_frame_id_, tf_buffer_, transformed_object1)) {
    return;
  }

  // build output msg
  autoware_auto_perception_msgs::msg::DetectedObjects output_msg;
  output_msg.header = input_object0_msg->header;

  /* global nearest neighbor */
  std::unordered_map<int, int> direct_assignment;
  std::unordered_map<int, int> reverse_assignment;
  Eigen::MatrixXd score_matrix =
    data_association_.calcScoreMatrix(transformed_object1, transformed_object0);
  data_association_.assign(score_matrix, direct_assignment, reverse_assignment);
  /* global nearest neighbor */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  Eigen::MatrixXd score_matrix =
    data_association_->calcScoreMatrix(transformed_object1, transformed_object0);
  data_association_->assign(score_matrix, direct_assignment, reverse_assignment);
  for (size_t object0_idx = 0; object0_idx < transformed_object0.objects.size(); ++object0_idx) {
    if (direct_assignment.find(object0_idx) != direct_assignment.end()) {  // found
      // The one with the higher score will be hired.
      if (
        transformed_object1.objects.at(direct_assignment.at(object0_idx)).existence_probability <
        transformed_object0.objects.at(object0_idx).existence_probability) {
        output_msg.objects.push_back(transformed_object0.objects.at(object0_idx));
      } else {
        output_msg.objects.push_back(
          transformed_object1.objects.at(direct_assignment.at(object0_idx)));
      }
    } else {  // not found
      output_msg.objects.push_back(transformed_object0.objects.at(object0_idx));
    }
  }
  for (size_t object1_idx = 0; object1_idx < transformed_object1.objects.size(); ++object1_idx) {
    if (reverse_assignment.find(object1_idx) != reverse_assignment.end()) {  // found
    } else {                                                                 // not found
      output_msg.objects.push_back(transformed_object1.objects.at(object1_idx));
    }
  }

  // publish output msg
  merged_object_pub_->publish(output_msg);
}
}  // namespace object_association

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(object_association::ObjectAssociationMergerNode)
