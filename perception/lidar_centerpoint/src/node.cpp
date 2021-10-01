// Copyright 2021 Tier IV, Inc.
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

#include <memory>
#include <vector>

#include "autoware_utils/geometry/geometry.hpp"
#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "config.hpp"
#include "lidar_centerpoint/node.hpp"

namespace centerpoint
{
LidarCenterPointNode::LidarCenterPointNode(const rclcpp::NodeOptions & node_options)
: Node("lidar_center_point", node_options)
{
  score_threshold_ = this->declare_parameter("score_threshold", 0.4);
  densification_base_frame_ = this->declare_parameter("densification_base_frame", "map");
  densification_past_frames_ = this->declare_parameter("densification_past_frames", 1);
  use_vfe_trt_ = this->declare_parameter("use_vfe_trt", false);
  use_head_trt_ = this->declare_parameter("use_head_trt", true);
  trt_precision_ = this->declare_parameter("trt_precision", "fp16");
  vfe_onnx_path_ = this->declare_parameter("vfe_onnx_path", "");
  vfe_engine_path_ = this->declare_parameter("vfe_engine_path", "");
  vfe_pt_path_ = this->declare_parameter("vfe_pt_path", "");
  head_onnx_path_ = this->declare_parameter("head_onnx_path", "");
  head_engine_path_ = this->declare_parameter("head_engine_path", "");
  head_pt_path_ = this->declare_parameter("head_pt_path", "");

  NetworkParam vfe_param(
    vfe_onnx_path_, vfe_engine_path_, vfe_pt_path_, trt_precision_, use_vfe_trt_);
  NetworkParam head_param(
    head_onnx_path_, head_engine_path_, head_pt_path_, trt_precision_, use_head_trt_);
  densification_ptr_ = std::make_unique<PointCloudDensification>(
    densification_base_frame_, densification_past_frames_, this->get_clock());
  detector_ptr_ = std::make_unique<CenterPointTRT>(vfe_param, head_param, /*verbose=*/ false);

  pointcloud_sub_ =
    this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&LidarCenterPointNode::pointCloudCallback, this, std::placeholders::_1));
  objects_pub_ =
    this->create_publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
    "~/output/objects", rclcpp::QoS{1});
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/debug/pointcloud_densification", rclcpp::SensorDataQoS{}.keep_last(1));
}

void LidarCenterPointNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg)
{
  const auto objects_sub_count = objects_pub_->get_subscription_count() +
    objects_pub_->get_intra_process_subscription_count();
  const auto pointcloud_sub_count = pointcloud_pub_->get_subscription_count() +
    pointcloud_pub_->get_intra_process_subscription_count();
  if (objects_sub_count < 1 && pointcloud_sub_count < 1) {return;}

  auto stacked_pointcloud_msg = densification_ptr_->stackPointCloud(*input_pointcloud_msg);
  std::vector<float> boxes3d_vec = detector_ptr_->detect(stacked_pointcloud_msg);

  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray output_msg;
  output_msg.header = input_pointcloud_msg->header;
  for (size_t obj_i = 0; obj_i < boxes3d_vec.size() / Config::num_box_features; obj_i++) {
    float score = boxes3d_vec[obj_i * Config::num_box_features + 0];
    if (score < score_threshold_) {
      continue;
    }

    int class_id = static_cast<int>(boxes3d_vec[obj_i * Config::num_box_features + 1]);
    float x = boxes3d_vec[obj_i * Config::num_box_features + 2];
    float y = boxes3d_vec[obj_i * Config::num_box_features + 3];
    float z = boxes3d_vec[obj_i * Config::num_box_features + 4];
    float l = boxes3d_vec[obj_i * Config::num_box_features + 5];
    float w = boxes3d_vec[obj_i * Config::num_box_features + 6];
    float h = boxes3d_vec[obj_i * Config::num_box_features + 7];
    float yaw = boxes3d_vec[obj_i * Config::num_box_features + 8];

    autoware_perception_msgs::msg::DynamicObjectWithFeature feature_obj;
    autoware_perception_msgs::msg::Semantic semantic;
    switch (class_id) {
      case 0:
        semantic.type = autoware_perception_msgs::msg::Semantic::CAR;
        break;
      case 1:
        semantic.type = autoware_perception_msgs::msg::Semantic::PEDESTRIAN;
        break;
      case 2:
        semantic.type = autoware_perception_msgs::msg::Semantic::BICYCLE;
        break;
      default:
        semantic.type = autoware_perception_msgs::msg::Semantic::UNKNOWN;
    }
    semantic.confidence = score;
    feature_obj.object.semantic = semantic;

    geometry_msgs::msg::Pose pose;
    pose.position = autoware_utils::createPoint(x, y, z);
    pose.orientation = autoware_utils::createQuaternionFromYaw(yaw);
    feature_obj.object.state.pose_covariance.pose = pose;

    autoware_perception_msgs::msg::Shape shape;
    shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    shape.dimensions.x = l;
    shape.dimensions.y = w;
    shape.dimensions.z = h;
    feature_obj.object.shape = shape;

    // Note: object size is referred from multi_object_tracker
    if ((w * l > 2.2 * 5.5) && (w * l <= 2.5 * 7.9)) {
      feature_obj.object.semantic.type = semantic.type =
        autoware_perception_msgs::msg::Semantic::TRUCK;
    } else if (w * l > 2.5 * 7.9) {
      feature_obj.object.semantic.type = semantic.type =
        autoware_perception_msgs::msg::Semantic::BUS;
    }

    output_msg.feature_objects.emplace_back(feature_obj);
  }

  if (objects_sub_count > 0) {
    objects_pub_->publish(output_msg);
  }
  if (pointcloud_sub_count > 0) {
    pointcloud_pub_->publish(stacked_pointcloud_msg);
  }
}

}  // namespace centerpoint

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(centerpoint::LidarCenterPointNode)
