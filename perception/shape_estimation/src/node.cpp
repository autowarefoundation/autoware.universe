// Copyright 2018 Autoware Foundation. All rights reserved.
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

#include "node.hpp"

#include <memory>

#include "shape_estimation/shape_estimator.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h"

using SemanticType = autoware_perception_msgs::msg::Semantic;

ShapeEstimationNode::ShapeEstimationNode(const rclcpp::NodeOptions & node_options)
: Node("shape_estimation", node_options)
{
  using std::placeholders::_1;
  sub_ = create_subscription<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
    "input", rclcpp::QoS{1}, std::bind(&ShapeEstimationNode::callback, this, _1));

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();  // to latch the topic
  pub_ = create_publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
    "objects", durable_qos);

  bool use_corrector = declare_parameter("use_corrector", true);
  bool use_filter = declare_parameter("use_filter", true);
  use_vehicle_reference_yaw_ = declare_parameter("use_vehicle_reference_yaw", true);
  estimator_ = std::make_unique<ShapeEstimator>(use_corrector, use_filter);
}

void ShapeEstimationNode::callback(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr input_msg)
{
  // Guard
  if (pub_->get_subscription_count() < 1) {return;}

  // Create output msg
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray output_msg;
  output_msg.header = input_msg->header;

  // Estimate shape for each object and pack msg
  for (const auto & feature_object : input_msg->feature_objects) {
    const auto & object = feature_object.object;
    const auto & type = object.semantic.type;
    const auto & feature = feature_object.feature;
    const bool is_vehicle =
      SemanticType::CAR == type || SemanticType::TRUCK == type || SemanticType::BUS == type;

    // convert ros to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(feature.cluster, *cluster);

    // check cluster data
    if (cluster->empty()) {continue;}

    // estimate shape and pose
    autoware_perception_msgs::msg::Shape shape;
    geometry_msgs::msg::Pose pose;
    boost::optional<float> yaw = boost::none;
    if (use_vehicle_reference_yaw_ && is_vehicle) {
      yaw = tf2::getYaw(object.state.pose_covariance.pose.orientation);
    }
    const bool estimated_success =
      estimator_->estimateShapeAndPose(object.semantic.type, *cluster, yaw, shape, pose);

    // If the shape estimation fails, ignore it.
    if (!estimated_success) {continue;}

    output_msg.feature_objects.push_back(feature_object);
    output_msg.feature_objects.back().object.shape = shape;
    output_msg.feature_objects.back().object.state.pose_covariance.pose = pose;
  }

  // Publish
  pub_->publish(output_msg);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ShapeEstimationNode)
