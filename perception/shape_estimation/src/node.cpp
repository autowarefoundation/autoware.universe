/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
 *
 *
 * v1.0 Yukihiro Saito
 */

#include "shape_estimation/node.hpp"
#include "shape_estimation/shape_estimator.hpp"

ShapeEstimationNode::ShapeEstimationNode() : Node("shape_estimation")
{
  using std::placeholders::_1;
  sub_ = create_subscription<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
    "input", rclcpp::QoS{1}, std::bind(&ShapeEstimationNode::callback, this, _1));

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();  // to latch the topic
  pub_ = create_publisher<autoware_perception_msgs::msg::DynamicObjectWithFeatureArray>(
    "objects", durable_qos);

  bool use_corrector = declare_parameter("use_corrector", true);
  double l_shape_fitting_search_angle_range =
    declare_parameter("l_shape_fitting_search_angle_range", 3.0);
  bool orientation_reliable = declare_parameter("orientation_reliable", true);
  estimator_ = std::make_unique<ShapeEstimator>(
    l_shape_fitting_search_angle_range, use_corrector, orientation_reliable);
}

void ShapeEstimationNode::callback(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray::ConstSharedPtr input_msg)
{
  // Guard
  if (pub_->get_subscription_count() < 1) return;

  // Create output msg
  autoware_perception_msgs::msg::DynamicObjectWithFeatureArray output_msg;
  output_msg.header = input_msg->header;

  // Estimate shape for each object and pack msg
  for (const auto & feature_object : input_msg->feature_objects) {
    // convert ros to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(feature_object.feature.cluster, *cluster);
    // estimate shape and pose
    autoware_perception_msgs::msg::Shape shape;
    geometry_msgs::msg::Pose pose;

    if (!estimator_->getShapeAndPose(
          feature_object.object.semantic.type, *cluster, feature_object.object.state, shape, pose))
      continue;

    output_msg.feature_objects.push_back(feature_object);
    output_msg.feature_objects.back().object.shape = shape;
    output_msg.feature_objects.back().object.state.pose_covariance.pose = pose;
  }

  // Publish
  pub_->publish(output_msg);
  return;
}
