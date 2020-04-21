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

ShapeEstimationNode::ShapeEstimationNode() : nh_(""), pnh_("~")
{
  sub_ = nh_.subscribe("input", 1, &ShapeEstimationNode::callback, this);
  pub_ = nh_.advertise<autoware_perception_msgs::DynamicObjectWithFeatureArray>("objects", 1, true);
  // pnh_.param<bool>("use_map_corrent", use_map_correct_, true);
  // if (use_map_correct_)
  //   map_corrector_node_ptr_ = std::make_shared<MapCorrectorNode>();
}

void ShapeEstimationNode::callback(
  const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_msg)
{
  // Guard
  if (pub_.getNumSubscribers() < 1) return;

  // Create output msg
  autoware_perception_msgs::DynamicObjectWithFeatureArray output_msg;
  output_msg.header = input_msg->header;

  // Estimate shape for each object and pack msg
  for (const auto & feature_object : input_msg->feature_objects) {
    // convert ros to pcl
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(feature_object.feature.cluster, *cluster);
    // estimate shape and pose
    autoware_perception_msgs::Shape shape;
    geometry_msgs::Pose pose;
    bool orientation;
    if (!estimator_.getShapeAndPose(
          feature_object.object.semantic.type, *cluster, shape, pose, orientation))
      continue;
    output_msg.feature_objects.push_back(feature_object);
    output_msg.feature_objects.back().object.shape = shape;
    output_msg.feature_objects.back().object.state.pose_covariance.pose = pose;
    output_msg.feature_objects.back().object.state.orientation_reliable = orientation;
  }
  // if (use_map_correct_)
  // {
  //   map_corrector_node_ptr_->correct(output_msg);
  // }

  // Publish
  pub_.publish(output_msg);
  return;
}
