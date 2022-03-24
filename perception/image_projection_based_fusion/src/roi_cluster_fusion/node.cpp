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

#include "image_projection_based_fusion/roi_cluster_fusion/node.hpp"

namespace
{

using tier4_perception_msgs::msg::DetectedObjectWithFeature;

void extractRois(
  const std::vector<DetectedObjectWithFeature> & feature_objects,
  std::vector<sensor_msgs::msg::RegionOfInterest> & rois)
{
  rois.reserve(feature_objects.size());
  for (const auto & obj : feature_objects) {
    rois.push_back(obj.feature.roi);
  }
}
}  // namespace

namespace image_projection_based_fusion
{

RoiClusterFusionNode::RoiClusterFusionNode(const rclcpp::NodeOptions & options)
: FusionNode<DetectedObjectsWithFeature>("roi_cluster_fusion", options)
{
}

void RoiClusterFusionNode::fusionOnSingleImage(
  const int image_id, const DetectedObjectsWithFeature & input_roi_msg)
{
  std::cout << "RoiClusterFusionNode::fusionOnSingleImage" << std::endl;

  std::vector<sensor_msgs::msg::RegionOfInterest> rois;
  extractRois(input_roi_msg.feature_objects, rois);
  debugger_->image_rois_ = rois;
  debugger_->publishImage(image_id, input_roi_msg.header.stamp);
}

}  // namespace image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_projection_based_fusion::RoiClusterFusionNode)
