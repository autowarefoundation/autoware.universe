// Copyright 2022 Tier IV, Inc.
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

#ifndef IMAGE_PROJECTION_BASED_FUSION__ROI_DETECTED_OBJECT_FUSION__NODE_HPP_
#define IMAGE_PROJECTION_BASED_FUSION__ROI_DETECTED_OBJECT_FUSION__NODE_HPP_

#include "image_projection_based_fusion/fusion_node.hpp"

#include <memory>

namespace image_projection_based_fusion
{

class RoiDetectedObjectFusionNode : public FusionNode<DetectedObjects>
{
public:
  explicit RoiDetectedObjectFusionNode(const rclcpp::NodeOptions & options);

protected:
  void fusionOnSingleImage(
    const int image_id, const DetectedObjectsWithFeature & input_roi_msg) override;
};

}  // namespace image_projection_based_fusion

#endif  // IMAGE_PROJECTION_BASED_FUSION__ROI_DETECTED_OBJECT_FUSION__NODE_HPP_
