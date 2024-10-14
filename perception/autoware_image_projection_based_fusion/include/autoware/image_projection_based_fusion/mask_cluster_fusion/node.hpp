// Copyright 2024 The Autoware Contributors
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

#include "autoware/image_projection_based_fusion/fusion_node.hpp"

#include <autoware/image_projection_based_fusion/utils/utils.hpp>

#include <cv_bridge/cv_bridge.h>

#include <string>
#include <vector>

namespace autoware::image_projection_based_fusion
{
struct MatchedCluster
{
  uint32_t mask_index{};
  uint32_t cluster_index{};
  uint32_t valid_point_number{};
  sensor_msgs::msg::RegionOfInterest roi;
};
using MatchedClusters = std::vector<MatchedCluster>;

class MaskClusterFusionNode
: public FusionNode<DetectedObjectsWithFeature, DetectedObjectWithFeature, SegmentationMask>
{
public:
  explicit MaskClusterFusionNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<DetectedObjects>::SharedPtr detected_objects_pub_;

protected:
  void preprocess(DetectedObjectsWithFeature & output_msg) override;

  void postprocess(DetectedObjectsWithFeature & output_msg) override;

  void fuseOnSingleImage(
    const DetectedObjectsWithFeature & input_cluster_msg, const std::size_t image_id,
    const SegmentationMask & input_mask_msg, const sensor_msgs::msg::CameraInfo & camera_info,
    DetectedObjectsWithFeature & output_object_msg) override;

  bool out_of_scope(const DetectedObjectWithFeature & obj) override;

  double fusion_distance_;
  double fusion_ratio_;
  bool remove_unknown_;
};
}  // namespace autoware::image_projection_based_fusion