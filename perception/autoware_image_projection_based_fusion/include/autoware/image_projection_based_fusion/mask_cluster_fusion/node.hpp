#include "autoware/image_projection_based_fusion/fusion_node.hpp"

#include <autoware/image_projection_based_fusion/utils/utils.hpp>

#include <cv_bridge/cv_bridge.h>

#include <string>
#include <vector>

namespace autoware::image_projection_based_fusion
{
class MaskClusterFusionNode
: public FusionNode<DetectedObjectsWithFeature, DetectedObjectWithFeature, SegmentationMask>
{
public:
  explicit MaskClusterFusionNode(const rclcpp::NodeOptions & options);

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