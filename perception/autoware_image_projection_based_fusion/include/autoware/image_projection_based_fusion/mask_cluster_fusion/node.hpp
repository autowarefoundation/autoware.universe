#include "autoware/image_projection_based_fusion/fusion_node.hpp"

#include <autoware/image_projection_based_fusion/utils/utils.hpp>

#include <cv_bridge/cv_bridge.h>

#include <string>
#include <vector>

namespace autoware::image_projection_based_fusion {
    const std::map<std::string, uint8_t> IOU_MODE_MAP{{"iou",   0},
                                                      {"iou_x", 1},
                                                      {"iou_y", 2}};

    class MaskClusterFusionNode
            : public FusionNode<DetectedObjectsWithFeature, DetectedObjectWithFeature, SegmentationMask> {
    public:
        explicit MaskClusterFusionNode(const rclcpp::NodeOptions &options);

    protected:
        void preprocess(DetectedObjectsWithFeature &output_msg) override;

        void postprocess(DetectedObjectsWithFeature &output_msg) override;

        void fuseOnSingleImage(
                const DetectedObjectsWithFeature &input_cluster_msg, const std::size_t image_id,
                const SegmentationMask &input_mask_msg,
                const sensor_msgs::msg::CameraInfo &camera_info,
                DetectedObjectsWithFeature &output_object_msg) override;

        bool out_of_scope(const DetectedObjectWithFeature & obj) override;

        std::string trust_object_iou_mode_{"iou"};
        bool use_cluster_semantic_type_{false};
        bool only_allow_inside_cluster_{false};
        double roi_scale_factor_{1.1};
        double iou_threshold_{0.0};
        double unknown_iou_threshold_{0.0};
        const float min_roi_existence_prob_ =
                0.1;  // keep small value to lessen affect on merger object stage
        bool remove_unknown_;
        double fusion_distance_;
        double trust_object_distance_;
        std::string non_trust_object_iou_mode_{"iou_x"};
    };
} // namespace autoware::image_projection_based_fusion