// Copyright 2023 TIER IV, Inc.
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

#include "image_projection_based_fusion/roi_pointcloud_fusion/node.hpp"

#include "image_projection_based_fusion/utils/geometry.hpp"
#include "image_projection_based_fusion/utils/utils.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

namespace image_projection_based_fusion
{
RoiPointCloudFusionNode::RoiPointCloudFusionNode(const rclcpp::NodeOptions & options)
: FusionNode<PointCloud2, DetectedObjects>("roi_pointcloud_fusion", options)
{
  min_cluster_size_ = static_cast<int>(declare_parameter("min_cluster_size", 2));
  cluster_threshold_radius_ = declare_parameter<double>("cluster_threshold_radius", 0.5);
  cluster_threshold_distance_ = declare_parameter<double>("cluster_threshold_distance", 1.0);
  pub_objects_ptr_ = this->create_publisher<DetectedObjects>("output_objects", rclcpp::QoS{1});
}

void RoiPointCloudFusionNode::preprocess(__attribute__((unused))
                                         sensor_msgs::msg::PointCloud2 & pointcloud_msg)
{
  return;
}

void RoiPointCloudFusionNode::postprocess(__attribute__((unused))
                                          sensor_msgs::msg::PointCloud2 & pointcloud_msg)
{
  const auto objects_sub_count = pub_objects_ptr_->get_subscription_count() +
                                 pub_objects_ptr_->get_intra_process_subscription_count();
  if (objects_sub_count < 1) {
    return;
  }

  DetectedObjects output_msg;
  output_msg.header = pointcloud_msg.header;
  output_msg.objects = output_fused_objects_;

  if (objects_sub_count > 0) {
    pub_objects_ptr_->publish(output_msg);
  }
  output_fused_objects_.clear();
  // return;
}
void RoiPointCloudFusionNode::fuseOnSingleImage(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
  __attribute__((unused)) const std::size_t image_id,
  const DetectedObjectsWithFeature & input_roi_msg,
  const sensor_msgs::msg::CameraInfo & camera_info,
  __attribute__((unused)) sensor_msgs::msg::PointCloud2 & output_pointcloud_msg)
{
  if (input_pointcloud_msg.data.empty()) {
    return;
  }
  std::vector<DetectedObjectWithFeature> output_objs;

  for (const auto & feature_obj : input_roi_msg.feature_objects) {
    if (fuse_unknown_only_) {
      bool is_roi_label_unknown = feature_obj.object.classification.front().label ==
                                  autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
      if (is_roi_label_unknown) {
        output_objs.push_back(feature_obj);
      }
    } else {
      output_objs.push_back(feature_obj);
    }
  }
  if (output_objs.empty()) {
    return;
  }
  // TODO: add making sure input_pointcloud_msg's frame_id is base_link
  Eigen::Matrix4d projection;
  projection << camera_info.p.at(0), camera_info.p.at(1), camera_info.p.at(2), camera_info.p.at(3),
    camera_info.p.at(4), camera_info.p.at(5), camera_info.p.at(6), camera_info.p.at(7),
    camera_info.p.at(8), camera_info.p.at(9), camera_info.p.at(10), camera_info.p.at(11);
  geometry_msgs::msg::TransformStamped transform_stamped;
  // transform pointcloud from frame id to camera optical frame id
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer_, input_roi_msg.header.frame_id, input_pointcloud_msg.header.frame_id,
      input_roi_msg.header.stamp);
    if (!transform_stamped_optional) {
      return;
    }
    transform_stamped = transform_stamped_optional.value();
  }

  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(input_pointcloud_msg, transformed_cloud, transform_stamped);

  std::vector<PointCloud> clusters;
  clusters.resize(output_objs.size());

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cloud, "x"),
       iter_y(transformed_cloud, "y"), iter_z(transformed_cloud, "z"),
       iter_orig_x(input_pointcloud_msg, "x"), iter_orig_y(input_pointcloud_msg, "y"),
       iter_orig_z(input_pointcloud_msg, "z");
       iter_x != iter_x.end();
       ++iter_x, ++iter_y, ++iter_z, ++iter_orig_x, ++iter_orig_y, ++iter_orig_z) {
    if (*iter_z <= 0.0) {
      continue;
    }
    Eigen::Vector4d projected_point = projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
    Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(
      projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());

    for (std::size_t i = 0; i < output_objs.size(); ++i) {
      auto & feature_obj = output_objs.at(i);
      const auto & check_roi = feature_obj.feature.roi;
      auto & cluster = clusters.at(i);

      if (
        check_roi.x_offset <= normalized_projected_point.x() &&
        check_roi.y_offset <= normalized_projected_point.y() &&
        check_roi.x_offset + check_roi.width >= normalized_projected_point.x() &&
        check_roi.y_offset + check_roi.height >= normalized_projected_point.y()) {
        cluster.push_back(pcl::PointXYZ(*iter_orig_x, *iter_orig_y, *iter_orig_z));
      }
    }
  }

  for (std::size_t i = 0; i < clusters.size(); ++i) {
    auto & cluster = clusters.at(i);
    auto & feature_obj = output_objs.at(i);
    if (cluster.points.size() < std::size_t(min_cluster_size_)) {
      continue;
    }
    auto refine_cluster = closest_cluster(
      cluster, cluster_threshold_radius_, cluster_threshold_distance_, min_cluster_size_);
    if (refine_cluster.points.size() < std::size_t(min_cluster_size_)) {
      continue;
    }
    addShapeAndKinematic(refine_cluster, feature_obj);
    output_fused_objects_.push_back(feature_obj.object);
  }
}

bool RoiPointCloudFusionNode::out_of_scope(__attribute__((unused)) const DetectedObjects & obj)
{
  return false;
}
}  // namespace image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_projection_based_fusion::RoiPointCloudFusionNode)
