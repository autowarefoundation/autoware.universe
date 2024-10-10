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

#include "autoware/image_projection_based_fusion/mask_cluster_fusion/node.hpp"

#include "autoware/euclidean_cluster/utils.hpp"
#include "autoware/image_projection_based_fusion/utils/geometry.hpp"
#include "autoware/image_projection_based_fusion/utils/utils.hpp"

namespace autoware::image_projection_based_fusion
{
bool is_far_enough(const DetectedObjectWithFeature & obj, const double distance_threshold)
{
  const auto & position = obj.object.kinematics.pose_with_covariance.pose.position;
  return position.x * position.x + position.y * position.y >
         distance_threshold * distance_threshold;
}

MaskClusterFusionNode::MaskClusterFusionNode(const rclcpp::NodeOptions & options)
: FusionNode<DetectedObjectsWithFeature, DetectedObjectWithFeature, SegmentationMask>(
    "mask_cluster_fusion", options)
{
  fusion_distance_ = declare_parameter<double>("fusion_distance");
  fusion_ratio_ = declare_parameter<double>("fusion_ratio");
  remove_unknown_ = declare_parameter<bool>("remove_unknown");
}

void MaskClusterFusionNode::preprocess(__attribute__((unused))
                                       DetectedObjectsWithFeature & output_msg)
{
  return;
}

void MaskClusterFusionNode::postprocess(__attribute__((unused))
                                        DetectedObjectsWithFeature & output_msg)
{
  if (!remove_unknown_) {
    return;
  }
  DetectedObjectsWithFeature known_objects;
  known_objects.feature_objects.reserve(output_msg.feature_objects.size());
  for (auto & feature_object : output_msg.feature_objects) {
    bool is_roi_label_known = feature_object.object.classification.front().label !=
                              autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
    if (is_roi_label_known) {
      known_objects.feature_objects.push_back(feature_object);
    }
  }
  output_msg.feature_objects = known_objects.feature_objects;
}

void MaskClusterFusionNode::fuseOnSingleImage(
  __attribute__((unused)) const DetectedObjectsWithFeature & input_cluster_msg,
  __attribute__((unused)) const std::size_t image_id,
  __attribute__((unused)) const SegmentationMask & input_mask_msg,
  __attribute__((unused)) const sensor_msgs::msg::CameraInfo & camera_info,
  __attribute__((unused)) DetectedObjectsWithFeature & output_object_msg)
{
  if (!checkCameraInfo(camera_info)) return;

  if (input_mask_msg.image.height == 0 || input_mask_msg.image.width == 0) {
    return;
  }

  // Convert mask to cv::Mat - Resize mask to the same size as the camera image
  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(
      std::make_shared<sensor_msgs::msg::Image>(input_mask_msg.image),
      input_mask_msg.image.encoding);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception:%s", e.what());
    return;
  }
  cv::Mat mask = in_image_ptr->image;
  const int orig_width = camera_info.width;
  const int orig_height = camera_info.height;
  // resize mask to the same size as the camera image
  cv::resize(mask, mask, cv::Size(orig_width, orig_height), 0, 0, cv::INTER_NEAREST);

  image_geometry::PinholeCameraModel pinhole_camera_model;
  pinhole_camera_model.fromCameraInfo(camera_info);

  // get transform from cluster frame id to camera optical frame id
  geometry_msgs::msg::TransformStamped transform_stamped;
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer_, /*target*/ camera_info.header.frame_id,
      /*source*/ input_cluster_msg.header.frame_id, camera_info.header.stamp);
    if (!transform_stamped_optional) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Failed to get transform from " << input_cluster_msg.header.frame_id << " to "
                                                      << camera_info.header.frame_id);
      return;
    }
    transform_stamped = transform_stamped_optional.value();
  }

  for (std::size_t i = 0; i < input_cluster_msg.feature_objects.size(); ++i) {
    if (input_cluster_msg.feature_objects.at(i).feature.cluster.data.empty()) {
      continue;
    }

    if (is_far_enough(input_cluster_msg.feature_objects.at(i), fusion_distance_)) {
      continue;
    }

    // filter point out of scope
    if (debugger_ && out_of_scope(input_cluster_msg.feature_objects.at(i))) {
      continue;
    }

    sensor_msgs::msg::PointCloud2 transformed_cluster;
    tf2::doTransform(
      input_cluster_msg.feature_objects.at(i).feature.cluster, transformed_cluster,
      transform_stamped);

    auto min_x(static_cast<int32_t>(camera_info.width));
    auto min_y(static_cast<int32_t>(camera_info.height));
    int32_t max_x(0);
    int32_t max_y(0);

    double total_projected_points = 0;
    std::vector<double> point_counter_each_class = {0, 0, 0, 0, 0, 0, 0, 0};

    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cluster, "x"),
         iter_y(transformed_cluster, "y"), iter_z(transformed_cluster, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (*iter_z <= 0.0) {
        continue;
      }

      Eigen::Vector2d projected_point =
        calcRawImageProjectedPoint(pinhole_camera_model, cv::Point3d(*iter_x, *iter_y, *iter_z));

      if (
        0 >= static_cast<int32_t>(projected_point.x()) ||
        static_cast<int32_t>(projected_point.x()) >= static_cast<int32_t>(camera_info.width) - 1 ||
        0 >= static_cast<int32_t>(projected_point.y()) ||
        static_cast<int32_t>(projected_point.y()) >= static_cast<int32_t>(camera_info.height) - 1) {
        continue;
      }

      const uint8_t pixel_value = mask.at<uint8_t>(
        static_cast<uint16_t>(projected_point.y()), static_cast<uint16_t>(projected_point.x()));
      const auto label_id = static_cast<int>(
        static_cast<uint8_t>(input_mask_msg.config.classification[pixel_value - 1].label));
      if (pixel_value != 0) {
        min_x = std::min(static_cast<int32_t>(projected_point.x()), min_x);
        min_y = std::min(static_cast<int32_t>(projected_point.y()), min_y);
        max_x = std::max(static_cast<int32_t>(projected_point.x()), max_x);
        max_y = std::max(static_cast<int32_t>(projected_point.y()), max_y);

        point_counter_each_class[label_id]++;
      }
      total_projected_points++;
    }

    bool all_zero = std::all_of(
      point_counter_each_class.begin(), point_counter_each_class.end(),
      [](double value) { return value == 0; });
    if (all_zero) {
      continue;
    }

    auto max_it =
      std::max_element(point_counter_each_class.begin(), point_counter_each_class.end());
    auto max_index = std::distance(point_counter_each_class.begin(), max_it);

    RCLCPP_DEBUG(get_logger(), "Number of points in the cluster: %f", *max_it);
    RCLCPP_DEBUG(get_logger(), "Total projected points: %f", total_projected_points);
    RCLCPP_DEBUG(get_logger(), "Fusion ratio: %f", *max_it / total_projected_points);

    if (*max_it / total_projected_points < fusion_ratio_) {
      continue;
    }

    // Set classification of the object
    std::vector<autoware_perception_msgs::msg::ObjectClassification> classification;
    autoware_perception_msgs::msg::ObjectClassification object_classification;
    object_classification.label = static_cast<uint8_t>(max_index);
    object_classification.probability = 1.0;
    classification.push_back(object_classification);
    output_object_msg.feature_objects.at(i).object.classification = classification;

    // Set roi of the object
    sensor_msgs::msg::RegionOfInterest roi;
    roi.x_offset = min_x;
    roi.y_offset = min_y;
    roi.width = max_x - min_x;
    roi.height = max_y - min_y;
    output_object_msg.feature_objects.at(i).feature.roi = roi;
  }
}

bool MaskClusterFusionNode::out_of_scope(const DetectedObjectWithFeature & obj)
{
  auto cluster = obj.feature.cluster;
  bool is_out = false;
  auto valid_point = [](float p, float min_num, float max_num) -> bool {
    return (p > min_num) && (p < max_num);
  };

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cluster, "x"), iter_y(cluster, "y"),
       iter_z(cluster, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!valid_point(*iter_x, filter_scope_min_x_, filter_scope_max_x_)) {
      is_out = true;
      break;
    }

    if (!valid_point(*iter_y, filter_scope_min_y_, filter_scope_max_y_)) {
      is_out = true;
      break;
    }

    if (!valid_point(*iter_z, filter_scope_min_z_, filter_scope_max_z_)) {
      is_out = true;
      break;
    }
  }

  return is_out;
}
}  // namespace autoware::image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::image_projection_based_fusion::MaskClusterFusionNode)