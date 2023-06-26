// Copyright 2020 TIER IV, Inc.
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

#include <image_projection_based_fusion/utils/utils.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <cmath>

// cspell: ignore minx, maxx, miny, maxy, minz, maxz

namespace image_projection_based_fusion
{

RoiClusterFusionNode::RoiClusterFusionNode(const rclcpp::NodeOptions & options)
: FusionNode<DetectedObjectsWithFeature, DetectedObjectWithFeature, Polygon2d>(
    "roi_cluster_fusion", options)
{
  use_iou_x_ = declare_parameter("use_iou_x", true);
  use_iou_y_ = declare_parameter("use_iou_y", false);
  use_iou_ = declare_parameter("use_iou", false);
  use_cluster_semantic_type_ = declare_parameter("use_cluster_semantic_type", false);
  iou_threshold_ = declare_parameter("iou_threshold", 0.1);
  remove_unknown_ = declare_parameter("remove_unknown", false);
  trust_distance_ = declare_parameter("trust_distance", 100.0);
}

void RoiClusterFusionNode::preprocess(DetectedObjectsWithFeature & output_cluster_msg)
{
  // reset cluster semantic type
  if (!use_cluster_semantic_type_) {
    for (auto & feature_object : output_cluster_msg.feature_objects) {
      feature_object.object.classification.front().label =
        autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
      feature_object.object.existence_probability = 0.0;
    }
  }
}

void RoiClusterFusionNode::postprocess(DetectedObjectsWithFeature & output_cluster_msg)
{
  if (!remove_unknown_) {
    return;
  }
  DetectedObjectsWithFeature known_objects;
  known_objects.feature_objects.reserve(output_cluster_msg.feature_objects.size());
  for (auto & feature_object : output_cluster_msg.feature_objects) {
    if (
      feature_object.object.classification.front().label !=
      autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN) {
      known_objects.feature_objects.push_back(feature_object);
    }
  }
  output_cluster_msg.feature_objects = known_objects.feature_objects;
}

void RoiClusterFusionNode::fuseOnSingleImage(
  const DetectedObjectsWithFeature & input_cluster_msg, const std::size_t image_id,
  const DetectedObjectsWithFeature & input_roi_msg,
  const sensor_msgs::msg::CameraInfo & camera_info, DetectedObjectsWithFeature & output_cluster_msg)
{
  std::vector<sensor_msgs::msg::RegionOfInterest> debug_image_rois;
  std::vector<Polygon2d> debug_cluster_rois;
  std::vector<Eigen::Vector2d> debug_image_points;

  Eigen::Matrix4d projection;
  projection << camera_info.p.at(0), camera_info.p.at(1), camera_info.p.at(2), camera_info.p.at(3),
    camera_info.p.at(4), camera_info.p.at(5), camera_info.p.at(6), camera_info.p.at(7),
    camera_info.p.at(8), camera_info.p.at(9), camera_info.p.at(10), camera_info.p.at(11);

  // get transform from cluster frame id to camera optical frame id
  geometry_msgs::msg::TransformStamped transform_stamped;
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer_, /*target*/ camera_info.header.frame_id,
      /*source*/ input_cluster_msg.header.frame_id, camera_info.header.stamp);
    if (!transform_stamped_optional) {
      return;
    }
    transform_stamped = transform_stamped_optional.value();
  }

  std::vector<ClusterRoi> cluster_roi_buffer;
  for (std::size_t i = 0; i < input_cluster_msg.feature_objects.size(); ++i) {
    if (const auto & object = input_cluster_msg.feature_objects.at(i);
        object.feature.cluster.data.empty() || filter_by_distance(object) ||
        (debugger_ && out_of_scope(object))) {
      continue;
    }

    sensor_msgs::msg::PointCloud2 transformed_cluster;
    tf2::doTransform(
      input_cluster_msg.feature_objects.at(i).feature.cluster, transformed_cluster,
      transform_stamped);

    std::vector<Eigen::Vector2d> projected_points;
    Eigen::Vector3d centroid(0.0, 0.0, 0.0);
    projected_points.reserve(transformed_cluster.data.size());
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cluster, "x"),
         iter_y(transformed_cluster, "y"), iter_z(transformed_cluster, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (*iter_z <= 0.0) {
        continue;
      }

      Eigen::Vector4d projected_point =
        projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);

      centroid.x() += projected_point.x();
      centroid.y() += projected_point.y();
      centroid.z() += projected_point.z();

      Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(
        projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());
      if (const int projected_x = static_cast<int>(normalized_projected_point.x()),
          projected_y = static_cast<int>(normalized_projected_point.y());
          0 <= projected_x && projected_x <= static_cast<int>(camera_info.width) - 1 &&
          0 <= projected_y && projected_y <= static_cast<int>(camera_info.height) - 1) {
        projected_points.emplace_back(normalized_projected_point);
        debug_image_points.emplace_back(normalized_projected_point);
      }
    }
    if (projected_points.empty()) {
      continue;
    }

    centroid /= static_cast<double>(projected_points.size());
    PolygonRoi polygon_roi = PolygonRoi(point2ConvexHull(projected_points));
    ClusterRoi cluster_roi{i, polygon_roi, centroid};
    cluster_roi_buffer.emplace_back(cluster_roi);
    debug_cluster_rois.emplace_back(polygon_roi.roi);
  }

  for (const auto & feature_obj : input_roi_msg.feature_objects) {
    size_t index = 0;
    double max_iou = 0.0;
    std::vector<ClusterRoi> candidates;
    for (const auto & cluster_roi : cluster_roi_buffer) {
      double iou(0.0), iou_x(0.0), iou_y(0.0);
      PolygonRoi feature_roi = PolygonRoi(roi2Polygon(feature_obj.feature.roi));
      if (use_iou_) {
        iou = calcIoU(cluster_roi.roi, feature_roi);
      }
      if (use_iou_x_) {
        iou_x = calcIoUX(cluster_roi.roi, feature_roi);
      }
      if (use_iou_y_) {
        iou_y = calcIoUY(cluster_roi.roi, feature_roi);
      }

      if (max_iou < iou + iou_x + iou_y) {
        max_iou = iou + iou_x + iou_y;
        index = cluster_roi.index;
      }

      if (0.0 < iou + iou_x + iou_y) {
        candidates.emplace_back(cluster_roi);
      }
    }

    if (!candidates.empty()) {
      double max_cluster_iou = 0.0;
      ClusterRoi *cluster_ptr_i = nullptr, *cluster_ptr_j = nullptr;
      // calculate iou for each two candidates.
      for (size_t i = 0; i < candidates.size() - 1; ++i) {
        for (size_t j = i + 1; j < candidates.size(); ++j) {
          double cluster_iou = calcIoU(candidates.at(i).roi, candidates.at(j).roi);
          if (max_cluster_iou < cluster_iou) {
            max_cluster_iou = cluster_iou;
            cluster_ptr_i = &candidates.at(i);
            cluster_ptr_j = &candidates.at(j);
          }
        }
      }

      if (0.25 < max_cluster_iou && cluster_ptr_i != nullptr && cluster_ptr_j != nullptr) {
        // occlusion case -> adopt farther object
        const auto & centroid_i = cluster_ptr_i->centroid;
        const auto & centroid_j = cluster_ptr_j->centroid;
        index = std::hypot(centroid_i.x(), centroid_i.y(), centroid_i.z()) <
                    std::hypot(centroid_j.x(), centroid_j.y(), centroid_j.z())
                  ? cluster_ptr_j->index
                  : cluster_ptr_i->index;
      }

      if (auto & cluster_object = output_cluster_msg.feature_objects.at(index).object;
          iou_threshold_ < max_iou &&
          cluster_object.existence_probability <= feature_obj.object.existence_probability &&
          feature_obj.object.classification.front().label !=
            autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN) {
        cluster_object.classification = feature_obj.object.classification;
      }
    }

    debug_image_rois.push_back(feature_obj.feature.roi);
  }

  if (debugger_) {
    debugger_->image_rois_ = debug_image_rois;
    debugger_->obstacle_rois_ = debug_cluster_rois;
    debugger_->obstacle_points_ = debug_image_points;
    debugger_->publishImage(image_id, input_roi_msg.header.stamp);
  }
}

bool RoiClusterFusionNode::out_of_scope(const DetectedObjectWithFeature & obj)
{
  const auto & cluster = obj.feature.cluster;
  bool is_out = false;
  auto valid_point = [](float p, float min_num, float max_num) -> bool {
    return (p > min_num) && (p < max_num);
  };

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cluster, "x"), iter_y(cluster, "y"),
       iter_z(cluster, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!valid_point(*iter_x, filter_scope_minx_, filter_scope_maxx_)) {
      is_out = true;
      break;
    }

    if (!valid_point(*iter_y, filter_scope_miny_, filter_scope_maxy_)) {
      is_out = true;
      break;
    }

    if (!valid_point(*iter_z, filter_scope_minz_, filter_scope_maxz_)) {
      is_out = true;
      break;
    }
  }

  return is_out;
}

bool RoiClusterFusionNode::filter_by_distance(const DetectedObjectWithFeature & obj)
{
  const auto & position = obj.object.kinematics.pose_with_covariance.pose.position;
  const auto square_distance = position.x * position.x + position.y + position.y;
  return square_distance > trust_distance_ * trust_distance_;
}

}  // namespace image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_projection_based_fusion::RoiClusterFusionNode)
