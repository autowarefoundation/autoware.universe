// Copyright 2022 TIER IV, Inc.
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

#include "image_projection_based_fusion/pointpainting_fusion/node.hpp"

#include <image_projection_based_fusion/utils/geometry.hpp>
#include <image_projection_based_fusion/utils/utils.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/constants.hpp>

namespace image_projection_based_fusion
{

PointpaintingFusionNode::PointpaintingFusionNode(const rclcpp::NodeOptions & options)
: FusionNode<sensor_msgs::msg::PointCloud2>("pointpainting_fusion", options)
{
  score_threshold_ = this->declare_parameter("score_threshold", 0.4);
  std::string densification_world_frame_id =
    this->declare_parameter("densification_world_frame_id", "map");
  int densification_num_past_frames = this->declare_parameter("densification_num_past_frames", 1);
  std::string trt_precision = this->declare_parameter("trt_precision", "fp16");
  std::string encoder_onnx_path = this->declare_parameter("encoder_onnx_path", "");
  std::string encoder_engine_path = this->declare_parameter("encoder_engine_path", "");
  std::string head_onnx_path = this->declare_parameter("head_onnx_path", "");
  std::string head_engine_path = this->declare_parameter("head_engine_path", "");
  class_names_ = this->declare_parameter<std::vector<std::string>>("class_names");
  rename_car_to_truck_and_bus_ = this->declare_parameter("rename_car_to_truck_and_bus", false);

  NetworkParam encoder_param(encoder_onnx_path, encoder_engine_path, trt_precision);
  NetworkParam head_param(head_onnx_path, head_engine_path, trt_precision);
  DensificationParam densification_param(
    densification_world_frame_id, densification_num_past_frames);
  detector_ptr_ = std::make_unique<CenterPointTRT>(
    class_names_.size(), score_threshold_, encoder_param, head_param, densification_param);

  obj_pub_ptr_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "~/output/objects", rclcpp::QoS{1});
}

void PointpaintingFusionNode::preprocess(sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg)
{
  sensor_msgs::msg::PointCloud2 tmp;
  tmp = painted_pointcloud_msg;

  // set fields
  sensor_msgs::PointCloud2Modifier pcd_modifier(painted_pointcloud_msg);
  pcd_modifier.clear();
  painted_pointcloud_msg.width = tmp.width;
  pcd_modifier.setPointCloud2Fields(
    7, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::FLOAT32, "CAR", 1, sensor_msgs::msg::PointField::FLOAT32,
    "PEDESTRIAN", 1, sensor_msgs::msg::PointField::FLOAT32, "BICYCLE", 1,
    sensor_msgs::msg::PointField::FLOAT32);
  painted_pointcloud_msg.point_step = 28;

  // filter points out of range
  const auto painted_point_step = painted_pointcloud_msg.point_step;
  size_t j = 0;
  sensor_msgs::PointCloud2Iterator<float> iter_painted_x(painted_pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_painted_y(painted_pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_painted_z(painted_pointcloud_msg, "z");
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(tmp, "x"), iter_y(tmp, "y"),
       iter_z(tmp, "z");
       iter_x != iter_x.end();
       ++iter_x, ++iter_y, ++iter_z, ++iter_painted_x, ++iter_painted_y, ++iter_painted_z) {
    if (*iter_x <= -76.8 || *iter_x >= 76.8 || *iter_y <= -76.8 || *iter_y >= 76.8) {
      continue;
    } else {
      *iter_painted_x = *iter_x;
      *iter_painted_y = *iter_y;
      *iter_painted_z = *iter_z;
      j += painted_point_step;
    }
  }
  painted_pointcloud_msg.data.resize(j);
  painted_pointcloud_msg.width = static_cast<uint32_t>(
    painted_pointcloud_msg.data.size() / painted_pointcloud_msg.height /
    painted_pointcloud_msg.point_step);
  painted_pointcloud_msg.row_step =
    static_cast<uint32_t>(painted_pointcloud_msg.data.size() / painted_pointcloud_msg.height);
}

void PointpaintingFusionNode::fuseOnSingleImage(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const int image_id,
  const DetectedObjectsWithFeature & input_roi_msg,
  const sensor_msgs::msg::CameraInfo & camera_info,
  sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg)
{
  std::vector<sensor_msgs::msg::RegionOfInterest> debug_image_rois;
  std::vector<Eigen::Vector2d> debug_image_points;

  // get transform from cluster frame id to camera optical frame id
  geometry_msgs::msg::TransformStamped transform_stamped;
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer_, /*target*/ camera_info.header.frame_id,
      /*source*/ painted_pointcloud_msg.header.frame_id, camera_info.header.stamp);
    if (!transform_stamped_optional) {
      return;
    }
    transform_stamped = transform_stamped_optional.value();
  }

  // get transform from pointcloud frame id to camera optical frame id
  // geometry_msgs::msg::TransformStamped transform_stamped;
  // try {
  //   transform_stamped = tf_buffer_.lookupTransform(
  //     /*target*/ camera_info.header.frame_id,
  //     /*src*/ painted_pointcloud_msg.header.frame_id, tf2::TimePointZero);
  // } catch (tf2::TransformException & ex) {
  //   RCLCPP_WARN(this->get_logger(), "%s", ex.what());
  //   return;
  // }

  // projection matrix
  Eigen::Matrix4d camera_projection;
  camera_projection << camera_info.p.at(0), camera_info.p.at(1), camera_info.p.at(2),
    camera_info.p.at(3), camera_info.p.at(4), camera_info.p.at(5), camera_info.p.at(6),
    camera_info.p.at(7), camera_info.p.at(8), camera_info.p.at(9), camera_info.p.at(10),
    camera_info.p.at(11);

  // transform
  sensor_msgs::msg::PointCloud2 transformed_pointcloud;
  tf2::doTransform(painted_pointcloud_msg, transformed_pointcloud, transform_stamped);

  // projection
  std::vector<Eigen::Vector2d> projected_points;
  projected_points.reserve(painted_pointcloud_msg.data.size());

  // iterate points
  sensor_msgs::PointCloud2Iterator<float> iter_painted_intensity(
    painted_pointcloud_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<float> iter_car(painted_pointcloud_msg, "CAR");
  sensor_msgs::PointCloud2Iterator<float> iter_ped(painted_pointcloud_msg, "PEDESTRIAN");
  sensor_msgs::PointCloud2Iterator<float> iter_bic(painted_pointcloud_msg, "BICYCLE");
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_pointcloud, "x"),
       iter_y(transformed_pointcloud, "y"), iter_z(transformed_pointcloud, "z");
       iter_x != iter_x.end();
       ++iter_x, ++iter_y, ++iter_z, ++iter_painted_intensity, ++iter_car, ++iter_ped, ++iter_bic) {
    // project
    Eigen::Vector4d projected_point =
      camera_projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
    Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(
      projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());

    // iterate 2d bbox
    for (size_t i = 0; i < input_roi_msg.feature_objects.size(); ++i) {
      sensor_msgs::msg::RegionOfInterest roi = input_roi_msg.feature_objects.at(i).feature.roi;
      // paint current point if it is inside bbox
      if (
        normalized_projected_point.x() >= roi.x_offset &&
        normalized_projected_point.x() <= roi.x_offset + roi.width &&
        normalized_projected_point.y() >= roi.y_offset &&
        normalized_projected_point.y() <= roi.y_offset + roi.height &&
        input_roi_msg.feature_objects.at(i).object.classification.front().label !=
          autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN) {
        switch (input_roi_msg.feature_objects.at(i).object.classification.front().label) {
          case autoware_auto_perception_msgs::msg::ObjectClassification::CAR:
            // std::cout << "car" << std::endl;
            *iter_car = 1.0;
            break;
          case autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN:
            *iter_ped = 1.0;
            break;
          case autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE:
            *iter_bic = 1.0;
            break;
        }
        if (debugger_) {
          projected_points.push_back(normalized_projected_point);
          debug_image_points.push_back(normalized_projected_point);
        }
      }
    }
  }

  if (debugger_) {
    for (size_t i = 0; i < input_roi_msg.feature_objects.size(); ++i) {
      debug_image_rois.push_back(input_roi_msg.feature_objects.at(i).feature.roi);
    }
    debugger_->image_rois_ = debug_image_rois;
    debugger_->obstacle_points_ = debug_image_points;
    debugger_->publishImage(image_id, input_roi_msg.header.stamp);
  }
}

void PointpaintingFusionNode::postprocess(sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg)
{
  std::vector<Box3D> det_boxes3d;
  bool is_success = detector_ptr_->detect(painted_pointcloud_msg, tf_buffer_, det_boxes3d);
  if (!is_success) {
    return;
  }

  autoware_auto_perception_msgs::msg::DetectedObjects output_obj_msg;
  output_obj_msg.header = painted_pointcloud_msg.header;
  for (const auto & box3d : det_boxes3d) {
    if (box3d.score < score_threshold_) {
      continue;
    }
    autoware_auto_perception_msgs::msg::DetectedObject obj;
    box3DToDetectedObject(box3d, obj);
    output_obj_msg.objects.emplace_back(obj);
  }

  obj_pub_ptr_->publish(output_obj_msg);
}

void PointpaintingFusionNode::box3DToDetectedObject(
  const Box3D & box3d, autoware_auto_perception_msgs::msg::DetectedObject & obj)
{
  // TODO(yukke42): the value of classification confidence of DNN, not probability.
  obj.existence_probability = box3d.score;

  // classification
  autoware_auto_perception_msgs::msg::ObjectClassification classification;
  classification.probability = 1.0f;
  if (box3d.label >= 0 && static_cast<size_t>(box3d.label) < class_names_.size()) {
    classification.label = getSemanticType(class_names_[box3d.label]);
  } else {
    classification.label = Label::UNKNOWN;
  }

  float l = box3d.length;
  float w = box3d.width;
  if (classification.label == Label::CAR && rename_car_to_truck_and_bus_) {
    // Note: object size is referred from multi_object_tracker
    if ((w * l > 2.2 * 5.5) && (w * l <= 2.5 * 7.9)) {
      classification.label = Label::TRUCK;
    } else if (w * l > 2.5 * 7.9) {
      classification.label = Label::BUS;
    }
  }

  if (isCarLikeVehicleLabel(classification.label)) {
    obj.kinematics.orientation_availability =
      autoware_auto_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
  }

  obj.classification.emplace_back(classification);

  // pose and shape
  // mmdet3d yaw format to ros yaw format
  float yaw = -box3d.yaw - tier4_autoware_utils::pi / 2;
  obj.kinematics.pose_with_covariance.pose.position =
    tier4_autoware_utils::createPoint(box3d.x, box3d.y, box3d.z);
  obj.kinematics.pose_with_covariance.pose.orientation =
    tier4_autoware_utils::createQuaternionFromYaw(yaw);
  obj.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;
  obj.shape.dimensions =
    tier4_autoware_utils::createTranslation(box3d.length, box3d.width, box3d.height);

  // twist
  float vel_x = box3d.vel_x;
  float vel_y = box3d.vel_y;
  geometry_msgs::msg::Twist twist;
  twist.linear.x = std::sqrt(std::pow(vel_x, 2) + std::pow(vel_y, 2));
  twist.angular.z = 2 * (std::atan2(vel_y, vel_x) - yaw);
  obj.kinematics.twist_with_covariance.twist = twist;
  obj.kinematics.has_twist = true;
}

uint8_t PointpaintingFusionNode::getSemanticType(const std::string & class_name)
{
  if (class_name == "CAR") {
    return Label::CAR;
  } else if (class_name == "TRUCK") {
    return Label::TRUCK;
  } else if (class_name == "BUS") {
    return Label::BUS;
  } else if (class_name == "TRAILER") {
    return Label::TRAILER;
  } else if (class_name == "BICYCLE") {
    return Label::BICYCLE;
  } else if (class_name == "MOTORBIKE") {
    return Label::MOTORCYCLE;
  } else if (class_name == "PEDESTRIAN") {
    return Label::PEDESTRIAN;
  } else {
    return Label::UNKNOWN;
  }
}

bool PointpaintingFusionNode::isCarLikeVehicleLabel(const uint8_t label)
{
  return label == Label::CAR || label == Label::TRUCK || label == Label::BUS ||
         label == Label::TRAILER;
}

}  // namespace image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_projection_based_fusion::PointpaintingFusionNode)
