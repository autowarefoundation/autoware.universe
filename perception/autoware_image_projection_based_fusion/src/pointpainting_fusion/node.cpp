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

#include "autoware/image_projection_based_fusion/pointpainting_fusion/node.hpp"

#include "autoware/point_types/types.hpp"

#include <autoware/image_projection_based_fusion/utils/geometry.hpp>
#include <autoware/image_projection_based_fusion/utils/utils.hpp>
#include <autoware/lidar_centerpoint/centerpoint_config.hpp>
#include <autoware/lidar_centerpoint/preprocess/pointcloud_densification.hpp>
#include <autoware/lidar_centerpoint/ros_utils.hpp>
#include <autoware/lidar_centerpoint/utils.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/constants.hpp>
#include <autoware/universe_utils/system/time_keeper.hpp>
#include <pcl_ros/transforms.hpp>

#include <omp.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace
{
using autoware::universe_utils::ScopedTimeTrack;

Eigen::Affine3f _transformToEigen(const geometry_msgs::msg::Transform & t)
{
  Eigen::Affine3f a;
  a.matrix() = tf2::transformToEigen(t).matrix().cast<float>();
  return a;
}

}  // namespace

namespace autoware::image_projection_based_fusion
{

inline bool isVehicle(int label2d)
{
  return label2d == autoware_perception_msgs::msg::ObjectClassification::CAR ||
         label2d == autoware_perception_msgs::msg::ObjectClassification::TRUCK ||
         label2d == autoware_perception_msgs::msg::ObjectClassification::TRAILER ||
         label2d == autoware_perception_msgs::msg::ObjectClassification::BUS;
}

inline bool isCar(int label2d)
{
  return label2d == autoware_perception_msgs::msg::ObjectClassification::CAR;
}

inline bool isTruck(int label2d)
{
  return label2d == autoware_perception_msgs::msg::ObjectClassification::TRUCK ||
         label2d == autoware_perception_msgs::msg::ObjectClassification::TRAILER;
}

inline bool isBus(int label2d)
{
  return label2d == autoware_perception_msgs::msg::ObjectClassification::BUS;
}

inline bool isPedestrian(int label2d)
{
  return label2d == autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
}

inline bool isBicycle(int label2d)
{
  return label2d == autoware_perception_msgs::msg::ObjectClassification::BICYCLE ||
         label2d == autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE;
}

inline bool isUnknown(int label2d)
{
  return label2d == autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
}

PointPaintingFusionNode::PointPaintingFusionNode(const rclcpp::NodeOptions & options)
: FusionNode<PointCloudMsgType, RoiMsgType, DetectedObjects>("pointpainting_fusion", options)
{
  omp_num_threads_ = this->declare_parameter<int>("omp_params.num_threads");
  const float score_threshold =
    static_cast<float>(this->declare_parameter<double>("post_process_params.score_threshold"));
  const float circle_nms_dist_threshold = static_cast<float>(
    this->declare_parameter<double>("post_process_params.circle_nms_dist_threshold"));
  const auto yaw_norm_thresholds =
    this->declare_parameter<std::vector<double>>("post_process_params.yaw_norm_thresholds");
  // densification param
  const std::string densification_world_frame_id =
    this->declare_parameter<std::string>("densification_params.world_frame_id");
  const int densification_num_past_frames =
    this->declare_parameter<int>("densification_params.num_past_frames");
  // network param
  const std::string trt_precision = this->declare_parameter<std::string>("trt_precision");
  const std::size_t cloud_capacity = this->declare_parameter<std::int64_t>("cloud_capacity");
  const std::string encoder_onnx_path = this->declare_parameter<std::string>("encoder_onnx_path");
  const std::string encoder_engine_path =
    this->declare_parameter<std::string>("encoder_engine_path");
  const std::string head_onnx_path = this->declare_parameter<std::string>("head_onnx_path");
  const std::string head_engine_path = this->declare_parameter<std::string>("head_engine_path");

  class_names_ = this->declare_parameter<std::vector<std::string>>("model_params.class_names");
  const auto paint_class_names =
    this->declare_parameter<std::vector<std::string>>("model_params.paint_class_names");
  std::vector<std::string> classes_{"CAR", "TRUCK", "BUS", "BICYCLE", "PEDESTRIAN"};
  if (
    std::find(paint_class_names.begin(), paint_class_names.end(), "TRUCK") !=
    paint_class_names.end()) {
    isClassTable_["CAR"] = std::bind(&isCar, std::placeholders::_1);
  } else {
    isClassTable_["CAR"] = std::bind(&isVehicle, std::placeholders::_1);
  }
  isClassTable_["TRUCK"] = std::bind(&isTruck, std::placeholders::_1);
  isClassTable_["BUS"] = std::bind(&isBus, std::placeholders::_1);
  isClassTable_["BICYCLE"] = std::bind(&isBicycle, std::placeholders::_1);
  isClassTable_["PEDESTRIAN"] = std::bind(&isPedestrian, std::placeholders::_1);
  for (const auto & cls : classes_) {
    auto it = find(paint_class_names.begin(), paint_class_names.end(), cls);
    if (it != paint_class_names.end()) {
      int index = it - paint_class_names.begin();
      class_index_[cls] = pow(2, index);  // regard each class as a bit in binary
    } else {
      isClassTable_.erase(cls);
    }
  }
  has_twist_ = this->declare_parameter<bool>("model_params.has_twist");
  has_variance_ = this->declare_parameter<bool>("model_params.has_variance");
  const std::size_t point_feature_size = static_cast<std::size_t>(
    this->declare_parameter<std::int64_t>("model_params.point_feature_size"));
  const std::size_t max_voxel_size =
    static_cast<std::size_t>(this->declare_parameter<std::int64_t>("model_params.max_voxel_size"));
  pointcloud_range = this->declare_parameter<std::vector<double>>("model_params.point_cloud_range");
  const auto voxel_size = this->declare_parameter<std::vector<double>>("model_params.voxel_size");
  const std::size_t downsample_factor = static_cast<std::size_t>(
    this->declare_parameter<std::int64_t>("model_params.downsample_factor"));
  const std::size_t encoder_in_feature_size = static_cast<std::size_t>(
    this->declare_parameter<std::int64_t>("model_params.encoder_in_feature_size"));
  const auto allow_remapping_by_area_matrix =
    this->declare_parameter<std::vector<int64_t>>("allow_remapping_by_area_matrix");
  const auto min_area_matrix = this->declare_parameter<std::vector<double>>("min_area_matrix");
  const auto max_area_matrix = this->declare_parameter<std::vector<double>>("max_area_matrix");

  // subscriber
  std::function<void(const PointCloudMsgType::ConstSharedPtr msg)> sub_callback =
    std::bind(&PointPaintingFusionNode::subCallback, this, std::placeholders::_1);
  det3d_sub_ = this->create_subscription<PointCloudMsgType>(
    "~/input/pointcloud", rclcpp::SensorDataQoS().keep_last(3), sub_callback);

  // publisher
  pub_ptr_ = this->create_publisher<DetectedObjects>("~/output/objects", rclcpp::QoS{1});
  painted_point_pub_ptr_ =
    this->create_publisher<PointCloudMsgType>("~/debug/painted_pointcloud", rclcpp::QoS{1});

  detection_class_remapper_.setParameters(
    allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

  {
    autoware::lidar_centerpoint::NMSParams p;
    p.search_distance_2d_ =
      this->declare_parameter<double>("post_process_params.iou_nms_search_distance_2d");
    p.iou_threshold_ = this->declare_parameter<double>("post_process_params.iou_nms_threshold");
    iou_bev_nms_.setParameters(p);
  }

  autoware::tensorrt_common::TrtCommonConfig encoder_param(
    encoder_onnx_path, trt_precision, encoder_engine_path);
  autoware::tensorrt_common::TrtCommonConfig head_param(
    head_onnx_path, trt_precision, head_engine_path);
  autoware::lidar_centerpoint::DensificationParam densification_param(
    densification_world_frame_id, densification_num_past_frames);
  autoware::lidar_centerpoint::CenterPointConfig config(
    class_names_.size(), point_feature_size, cloud_capacity, max_voxel_size, pointcloud_range,
    voxel_size, downsample_factor, encoder_in_feature_size, score_threshold,
    circle_nms_dist_threshold, yaw_norm_thresholds, has_variance_);

  // create detector
  detector_ptr_ = std::make_unique<image_projection_based_fusion::PointPaintingTRT>(
    encoder_param, head_param, densification_param, config);
  diagnostics_interface_ptr_ =
    std::make_unique<autoware::universe_utils::DiagnosticsInterface>(this, "pointpainting_trt");

  if (this->declare_parameter("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine is built and shutdown node.");
    rclcpp::shutdown();
  }
}

void PointPaintingFusionNode::preprocess(PointCloudMsgType & painted_pointcloud_msg)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (painted_pointcloud_msg.data.empty() || painted_pointcloud_msg.fields.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Empty sensor points!");
    return;
  }

  sensor_msgs::msg::PointCloud2 tmp;
  tmp = painted_pointcloud_msg;

  // set fields
  sensor_msgs::PointCloud2Modifier pcd_modifier(painted_pointcloud_msg);
  pcd_modifier.clear();
  pcd_modifier.reserve(tmp.width);
  painted_pointcloud_msg.width = tmp.width;
  constexpr int num_fields = 5;
  pcd_modifier.setPointCloud2Fields(
    num_fields, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
    sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, "CLASS", 1,
    sensor_msgs::msg::PointField::FLOAT32);
  painted_pointcloud_msg.point_step = num_fields * sizeof(float);
  // filter points out of range
  const auto painted_point_step = painted_pointcloud_msg.point_step;
  size_t j = 0;
  sensor_msgs::PointCloud2Iterator<float> iter_painted_x(painted_pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_painted_y(painted_pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_painted_z(painted_pointcloud_msg, "z");
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(tmp, "x"), iter_y(tmp, "y"),
       iter_z(tmp, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (
      *iter_x <= pointcloud_range.at(0) || *iter_x >= pointcloud_range.at(3) ||
      *iter_y <= pointcloud_range.at(1) || *iter_y >= pointcloud_range.at(4)) {
      continue;
    } else {
      *iter_painted_x = *iter_x;
      *iter_painted_y = *iter_y;
      *iter_painted_z = *iter_z;
      j += painted_point_step;
      ++iter_painted_x;
      ++iter_painted_y;
      ++iter_painted_z;
    }
  }

  painted_pointcloud_msg.data.resize(j);
  painted_pointcloud_msg.width = static_cast<uint32_t>(
    painted_pointcloud_msg.data.size() / painted_pointcloud_msg.height /
    painted_pointcloud_msg.point_step);
  painted_pointcloud_msg.row_step =
    static_cast<uint32_t>(painted_pointcloud_msg.data.size() / painted_pointcloud_msg.height);
}

void PointPaintingFusionNode::fuseOnSingleImage(
  __attribute__((unused)) const PointCloudMsgType & input_pointcloud_msg,
  const Det2dStatus<RoiMsgType> & det2d, const RoiMsgType & input_roi_msg,
  PointCloudMsgType & painted_pointcloud_msg)
{
  if (painted_pointcloud_msg.data.empty() || painted_pointcloud_msg.fields.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Empty sensor points!");
    return;
  }

  auto num_bbox = (input_roi_msg.feature_objects).size();
  if (num_bbox == 0) {
    return;
  }

  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::vector<sensor_msgs::msg::RegionOfInterest> debug_image_rois;
  std::vector<Eigen::Vector2d> debug_image_points;

  // get transform from cluster frame id to camera optical frame id
  // geometry_msgs::msg::TransformStamped transform_stamped;
  Eigen::Affine3f lidar2cam_affine;
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer_, /*target*/ input_roi_msg.header.frame_id,
      /*source*/ painted_pointcloud_msg.header.frame_id, input_roi_msg.header.stamp);
    if (!transform_stamped_optional) {
      return;
    }
    lidar2cam_affine = _transformToEigen(transform_stamped_optional.value().transform);
  }

  const auto x_offset = painted_pointcloud_msg.fields
                          .at(static_cast<size_t>(autoware::point_types::PointXYZIRCIndex::X))
                          .offset;
  const auto y_offset = painted_pointcloud_msg.fields
                          .at(static_cast<size_t>(autoware::point_types::PointXYZIRCIndex::Y))
                          .offset;
  const auto z_offset = painted_pointcloud_msg.fields
                          .at(static_cast<size_t>(autoware::point_types::PointXYZIRCIndex::Z))
                          .offset;
  const auto class_offset = painted_pointcloud_msg.fields.at(4).offset;
  const auto p_step = painted_pointcloud_msg.point_step;

  Eigen::Vector3f point_lidar, point_camera;
  /** dc : don't care

x    | f  x1 x2  dc ||xc|
y  = | y1  f y2  dc ||yc|
dc   | dc dc dc  dc ||zc|
                     |dc|
   **/

  auto objects = input_roi_msg.feature_objects;
  int iterations = painted_pointcloud_msg.data.size() / painted_pointcloud_msg.point_step;
  // iterate points
  // Requires 'OMP_NUM_THREADS=N'
  omp_set_num_threads(omp_num_threads_);
  std::vector<std::vector<Eigen::Vector2d>> local_vectors(omp_num_threads_);
#pragma omp parallel
  {
#pragma omp for
    for (int i = 0; i < iterations; i++) {
      int stride = p_step * i;
      unsigned char * data = &painted_pointcloud_msg.data[0];
      unsigned char * output = &painted_pointcloud_msg.data[0];
      // cppcheck-suppress-begin invalidPointerCast
      float p_x = *reinterpret_cast<const float *>(&data[stride + x_offset]);
      float p_y = *reinterpret_cast<const float *>(&data[stride + y_offset]);
      float p_z = *reinterpret_cast<const float *>(&data[stride + z_offset]);
      // cppcheck-suppress-end invalidPointerCast
      point_lidar << p_x, p_y, p_z;
      point_camera = lidar2cam_affine * point_lidar;
      p_x = point_camera.x();
      p_y = point_camera.y();
      p_z = point_camera.z();

      if (det2d.camera_projector_ptr->isOutsideHorizontalView(p_x, p_z)) {
        continue;
      }

      // project
      Eigen::Vector2d projected_point;
      if (det2d.camera_projector_ptr->calcImageProjectedPoint(
            cv::Point3d(p_x, p_y, p_z), projected_point)) {
        // iterate 2d bbox
        for (const auto & feature_object : objects) {
          sensor_msgs::msg::RegionOfInterest roi = feature_object.feature.roi;
          // paint current point if it is inside bbox
          int label2d = feature_object.object.classification.front().label;
          if (
            !isUnknown(label2d) &&
            isInsideBbox(projected_point.x(), projected_point.y(), roi, 1.0)) {
            // cppcheck-suppress invalidPointerCast
            auto p_class = reinterpret_cast<float *>(&output[stride + class_offset]);
            for (const auto & cls : isClassTable_) {
              // add up the class values if the point belongs to multiple classes
              *p_class = cls.second(label2d) ? (class_index_[cls.first] + *p_class) : *p_class;
            }
          }
          if (debugger_) {
            int thread_id = omp_get_thread_num();
            local_vectors[thread_id].push_back(projected_point);
          }
        }
      }
    }
    if (debugger_) {
      std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
      if (time_keeper_)
        inner_st_ptr = std::make_unique<ScopedTimeTrack>("publish debug message", *time_keeper_);

      for (const auto & feature_object : input_roi_msg.feature_objects) {
        debug_image_rois.push_back(feature_object.feature.roi);
      }

      for (const auto & local_vec : local_vectors) {
        debug_image_points.insert(debug_image_points.end(), local_vec.begin(), local_vec.end());
      }

      debugger_->image_rois_ = debug_image_rois;
      debugger_->obstacle_points_ = debug_image_points;
      debugger_->publishImage(det2d.id, input_roi_msg.header.stamp);
    }
  }
}

void PointPaintingFusionNode::postprocess(
  const PointCloudMsgType & painted_pointcloud_msg, DetectedObjects & output_msg)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);
  diagnostics_interface_ptr_->clear();

  output_msg.header = painted_pointcloud_msg.header;
  output_msg.objects.clear();

  const auto objects_sub_count =
    pub_ptr_->get_subscription_count() + pub_ptr_->get_intra_process_subscription_count();
  if (objects_sub_count < 1) {
    return;
  }

  if (painted_pointcloud_msg.data.empty() || painted_pointcloud_msg.fields.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Empty sensor points!");
    return;
  }

  std::vector<autoware::lidar_centerpoint::Box3D> det_boxes3d;
  bool is_num_pillars_within_range = true;
  bool is_success = detector_ptr_->detect(
    painted_pointcloud_msg, tf_buffer_, det_boxes3d, is_num_pillars_within_range);
  if (!is_success) {
    return;
  }
  diagnostics_interface_ptr_->add_key_value(
    "is_num_pillars_within_range", is_num_pillars_within_range);
  if (!is_num_pillars_within_range) {
    std::stringstream message;
    message << "PointPaintingTRT::detect: The actual number of pillars exceeds its maximum value, "
            << "which may limit the detection performance.";
    diagnostics_interface_ptr_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }

  std::vector<autoware_perception_msgs::msg::DetectedObject> raw_objects;
  raw_objects.reserve(det_boxes3d.size());
  for (const auto & box3d : det_boxes3d) {
    autoware_perception_msgs::msg::DetectedObject obj;
    box3DToDetectedObject(box3d, class_names_, has_twist_, has_variance_, obj);
    raw_objects.emplace_back(obj);
  }

  // prepare output message
  output_msg.objects = iou_bev_nms_.apply(raw_objects);

  detection_class_remapper_.mapClasses(output_msg);

  // publish debug message: painted pointcloud
  if (debugger_ && painted_point_pub_ptr_->get_subscription_count() > 0) {
    painted_point_pub_ptr_->publish(painted_pointcloud_msg);
  }
  diagnostics_interface_ptr_->publish(painted_pointcloud_msg.header.stamp);
}

}  // namespace autoware::image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::image_projection_based_fusion::PointPaintingFusionNode)
