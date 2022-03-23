// Copyright 2021 Tier IV, Inc.
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

#include "pointpainting/node.hpp"

// #include <config.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pointpainting
{
PointPaintingNode::PointPaintingNode(const rclcpp::NodeOptions & node_options)
: Node("pointpainting", node_options),
  tf_buffer_(this->get_clock())  //, tf_listener_ptr_(tf_buffer_)
{
  score_threshold_ = this->declare_parameter("score_threshold", 0.4);
  pointcloud_sub_.subscribe(this, "~/input/pointcloud", rclcpp::QoS{1}.get_rmw_qos_profile());

  int rois_number = declare_parameter("rois_number", 1);
  if (rois_number < 1) {
    RCLCPP_WARN(this->get_logger(), "minimum roi_num is 1. current roi_num is %d", rois_number);
    rois_number = 1;
  }
  if (8 < rois_number) {
    RCLCPP_WARN(this->get_logger(), "maximum roi_num is 8. current roi_num is %d", rois_number);
    rois_number = 8;
  }

  for (int id = 0; id < rois_number; ++id) {
    std::function<void(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)> fcn =
      std::bind(&PointPaintingNode::cameraInfoCallback, this, std::placeholders::_1, id);
    v_camera_info_sub_.push_back(this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "input/camera_info" + std::to_string(id), rclcpp::QoS{1}.best_effort(), fcn));
  }

  v_roi_sub_.resize(rois_number);
  for (int id = 0; id < static_cast<int>(v_roi_sub_.size()); ++id) {
    v_roi_sub_.at(id) = std::make_shared<
      message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature>>(
      this, "input/rois" + std::to_string(id), rclcpp::QoS{1}.get_rmw_qos_profile());
  }
  // add dummy callback to enable passthrough filter(?)
  v_roi_sub_.at(0)->registerCallback(bind(&PointPaintingNode::dummyCallback, this, _1));
  switch (rois_number) {
    case 1:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), pointcloud_sub_, *v_roi_sub_.at(0), passthrough_, passthrough_,
        passthrough_, passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 2:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), pointcloud_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), passthrough_,
        passthrough_, passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 3:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), pointcloud_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        passthrough_, passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 4:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), pointcloud_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 5:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), pointcloud_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), *v_roi_sub_.at(4), passthrough_, passthrough_, passthrough_);
      break;
    case 6:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), pointcloud_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), *v_roi_sub_.at(4), *v_roi_sub_.at(5), passthrough_, passthrough_);
      break;
    case 7:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), pointcloud_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), *v_roi_sub_.at(4), *v_roi_sub_.at(5), *v_roi_sub_.at(6), passthrough_);
      break;
    case 8:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), pointcloud_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), *v_roi_sub_.at(4), *v_roi_sub_.at(5), *v_roi_sub_.at(6),
        *v_roi_sub_.at(7));
      break;
    default:
      return;
  }

  // use_encoder_trt_ = this->declare_parameter("use_encoder_trt", false);
  // use_head_trt_ = this->declare_parameter("use_head_trt", true);
  // trt_precision_ = this->declare_parameter("trt_precision", "fp16");
  // encoder_onnx_path_ = this->declare_parameter("encoder_onnx_path", "");
  // encoder_engine_path_ = this->declare_parameter("encoder_engine_path", "");
  // encoder_pt_path_ = this->declare_parameter("encoder_pt_path", "");
  // head_onnx_path_ = this->declare_parameter("head_onnx_path", "");
  // head_engine_path_ = this->declare_parameter("head_engine_path", "");
  // head_pt_path_ = this->declare_parameter("head_pt_path", "");
  // class_names_ = this->declare_parameter<std::vector<std::string>>("class_names");
  // rename_car_to_truck_and_bus_ = this->declare_parameter("rename_car_to_truck_and_bus", false);

  // NetworkParam encoder_param(
  //     encoder_onnx_path_, encoder_engine_path_, encoder_pt_path_, trt_precision_,
  //     use_encoder_trt_);
  // NetworkParam head_param(
  //     head_onnx_path_, head_engine_path_, head_pt_path_, trt_precision_, use_head_trt_);
  // DensificationParam densification_param(
  //     densification_world_frame_id, densification_num_past_frames);
  // detector_ptr_ = std::make_unique<CenterPointTRT>(
  //     static_cast<int>(class_names_.size()), encoder_param, head_param, densification_param);

  sync_ptr_->registerCallback(std::bind(
    &PointPaintingNode::fusionCallback, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
    std::placeholders::_7, std::placeholders::_8, std::placeholders::_9));
  objects_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "~/output/objects", rclcpp::QoS{1});
}

void PointPaintingNode::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_camera_info_msg, const int id)
{
  m_camera_info_[id] = *input_camera_info_msg;
}

void PointPaintingNode::fusionCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi0_msg,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi1_msg,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi2_msg,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi3_msg,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi4_msg,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi5_msg,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi6_msg,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi7_msg)
{
  std::cout << "fusionCallback" << std::endl;

  // Guard
  const auto objects_sub_count =
    objects_pub_->get_subscription_count() + objects_pub_->get_intra_process_subscription_count();
  if (objects_sub_count < 1) {
    return;
  }

  // iterate rois
  for (int id = 0; id < static_cast<int>(v_roi_sub_.size()); ++id) {
    // debug variable
    std::vector<sensor_msgs::msg::RegionOfInterest> debug_image_rois;
    std::vector<Eigen::Vector2d> debug_image_points;
    // check camera info
    if (m_camera_info_.find(id) == m_camera_info_.end()) {
      RCLCPP_WARN(this->get_logger(), "no camera info. id is %d", id);
      continue;
    }

    // get yolo result
    tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_roi_msg;
    if (id == 0) {
      input_roi_msg = input_roi0_msg;
    } else if (id == 1) {
      input_roi_msg = input_roi1_msg;
    } else if (id == 2) {
      input_roi_msg = input_roi2_msg;
    } else if (id == 3) {
      input_roi_msg = input_roi3_msg;
    } else if (id == 4) {
      input_roi_msg = input_roi4_msg;
    } else if (id == 5) {
      input_roi_msg = input_roi5_msg;
    } else if (id == 6) {
      input_roi_msg = input_roi6_msg;
    } else if (id == 7) {
      input_roi_msg = input_roi7_msg;
    }

    // projection matrix
    Eigen::Matrix4d projection;
    projection << m_camera_info_.at(id).p.at(0), m_camera_info_.at(id).p.at(1),
      m_camera_info_.at(id).p.at(2), m_camera_info_.at(id).p.at(3), m_camera_info_.at(id).p.at(4),
      m_camera_info_.at(id).p.at(5), m_camera_info_.at(id).p.at(6), m_camera_info_.at(id).p.at(7),
      m_camera_info_.at(id).p.at(8), m_camera_info_.at(id).p.at(9), m_camera_info_.at(id).p.at(10),
      m_camera_info_.at(id).p.at(11);

    // get transform from cluster frame id to camera optical frame id
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        /*target*/ m_camera_info_.at(id).header.frame_id,
        /*src*/ input_pointcloud_msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    // transform
    sensor_msgs::msg::PointCloud2 transformed_pointcloud;
    tf2::doTransform(*input_pointcloud_msg, transformed_pointcloud, transform_stamped);

    // projection
    std::vector<Eigen::Vector2d> projected_points;
    projected_points.reserve(input_pointcloud_msg->data.size());
    std::vector<Eigen::Vector3d> pts_class;
    pts_class.reserve(input_pointcloud_msg->data.size());
    int min_x(m_camera_info_.at(id).width), min_y(m_camera_info_.at(id).height), max_x(0), max_y(0);
    int cnt_range_filtered = 0;
    // iterate points
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_pointcloud, "x"),
         iter_y(transformed_pointcloud, "y"), iter_z(transformed_pointcloud, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      // filter points out of range
      if (
        *iter_z <= 0.0 || *iter_x <= -76.8 || *iter_x >= 76.8 || *iter_y <= -76.8 ||
        *iter_y >= 76.8 || *iter_z >= 6) {
        cnt_range_filtered++;
        continue;
      }
      // project
      Eigen::Vector4d projected_point =
        projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
      Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(
        projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());

      // iterate 2d bbox
      Eigen::Vector3d pt_class = {0., 0., 0.};
      int cls_index;
      for (size_t i = 0; i < input_roi_msg->feature_objects.size(); ++i) {
        sensor_msgs::msg::RegionOfInterest roi = input_roi_msg->feature_objects.at(i).feature.roi;
        // add class if current point is inside bbox
        if (
          normalized_projected_point.x() >= roi.x_offset &&
          normalized_projected_point.x() <= roi.x_offset + roi.width &&
          normalized_projected_point.y() >= roi.y_offset &&
          normalized_projected_point.y() <= roi.y_offset + roi.height &&
          input_roi_msg->feature_objects.at(i).object.classification.front().label !=
            autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN) {
          switch (input_roi_msg->feature_objects.at(i).object.classification.front().label) {
            case autoware_auto_perception_msgs::msg::ObjectClassification::CAR:
              cls_index = 0;
            case autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN:
              cls_index = 1;
            case autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE:
              cls_index = 2;
          }
          pt_class(cls_index) = 1.0;
          debug_image_points.push_back(normalized_projected_point);
        }
      }
    }

    // if (debugger_) {
    //   std::cout << "debug" << std::endl;
    //   debugger_->showImage(
    //     id, input_roi_msg->header.stamp, debug_image_rois, debug_pointcloud_rois,
    //     debug_image_points);
    // }
    // painting

    // run centerpoint
    // std::vector<float> boxes3d_vec = detector_ptr_->detect(*input_pointcloud_msg, tf_buffer_);
  }
}
}  // namespace pointpainting

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointpainting::PointPaintingNode)