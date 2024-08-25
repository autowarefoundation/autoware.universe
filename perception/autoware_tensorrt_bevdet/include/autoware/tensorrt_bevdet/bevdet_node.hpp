// Copyright 2024 TIER IV, Inc.
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
#ifndef AUTOWARE__TENSORRT_BEVDET__BEVDET_NODE_HPP_
#define AUTOWARE__TENSORRT_BEVDET__BEVDET_NODE_HPP_

#include "bevdet.hpp"
#include "cpu_jpegdecoder.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>  // msg2pcl
#include <yaml-cpp/yaml.h>

#include <cassert>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

uint8_t getSemanticType(const std::string & class_name);

void box3DToDetectedObjects(
  const std::vector<Box> & boxes, autoware_perception_msgs::msg::DetectedObjects & objects,
  const std::vector<std::string> & class_names, float score_thre, const bool has_twist);

void getTransform(
  const geometry_msgs::msg::TransformStamped & transform, Eigen::Quaternion<float> & rot,
  Eigen::Translation3f & translation);

void getCameraIntrinsics(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg, Eigen::Matrix3f & intrinsics);

class TRTBEVDetNode : public rclcpp::Node
{
private:
  size_t img_N_;
  int img_w_;
  int img_h_;

  std::string model_config_;

  std::string onnx_file_;
  std::string engine_file_;

  YAML::Node camconfig_;

  std::vector<std::string> imgs_name_;
  std::vector<std::string> class_names_;

  camsData sampleData_;
  std::shared_ptr<BEVDet> bevdet_;

  uchar * imgs_dev_ = nullptr;
  float score_thre_;

  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr pub_boxes_;
  // Subscribers of camera info for each camera, no need to synchonrize
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_f_caminfo_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_fl_caminfo_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_fr_caminfo_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_b_caminfo_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_bl_caminfo_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_br_caminfo_;
  std::vector<bool> caminfo_received_;
  bool camera_info_received_flag_ = false;

  // tf listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Camera parameters;
  std::vector<Eigen::Matrix3f> cams_intrin;
  std::vector<Eigen::Quaternion<float>> cams2ego_rot;
  std::vector<Eigen::Translation3f> cams2ego_trans;

  // Subscribers of images for each camera, synchonrized
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_f_img_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_fl_img_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_fr_img_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_b_img_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_bl_img_;
  message_filters::Subscriber<sensor_msgs::msg::Image> sub_br_img_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>
    MySyncPolicy;

  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  // Timer for checking initialization
  rclcpp::TimerBase::SharedPtr timer_;
  void initParameters();
  void startCameraInfoSubscription();
  void checkInitialization();
  void initModel();
  void startImageSubscription();

public:
  TRTBEVDetNode(const std::string & node_name, const rclcpp::NodeOptions & options);
  ~TRTBEVDetNode();

  void callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_fl_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_f_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_fr_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_bl_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_b_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_br_img);

  void camera_info_callback(int idx, const sensor_msgs::msg::CameraInfo::SharedPtr msg);
};

#endif  // AUTOWARE__TENSORRT_BEVDET__BEVDET_NODE_HPP_
