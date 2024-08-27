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

// cspell:ignore BEVDET, thre, TRTBEV, bevdet, caminfo, intrin
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

// Get the rotation and translation from a geometry_msgs::msg::TransformStamped
void getTransform(
  const geometry_msgs::msg::TransformStamped & transform, Eigen::Quaternion<float> & rot,
  Eigen::Translation3f & translation);

// Get the camera intrinsics from a sensor_msgs::msg::CameraInfo
void getCameraIntrinsics(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg, Eigen::Matrix3f & intrinsics);

class TRTBEVDetNode : public rclcpp::Node
{
  /**
   * @class TRTBEVDetNode
   * @brief This class represents a node for performing object detection using TensorRT on BEV
   * (Bird's Eye View) images.
   */
private:
  size_t img_N_;  ///< Number of images
  int img_w_;     ///< Width of the images
  int img_h_;     ///< Height of the images

  std::string model_config_;  ///< Path to the model configuration file

  std::string onnx_file_;    ///< Path to the ONNX file
  std::string engine_file_;  ///< Path to the TensorRT engine file

  std::vector<std::string> imgs_name_;    ///< Names of the images
  std::vector<std::string> class_names_;  ///< Names of the object classes

  camsData sampleData_;             ///< Sample data for camera parameters
  std::shared_ptr<BEVDet> bevdet_;  ///< Object for performing object detection

  uchar * imgs_dev_ = nullptr;  ///< Device pointer for storing the images
  float score_thre_;            ///< Score threshold for object detection

  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr
    pub_boxes_;  ///< Publisher for publishing the detected objects

  // Subscribers of camera info for each camera, no need to synchronize
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    sub_f_caminfo_;  ///< Subscriber for front camera info
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    sub_fl_caminfo_;  ///< Subscriber for front-left camera info
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    sub_fr_caminfo_;  ///< Subscriber for front-right camera info
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    sub_b_caminfo_;  ///< Subscriber for back camera info
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    sub_bl_caminfo_;  ///< Subscriber for back-left camera info
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
    sub_br_caminfo_;  ///< Subscriber for back-right camera info
  std::vector<bool>
    caminfo_received_;  ///< Flag indicating if camera info has been received for each camera
  bool camera_info_received_flag_ =
    false;  ///< Flag indicating if camera info has been received for all cameras

  // tf listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{
    nullptr};                                   ///< TF listener for transforming coordinates
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;  ///< Buffer for storing TF transforms

  // Camera parameters;
  std::vector<Eigen::Matrix3f> cams_intrin;  ///< Intrinsic camera parameters for each camera
  std::vector<Eigen::Quaternion<float>>
    cams2ego_rot;  ///< Rotation from camera frame to ego frame for each camera
  std::vector<Eigen::Translation3f>
    cams2ego_trans;  ///< Translation from camera frame to ego frame for each camera

  // Subscribers of images for each camera, synchronized
  message_filters::Subscriber<sensor_msgs::msg::Image>
    sub_f_img_;  ///< Subscriber for front camera image
  message_filters::Subscriber<sensor_msgs::msg::Image>
    sub_fl_img_;  ///< Subscriber for front-left camera image
  message_filters::Subscriber<sensor_msgs::msg::Image>
    sub_fr_img_;  ///< Subscriber for front-right camera image
  message_filters::Subscriber<sensor_msgs::msg::Image>
    sub_b_img_;  ///< Subscriber for back camera image
  message_filters::Subscriber<sensor_msgs::msg::Image>
    sub_bl_img_;  ///< Subscriber for back-left camera image
  message_filters::Subscriber<sensor_msgs::msg::Image>
    sub_br_img_;  ///< Subscriber for back-right camera image

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>
    MySyncPolicy;

  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;  ///< Synchronizer for synchronizing image callbacks

  // Timer for checking initialization
  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for checking initialization

  /**
   * @brief Starts the subscription to camera info topics for each camera.
   */
  void startCameraInfoSubscription();

  /**
   * @brief Checks if the node has been initialized properly.
   */
  void checkInitialization();

  /**
   * @brief Initializes the object detection model.
   */
  void initModel();

  /**
   * @brief Starts the subscription to image topics for each camera.
   */
  void startImageSubscription();

public:
  /**
   * @brief Constructor for TRTBEVDetNode.
   * @param node_name The name of the node.
   * @param options The options for the node.
   */
  TRTBEVDetNode(const std::string & node_name, const rclcpp::NodeOptions & options);

  /**
   * @brief Destructor for TRTBEVDetNode.
   */
  ~TRTBEVDetNode();

  /**
   * @brief Callback function for synchronized image messages.
   * @param msg_fl_img The front-left camera image message.
   * @param msg_f_img The front camera image message.
   * @param msg_fr_img The front-right camera image message.
   * @param msg_bl_img The back-left camera image message.
   * @param msg_b_img The back camera image message.
   * @param msg_br_img The back-right camera image message.
   */
  void callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_fl_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_f_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_fr_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_bl_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_b_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_br_img);

  /**
   * @brief Callback function for camera info messages. This function also reads from TF to get the
   * transformation from the camera frame to the ego frame.
   * @param idx The index of the camera.
   * @param msg The camera info message.
   */
  void camera_info_callback(int idx, const sensor_msgs::msg::CameraInfo::SharedPtr msg);
};

#endif  // AUTOWARE__TENSORRT_BEVDET__BEVDET_NODE_HPP_
