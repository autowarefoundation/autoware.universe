// Copyright 2020 Tier IV, Inc.
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

#ifndef TENSORRT_YOLO__NODELET_THREE_CAMERAS_HPP_
#define TENSORRT_YOLO__NODELET_THREE_CAMERAS_HPP_

#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trt_yolo.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace object_recognition
{
class TensorrtYoloNodeletThreeCameras : public rclcpp::Node
{
public:
  explicit TensorrtYoloNodeletThreeCameras(const rclcpp::NodeOptions & options);
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg0, const sensor_msgs::msg::Image::ConstSharedPtr image_msg1, const sensor_msgs::msg::Image::ConstSharedPtr image_msg2);
  bool readLabelFile(const std::string & filepath, std::vector<std::string> * labels);

private:
  std::mutex connect_mutex_;

  std::vector<image_transport::Publisher> image_pubs_;
  std::vector<rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr> objects_pubs_;
  std::vector<message_filters::Subscriber<sensor_msgs::msg::Image>> image_subs_;

  using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  typename std::shared_ptr<Sync> sync_ptr_;
  
  int batch_size_ = 3;
  rclcpp::TimerBase::SharedPtr timer_;

  yolo::Config yolo_config_;

  std::vector<std::string> labels_;
  std::unique_ptr<float[]> out_scores_;
  std::unique_ptr<float[]> out_boxes_;
  std::unique_ptr<float[]> out_classes_;
  std::unique_ptr<yolo::Net> net_ptr_;

  int out_classes_length_;
  int out_scores_length_;
  int out_boxes_length_;
};

}  // namespace object_recognition

#endif  // TENSORRT_YOLO__NODELET_THREE_CAMERAS_HPP_