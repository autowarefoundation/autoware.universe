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

#include "tensorrt_rtmdet/tensorrt_rtmdet_node.hpp"

#include <dlfcn.h>

#include <fstream>

namespace tensorrt_rtmdet
{
TrtRTMDetNode::TrtRTMDetNode(const rclcpp::NodeOptions & node_options)
: Node("tensorrt_rtmdet", node_options)
{
  {
    stop_watch_ptr_ =
      std::make_unique<autoware::universe_utils::StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ =
      std::make_unique<autoware::universe_utils::DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  std::string model_path = declare_parameter<std::string>("model_path");
  std::string color_map_path = declare_parameter<std::string>("color_map_path");
  std::string precision = declare_parameter<std::string>("precision");
  std::vector<double> mean = declare_parameter<std::vector<double>>("mean");
  std::vector<double> std = declare_parameter<std::vector<double>>("std");
  int number_classes = declare_parameter<int>("number_classes");
  double score_threshold = declare_parameter<double>("score_threshold");
  double nms_threshold = declare_parameter<double>("nms_threshold");
  double mask_threshold = declare_parameter<double>("mask_threshold");
  std::string calibration_algorithm = declare_parameter<std::string>("calibration_algorithm");
  int dla_core_id = declare_parameter<int>("dla_core_id");
  bool quantize_first_layer = declare_parameter<bool>("quantize_first_layer");
  bool quantize_last_layer = declare_parameter<bool>("quantize_last_layer");
  bool profile_per_layer = declare_parameter<bool>("profile_per_layer");
  double clip_value = declare_parameter<double>("clip_value");
  bool preprocess_on_gpu = declare_parameter<bool>("preprocess_on_gpu");
  //        bool is_publish_debug_image = declare_parameter<bool>("is_publish_debug_image");
  std::string calibration_image_list_path =
    declare_parameter<std::string>("calibration_image_list_path");
  std::vector<std::string> plugin_paths =
    declare_parameter<std::vector<std::string>>("plugin_paths");

  tensorrt_common::BuildConfig build_config(
    calibration_algorithm, dla_core_id, quantize_first_layer, quantize_last_layer,
    profile_per_layer, clip_value);

  const double norm_factor = 1.0;
  const std::string cache_dir = "";
  const tensorrt_common::BatchConfig batch_config{1, 1, 1};
  const size_t max_workspace_size = (1 << 30);

  mean_ = std::vector<float>(mean.begin(), mean.end());
  std_ = std::vector<float>(std.begin(), std.end());

  trt_rtmdet_ = std::make_unique<tensorrt_rtmdet::TrtRTMDet>(
    model_path, precision, number_classes, score_threshold, nms_threshold, mask_threshold,
    build_config, preprocess_on_gpu, calibration_image_list_path, norm_factor, mean_, std_,
    cache_dir, batch_config, max_workspace_size, color_map_path, plugin_paths);

  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&TrtRTMDetNode::onConnect, this));

  objects_pub_ =
    this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithMask>("~/out/objects", 1);

  debug_image_pub_ = image_transport::create_publisher(this, "~/out/debug_image");

  if (declare_parameter("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine file is built and exit.");
    rclcpp::shutdown();
  }
}

void TrtRTMDetNode::onConnect()
{
  using std::placeholders::_1;
  //        if (debug_image_pub_.getNumSubscribers() == 0) {
  //            image_sub_.shutdown();
  //        } else if (!image_sub_) {
  //            image_sub_ = image_transport::create_subscription(
  //                    this, "~/in/image", std::bind(&TrtRTMDetNode::onImage, this, _1), "raw",
  //                    rmw_qos_profile_sensor_data);
  //        }
  image_sub_ = image_transport::create_subscription(
    this, "~/in/image", std::bind(&TrtRTMDetNode::onImage, this, _1), "raw",
    rmw_qos_profile_sensor_data);
}

void TrtRTMDetNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  stop_watch_ptr_->toc("processing_time", true);

  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  tensorrt_rtmdet::ObjectArrays objects;
  tier4_perception_msgs::msg::DetectedObjectsWithMask detected_objects_with_mask;
  if (!trt_rtmdet_->doInference({in_image_ptr->image}, objects, detected_objects_with_mask)) {
    RCLCPP_WARN(this->get_logger(), "Fail to inference");
    return;
  }

  detected_objects_with_mask.header = msg->header;
  objects_pub_->publish(detected_objects_with_mask);

  if (debug_publisher_) {
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - msg->header.stamp).nanoseconds()))
        .count();
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }
}
}  // namespace tensorrt_rtmdet

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tensorrt_rtmdet::TrtRTMDetNode)
