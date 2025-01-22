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

#include "autoware/tensorrt_yolox/tensorrt_yolox_node.hpp"

#include "autoware/object_recognition_utils/object_classification.hpp"
#include "perception_utils/run_length_encoder.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_yolox
{
TrtYoloXNode::TrtYoloXNode(const rclcpp::NodeOptions & node_options)
: Node("tensorrt_yolox", node_options)
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

  const std::string model_path = this->declare_parameter<std::string>("model_path");
  const std::string label_path = this->declare_parameter<std::string>("label_path");
  const std::string precision = this->declare_parameter<std::string>("precision");
  const float score_threshold =
    static_cast<float>(this->declare_parameter<double>("score_threshold"));
  const float nms_threshold = static_cast<float>(this->declare_parameter<double>("nms_threshold"));
  const std::string calibration_algorithm =
    this->declare_parameter<std::string>("calibration_algorithm");
  const int dla_core_id = this->declare_parameter<int>("dla_core_id");
  const bool quantize_first_layer = this->declare_parameter<bool>("quantize_first_layer");
  const bool quantize_last_layer = this->declare_parameter<bool>("quantize_last_layer");
  const bool profile_per_layer = this->declare_parameter<bool>("profile_per_layer");
  const double clip_value = this->declare_parameter<double>("clip_value");
  const bool preprocess_on_gpu = this->declare_parameter<bool>("preprocess_on_gpu");
  const std::string calibration_image_list_path =
    this->declare_parameter<std::string>("calibration_image_list_path");
  const uint8_t gpu_id = this->declare_parameter<uint8_t>("gpu_id");

  std::string color_map_path = this->declare_parameter<std::string>("color_map_path");

  if (!readLabelFile(label_path)) {
    RCLCPP_ERROR(this->get_logger(), "Could not find label file");
    rclcpp::shutdown();
  }

  is_roi_overlap_segment_ = declare_parameter<bool>("is_roi_overlap_segment");
  is_publish_color_mask_ = declare_parameter<bool>("is_publish_color_mask");
  overlap_roi_score_threshold_ = declare_parameter<float>("overlap_roi_score_threshold");
  roi_overlay_segment_labels_.UNKNOWN =
    declare_parameter<bool>("roi_overlay_segment_label.UNKNOWN");
  roi_overlay_segment_labels_.CAR = declare_parameter<bool>("roi_overlay_segment_label.CAR");
  roi_overlay_segment_labels_.TRUCK = declare_parameter<bool>("roi_overlay_segment_label.TRUCK");
  roi_overlay_segment_labels_.BUS = declare_parameter<bool>("roi_overlay_segment_label.BUS");
  roi_overlay_segment_labels_.MOTORCYCLE =
    declare_parameter<bool>("roi_overlay_segment_label.MOTORCYCLE");
  roi_overlay_segment_labels_.BICYCLE =
    declare_parameter<bool>("roi_overlay_segment_label.BICYCLE");
  roi_overlay_segment_labels_.PEDESTRIAN =
    declare_parameter<bool>("roi_overlay_segment_label.PEDESTRIAN");
  roi_overlay_segment_labels_.ANIMAL = declare_parameter<bool>("roi_overlay_segment_label.ANIMAL");
  replaceLabelMap();

  TrtCommonConfig trt_config(
    model_path, precision, "", (1ULL << 30U), dla_core_id, profile_per_layer);

  CalibrationConfig calib_config(
    calibration_algorithm, quantize_first_layer, quantize_last_layer, clip_value);

  const double norm_factor = 1.0;
  const std::string cache_dir = "";

  trt_yolox_ = std::make_unique<tensorrt_yolox::TrtYoloX>(
    trt_config, label_map_.size(), score_threshold, nms_threshold, preprocess_on_gpu, gpu_id,
    calibration_image_list_path, norm_factor, cache_dir, color_map_path, calib_config);

  if (!trt_yolox_->isGPUInitialized()) {
    RCLCPP_ERROR(this->get_logger(), "GPU %d does not exist or is not suitable.", gpu_id);
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "GPU %d is selected for the inference!", gpu_id);

  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&TrtYoloXNode::onConnect, this));

  objects_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "~/out/objects", 1);
  mask_pub_ = image_transport::create_publisher(this, "~/out/mask");
  color_mask_pub_ = image_transport::create_publisher(this, "~/out/color_mask");
  image_pub_ = image_transport::create_publisher(this, "~/out/image");

  if (declare_parameter("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine file is built and exit.");
    rclcpp::shutdown();
  }
}

void TrtYoloXNode::onConnect()
{
  using std::placeholders::_1;
  if (
    objects_pub_->get_subscription_count() == 0 &&
    objects_pub_->get_intra_process_subscription_count() == 0 &&
    image_pub_.getNumSubscribers() == 0 && mask_pub_.getNumSubscribers() == 0 &&
    color_mask_pub_.getNumSubscribers() == 0) {
    image_sub_.shutdown();
  } else if (!image_sub_) {
    image_sub_ = image_transport::create_subscription(
      this, "~/in/image", std::bind(&TrtYoloXNode::onImage, this, _1), "raw",
      rmw_qos_profile_sensor_data);
  }
}

void TrtYoloXNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  stop_watch_ptr_->toc("processing_time", true);
  tier4_perception_msgs::msg::DetectedObjectsWithFeature out_objects;

  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  const auto width = in_image_ptr->image.cols;
  const auto height = in_image_ptr->image.rows;

  tensorrt_yolox::ObjectArrays objects;
  std::vector<cv::Mat> masks = {cv::Mat(cv::Size(height, width), CV_8UC1, cv::Scalar(0))};
  std::vector<cv::Mat> color_masks = {
    cv::Mat(cv::Size(height, width), CV_8UC3, cv::Scalar(0, 0, 0))};

  if (!trt_yolox_->doInference({in_image_ptr->image}, objects, masks, color_masks)) {
    RCLCPP_WARN(this->get_logger(), "Fail to inference");
    return;
  }
  auto & mask = masks.at(0);

  for (const auto & yolox_object : objects.at(0)) {
    tier4_perception_msgs::msg::DetectedObjectWithFeature object;
    object.feature.roi.x_offset = yolox_object.x_offset;
    object.feature.roi.y_offset = yolox_object.y_offset;
    object.feature.roi.width = yolox_object.width;
    object.feature.roi.height = yolox_object.height;
    object.object.existence_probability = yolox_object.score;
    object.object.classification = autoware::object_recognition_utils::toObjectClassifications(
      label_map_[yolox_object.type], 1.0f);
    out_objects.feature_objects.push_back(object);
    const auto left = std::max(0, static_cast<int>(object.feature.roi.x_offset));
    const auto top = std::max(0, static_cast<int>(object.feature.roi.y_offset));
    const auto right =
      std::min(static_cast<int>(object.feature.roi.x_offset + object.feature.roi.width), width);
    const auto bottom =
      std::min(static_cast<int>(object.feature.roi.y_offset + object.feature.roi.height), height);
    cv::rectangle(
      in_image_ptr->image, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255), 3,
      8, 0);
    // Refine mask: replacing segmentation mask by roi class
    // This should remove when the segmentation accuracy is high
    if (is_roi_overlap_segment_ && trt_yolox_->getMultitaskNum() > 0) {
      overlapSegmentByRoi(yolox_object, mask, width, height);
    }
  }
  if (trt_yolox_->getMultitaskNum() > 0) {
    sensor_msgs::msg::Image::SharedPtr out_mask_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, mask)
        .toImageMsg();
    out_mask_msg->header = msg->header;

    std::vector<std::pair<uint8_t, int>> compressed_data = perception_utils::runLengthEncoder(mask);
    int step = sizeof(uint8_t) + sizeof(int);
    out_mask_msg->data.resize(static_cast<int>(compressed_data.size()) * step);
    for (size_t i = 0; i < compressed_data.size(); ++i) {
      std::memcpy(&out_mask_msg->data[i * step], &compressed_data.at(i).first, sizeof(uint8_t));
      std::memcpy(&out_mask_msg->data[i * step + 1], &compressed_data.at(i).second, sizeof(int));
    }
    mask_pub_.publish(out_mask_msg);
  }
  image_pub_.publish(in_image_ptr->toImageMsg());
  out_objects.header = msg->header;
  objects_pub_->publish(out_objects);

  if (debug_publisher_) {
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - out_objects.header.stamp).nanoseconds()))
        .count();
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }

  if (is_publish_color_mask_ && trt_yolox_->getMultitaskNum() > 0) {
    cv::Mat color_mask = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC3);
    trt_yolox_->getColorizedMask(trt_yolox_->getColorMap(), mask, color_mask);
    sensor_msgs::msg::Image::SharedPtr output_color_mask_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, color_mask)
        .toImageMsg();
    output_color_mask_msg->header = msg->header;
    color_mask_pub_.publish(output_color_mask_msg);
  }
}

bool TrtYoloXNode::readLabelFile(const std::string & label_path)
{
  std::ifstream label_file(label_path);
  if (!label_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open label file. [%s]", label_path.c_str());
    return false;
  }
  int label_index{};
  std::string label;
  while (getline(label_file, label)) {
    std::transform(
      label.begin(), label.end(), label.begin(), [](auto c) { return std::toupper(c); });
    label_map_.insert({label_index, label});
    ++label_index;
  }
  return true;
}

void TrtYoloXNode::replaceLabelMap()
{
  for (std::size_t i = 0; i < label_map_.size(); ++i) {
    auto & label = label_map_[i];
    if (label == "PERSON") {
      label = "PEDESTRIAN";
    } else if (label == "MOTORBIKE") {
      label = "MOTORCYCLE";
    } else if (
      label != "CAR" && label != "PEDESTRIAN" && label != "BUS" && label != "TRUCK" &&
      label != "BICYCLE" && label != "MOTORCYCLE") {
      label = "UNKNOWN";
    }
  }
}

int TrtYoloXNode::mapRoiLabel2SegLabel(const int32_t roi_label_index)
{
  if (roi_overlay_segment_labels_.isOverlay(static_cast<uint8_t>(roi_label_index))) {
    std::string label = label_map_[roi_label_index];

    return remap_roi_to_semantic_[label];
  }
  return -1;
}

void TrtYoloXNode::overlapSegmentByRoi(
  const tensorrt_yolox::Object & roi_object, cv::Mat & mask, const int orig_width,
  const int orig_height)
{
  if (roi_object.score < overlap_roi_score_threshold_) return;
  int seg_class_index = mapRoiLabel2SegLabel(roi_object.type);
  if (seg_class_index < 0) return;

  const float scale_x = static_cast<float>(mask.cols) / static_cast<float>(orig_width);
  const float scale_y = static_cast<float>(mask.rows) / static_cast<float>(orig_height);
  const int roi_width = static_cast<int>(roi_object.width * scale_x);
  const int roi_height = static_cast<int>(roi_object.height * scale_y);
  const int roi_x_offset = static_cast<int>(roi_object.x_offset * scale_x);
  const int roi_y_offset = static_cast<int>(roi_object.y_offset * scale_y);

  cv::Mat replace_roi(
    cv::Size(roi_width, roi_height), mask.type(), static_cast<uint8_t>(seg_class_index));
  replace_roi.copyTo(mask.colRange(roi_x_offset, roi_x_offset + roi_width)
                       .rowRange(roi_y_offset, roi_y_offset + roi_height));
}

}  // namespace autoware::tensorrt_yolox

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_yolox::TrtYoloXNode)
