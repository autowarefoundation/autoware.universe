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

#include "tensorrt_yolox/tensorrt_yolox_node.hpp"

#include "object_recognition_utils/object_classification.hpp"

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace tensorrt_yolox
{
TrtYoloXNode::TrtYoloXNode(const rclcpp::NodeOptions & node_options)
: Node("tensorrt_yolox", node_options)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  auto declare_parameter_with_description =
    [this](std::string name, auto default_val, std::string description = "") {
      auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
      param_desc.description = description;
      return this->declare_parameter(name, default_val, param_desc);
    };

  std::string model_path =
    declare_parameter_with_description("model_path", "", "The onnx file name for YOLOX model");
  std::string label_path = declare_parameter_with_description(
    "label_path", "",
    "The label file that consists of label name texts for detected object categories");
  std::string precision = declare_parameter_with_description(
    "precision", "fp32",
    "operation precision to be used on inference. Valid value is one of: [fp32, fp16, int8]");
  float score_threshold = declare_parameter_with_description(
    "score_threshold", 0.3,
    ("Objects with a score lower than this value will be ignored. "
     "This threshold will be ignored if specified model contains EfficientNMS_TRT module in it"));
  float nms_threshold = declare_parameter_with_description(
    "nms_threshold", 0.7,
    ("Detection results will be ignored if IoU over this value. "
     "This threshold will be ignored if specified model contains EfficientNMS_TRT module in it"));
  std::string calibration_algorithm = declare_parameter_with_description(
    "calibration_algorithm", "MinMax",
    ("Calibration algorithm to be used for quantization when precision==int8. "
     "Valid value is one of: [Entropy, (Legacy | Percentile), MinMax]"));
  int dla_core_id = declare_parameter_with_description(
    "dla_core_id", -1,
    "If positive ID value is specified, the node assign inference task to the DLA core");
  bool quantize_first_layer = declare_parameter_with_description(
    "quantize_first_layer", false,
    ("If true, set the operating precision for the first (input) layer to be fp16. "
     "This option is valid only when precision==int8"));
  bool quantize_last_layer = declare_parameter_with_description(
    "quantize_last_layer", false,
    ("If true, set the operating precision for the last (output) layer to be fp16. "
     "This option is valid only when precision==int8"));
  bool profile_per_layer = declare_parameter_with_description(
    "profile_per_layer", false,
    ("If true, profiler function will be enabled. "
     "Since the profile function may affect execution speed, it is recommended "
     "to set this flag true only for development purpose."));
  double clip_value = declare_parameter_with_description(
    "clip_value", 0.0,
    ("If positive value is specified, "
     "the value of each layer output will be clipped between [0.0, clip_value]. "
     "This option is valid only when precision==int8 and used to manually specify "
     "the dynamic range instead of using any calibration"));
  bool preprocess_on_gpu = declare_parameter_with_description(
    "preprocess_on_gpu", true, "If true, pre-processing is performed on GPU");
  std::string calibration_image_list_path = declare_parameter_with_description(
    "calibration_image_list_path", "",
    ("Path to a file which contains path to images."
     "Those images will be used for int8 quantization."));

  std::string color_map_path = declare_parameter_with_description(
    "color_map_path", "", ("Path to a file which contains path to color map."));
  if (!readLabelFile(label_path)) {
    RCLCPP_ERROR(this->get_logger(), "Could not find label file");
    rclcpp::shutdown();
  }

  is_roi_overlap_segment_ = declare_parameter<bool>("is_roi_overlap_segment");
  is_publish_color_mask_ = declare_parameter<bool>("is_publish_color_mask");
  overlap_roi_score_threshold_ = declare_parameter<float>("overlap_roi_score_threshold");
  replaceLabelMap();

  tensorrt_common::BuildConfig build_config(
    calibration_algorithm, dla_core_id, quantize_first_layer, quantize_last_layer,
    profile_per_layer, clip_value);

  trt_yolox_ = std::make_unique<tensorrt_yolox::TrtYoloX>(
    model_path, precision, color_map_path, label_map_.size(), score_threshold, nms_threshold,
    build_config, preprocess_on_gpu, is_publish_color_mask_, calibration_image_list_path);

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
  cv::resize(
    mask, mask, cv::Size(in_image_ptr->image.cols, in_image_ptr->image.rows), 0, 0,
    cv::INTER_NEAREST);

  for (const auto & yolox_object : objects.at(0)) {
    tier4_perception_msgs::msg::DetectedObjectWithFeature object;
    object.feature.roi.x_offset = yolox_object.x_offset;
    object.feature.roi.y_offset = yolox_object.y_offset;
    object.feature.roi.width = yolox_object.width;
    object.feature.roi.height = yolox_object.height;
    object.object.existence_probability = yolox_object.score;
    object.object.classification =
      object_recognition_utils::toObjectClassifications(label_map_[yolox_object.type], 1.0f);
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
    if (is_roi_overlap_segment_) {
      overlapSegmentByRoi(yolox_object, mask);
    }
  }
  sensor_msgs::msg::Image::SharedPtr out_mask_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, mask)
      .toImageMsg();
  out_mask_msg->header = msg->header;
  mask_pub_.publish(out_mask_msg);

  image_pub_.publish(in_image_ptr->toImageMsg());
  out_objects.header = msg->header;
  objects_pub_->publish(out_objects);

  if (is_publish_color_mask_) {
    cv::Mat color_mask =
      cv::Mat::zeros(in_image_ptr->image.rows, in_image_ptr->image.cols, CV_8UC3);
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
  switch (roi_label_index) {
    case 0:
      return 5;
    case 1:
      return 13;
    case 2:
      return 14;
    case 3:
      return 15;
    case 4:
      return 18;
    case 5:
      return 17;
    case 6:
      return 11;
    default:
      return -1;
  }
  return -1;
}

void TrtYoloXNode::overlapSegmentByRoi(const tensorrt_yolox::Object & roi_object, cv::Mat & mask)
{
  if (roi_object.score < overlap_roi_score_threshold_) return;
  int seg_class_index = mapRoiLabel2SegLabel(roi_object.type);
  if (seg_class_index < 0) return;
  cv::Mat replace_roi(
    cv::Size(roi_object.width, roi_object.height), mask.type(),
    static_cast<uint8_t>(seg_class_index));
  replace_roi.copyTo(mask.colRange(roi_object.x_offset, roi_object.x_offset + roi_object.width)
                       .rowRange(roi_object.y_offset, roi_object.y_offset + roi_object.height));
}

}  // namespace tensorrt_yolox

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tensorrt_yolox::TrtYoloXNode)
