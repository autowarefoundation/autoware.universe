// Copyright 2023 TIER IV, Inc.
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

#include "traffic_light_fine_detector_node.hpp"

#include <map>

#if (defined(_MSC_VER) or (defined(__GNUC__) and (7 <= __GNUC_MAJOR__)))
#include <filesystem>
namespace fs = ::std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = ::std::experimental::filesystem;
#endif

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
float calWeightedIou(
  const sensor_msgs::msg::RegionOfInterest & bbox1, const autoware::tensorrt_yolox::Object & bbox2)
{
  int x1 = std::max(static_cast<int>(bbox1.x_offset), bbox2.x_offset);
  int x2 = std::min(static_cast<int>(bbox1.x_offset + bbox1.width), bbox2.x_offset + bbox2.width);
  int y1 = std::max(static_cast<int>(bbox1.y_offset), bbox2.y_offset);
  int y2 = std::min(static_cast<int>(bbox1.y_offset + bbox1.height), bbox2.y_offset + bbox2.height);
  int area1 = std::max(x2 - x1, 0) * std::max(y2 - y1, 0);
  int area2 = bbox1.width * bbox1.height + bbox2.width * bbox2.height - area1;
  if (area2 == 0) {
    return 0.0;
  }
  return bbox2.score * area1 / area2;
}

}  // namespace

namespace autoware::traffic_light
{
TrafficLightFineDetectorNode::TrafficLightFineDetectorNode(const rclcpp::NodeOptions & options)
: Node("traffic_light_fine_detector_node", options)
{
  int num_class = 2;
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  std::string model_path = declare_parameter("fine_detector_model_path", "");
  std::string label_path = declare_parameter("fine_detector_label_path", "");
  std::string precision = declare_parameter("fine_detector_precision", "fp16");
  const uint8_t gpu_id = declare_parameter("gpu_id", 0);
  // Objects with a score lower than this value will be ignored.
  // This threshold will be ignored if specified model contains EfficientNMS_TRT module in it
  score_thresh_ = declare_parameter("fine_detector_score_thresh", 0.3);
  // Detection results will be ignored if IoU over this value.
  // This threshold will be ignored if specified model contains EfficientNMS_TRT module in it
  float nms_threshold = declare_parameter("fine_detector_nms_thresh", 0.65);
  is_approximate_sync_ = this->declare_parameter<bool>("approximate_sync", false);

  if (!readLabelFile(label_path, tlr_label_id_, num_class)) {
    RCLCPP_ERROR(this->get_logger(), "Could not find tlr id");
  }

  const bool cuda_preprocess = true;
  const std::string calib_image_list = "";
  const double scale = 1.0;
  const std::string cache_dir = "";

  auto trt_config = autoware::tensorrt_common::TrtCommonConfig(model_path, precision);

  trt_yolox_ = std::make_unique<autoware::tensorrt_yolox::TrtYoloX>(
    trt_config, num_class, score_thresh_, nms_threshold, cuda_preprocess, gpu_id, calib_image_list,
    scale, cache_dir);

  batch_size_ = trt_yolox_->getBatchSize();

  if (!trt_yolox_->isGPUInitialized()) {
    RCLCPP_ERROR(this->get_logger(), "GPU %d does not exist or is not suitable.", gpu_id);
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "GPU %d is selected for the inference!", gpu_id);

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&TrafficLightFineDetectorNode::connectCb, this));

  std::lock_guard<std::mutex> lock(connect_mutex_);
  output_roi_pub_ = this->create_publisher<TrafficLightRoiArray>("~/output/rois", 1);
  exe_time_pub_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
    "~/debug/exe_time_ms", 1);
  if (is_approximate_sync_) {
    approximate_sync_.reset(
      new ApproximateSync(ApproximateSyncPolicy(10), image_sub_, rough_roi_sub_, expect_roi_sub_));
    approximate_sync_->registerCallback(
      std::bind(&TrafficLightFineDetectorNode::callback, this, _1, _2, _3));
  } else {
    sync_.reset(new Sync(SyncPolicy(10), image_sub_, rough_roi_sub_, expect_roi_sub_));
    sync_->registerCallback(std::bind(&TrafficLightFineDetectorNode::callback, this, _1, _2, _3));
  }

  if (declare_parameter("build_only", false)) {
    RCLCPP_INFO(get_logger(), "TensorRT engine is built and shutdown node.");
    rclcpp::shutdown();
  }
}

void TrafficLightFineDetectorNode::connectCb()
{
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (output_roi_pub_->get_subscription_count() == 0) {
    image_sub_.unsubscribe();
    rough_roi_sub_.unsubscribe();
    expect_roi_sub_.unsubscribe();
  } else if (!image_sub_.getSubscriber()) {
    image_sub_.subscribe(this, "~/input/image", "raw", rmw_qos_profile_sensor_data);
    rough_roi_sub_.subscribe(this, "~/input/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
    expect_roi_sub_.subscribe(this, "~/expect/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
  }
}

void TrafficLightFineDetectorNode::callback(
  const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg,
  const TrafficLightRoiArray::ConstSharedPtr rough_roi_msg,
  const TrafficLightRoiArray::ConstSharedPtr expect_roi_msg)
{
  if (in_image_msg->width < 2 || in_image_msg->height < 2) {
    return;
  }

  using std::chrono::high_resolution_clock;
  using std::chrono::milliseconds;
  const auto exe_start_time = high_resolution_clock::now();
  cv::Mat original_image;
  TrafficLightRoiArray out_rois;
  std::map<int, TrafficLightRoi> id2expectRoi;
  std::map<int, autoware::tensorrt_yolox::ObjectArray> id2detections;
  for (const auto & expect_roi : expect_roi_msg->rois) {
    id2expectRoi[expect_roi.traffic_light_id] = expect_roi;
  }

  rosMsg2CvMat(in_image_msg, original_image, "bgr8");
  std::vector<cv::Rect> rois;
  autoware::tensorrt_yolox::ObjectArrays inference_results;
  std::vector<cv::Point> lts;
  std::vector<size_t> roi_ids;

  for (size_t roi_i = 0; roi_i < rough_roi_msg->rois.size(); roi_i++) {
    const auto & rough_roi = rough_roi_msg->rois[roi_i];
    cv::Point lt(rough_roi.roi.x_offset, rough_roi.roi.y_offset);
    cv::Point rb(
      rough_roi.roi.x_offset + rough_roi.roi.width, rough_roi.roi.y_offset + rough_roi.roi.height);
    fitInFrame(lt, rb, cv::Size(original_image.size()));
    rois.emplace_back(lt, rb);
    lts.emplace_back(lt);
    roi_ids.emplace_back(rough_roi.traffic_light_id);
    // keep the actual batch size
    size_t true_batch_size = rois.size();
    // insert fake rois since the TRT model requires static batch size
    if (roi_i + 1 == rough_roi_msg->rois.size()) {
      while (static_cast<int>(rois.size()) < batch_size_) {
        rois.emplace_back(rois.front());
      }
    }
    if (static_cast<int>(rois.size()) == batch_size_) {
      trt_yolox_->doMultiScaleInference(original_image, inference_results, rois);
      for (size_t batch_i = 0; batch_i < true_batch_size; batch_i++) {
        for (const autoware::tensorrt_yolox::Object & detection : inference_results[batch_i]) {
          if (detection.score < score_thresh_) {
            continue;
          }
          cv::Point lt_roi(
            lts[batch_i].x + detection.x_offset, lts[batch_i].y + detection.y_offset);
          cv::Point rb_roi(lt_roi.x + detection.width, lt_roi.y + detection.height);
          fitInFrame(lt_roi, rb_roi, cv::Size(original_image.size()));
          autoware::tensorrt_yolox::Object det = detection;
          det.x_offset = lt_roi.x;
          det.y_offset = lt_roi.y;
          det.width = rb_roi.x - lt_roi.x;
          det.height = rb_roi.y - lt_roi.y;
          det.type = detection.type;
          id2detections[roi_ids[batch_i]].push_back(det);
        }
      }
      rois.clear();
      lts.clear();
      inference_results.clear();
      roi_ids.clear();
    }
  }
  detectionMatch(id2expectRoi, id2detections, out_rois);
  out_rois.header = rough_roi_msg->header;
  output_roi_pub_->publish(out_rois);
  const auto exe_end_time = high_resolution_clock::now();
  const double exe_time =
    std::chrono::duration_cast<milliseconds>(exe_end_time - exe_start_time).count();
  autoware_internal_debug_msgs::msg::Float32Stamped exe_time_msg;
  exe_time_msg.data = exe_time;
  exe_time_msg.stamp = this->now();
  exe_time_pub_->publish(exe_time_msg);
}

float TrafficLightFineDetectorNode::evalMatchScore(
  const std::map<int, TrafficLightRoi> & id2expectRoi,
  std::map<int, autoware::tensorrt_yolox::ObjectArray> & id2detections,
  std::map<int, autoware::tensorrt_yolox::Object> & id2bestDetection)
{
  float score_sum = 0.0f;
  id2bestDetection.clear();
  for (const auto & roi_p : id2expectRoi) {
    int tlr_id = roi_p.first;
    float max_score = 0.0f;
    const sensor_msgs::msg::RegionOfInterest & expected_roi = roi_p.second.roi;
    for (const autoware::tensorrt_yolox::Object & detection : id2detections[tlr_id]) {
      float score = ::calWeightedIou(expected_roi, detection);
      if (score >= max_score) {
        max_score = score;
        id2bestDetection[tlr_id] = detection;
      }
    }
    score_sum += max_score;
  }
  return score_sum;
}

void TrafficLightFineDetectorNode::detectionMatch(
  const std::map<int, TrafficLightRoi> & id2expectRoi,
  std::map<int, autoware::tensorrt_yolox::ObjectArray> & id2detections,
  TrafficLightRoiArray & out_rois)
{
  float max_score = 0.0f;
  std::map<int, autoware::tensorrt_yolox::Object> bestDetections;
  for (const auto & roi_pair : id2expectRoi) {
    int tlr_id = roi_pair.first;
    // the expected roi calculated from tf information
    const sensor_msgs::msg::RegionOfInterest & expect_roi = roi_pair.second.roi;
    int expect_cx = expect_roi.x_offset + expect_roi.width / 2;
    int expect_cy = expect_roi.y_offset + expect_roi.height / 2;
    for (const autoware::tensorrt_yolox::Object & det : id2detections[tlr_id]) {
      // for every detection, calculate the center offset between the detection and the
      // corresponding expected roi
      int det_cx = det.x_offset + det.width / 2;
      int det_cy = det.y_offset + det.height / 2;
      int dx = det_cx - expect_cx;
      int dy = det_cy - expect_cy;
      // transfer all the rough rois by the offset
      std::map<int, TrafficLightRoi> id2expectRoi_copy = id2expectRoi;
      for (auto & p : id2expectRoi_copy) {
        p.second.roi.x_offset += dx;
        p.second.roi.y_offset += dy;
      }
      // calculate the "match score" between expected rois and id2detections_copy
      std::map<int, autoware::tensorrt_yolox::Object> id2bestDetection;
      float score = evalMatchScore(id2expectRoi_copy, id2detections, id2bestDetection);
      if (score > max_score) {
        max_score = score;
        bestDetections = id2bestDetection;
      }
    }
  }

  out_rois.rois.clear();
  std::vector<size_t> invalid_roi_id;
  for (const auto & [tlr_id, roi] : id2expectRoi) {
    // if matches, update the roi info
    if (!bestDetections.count(tlr_id)) {
      invalid_roi_id.emplace_back(tlr_id);
      continue;
    }
    TrafficLightRoi tlr;
    tlr.traffic_light_id = tlr_id;
    const auto & object = bestDetections.at(tlr_id);
    tlr.traffic_light_type = roi.traffic_light_type;
    tlr.roi.x_offset = object.x_offset;
    tlr.roi.y_offset = object.y_offset;
    tlr.roi.width = object.width;
    tlr.roi.height = object.height;
    out_rois.rois.push_back(tlr);
  }
  // append undetected rois at the end
  for (const auto & id : invalid_roi_id) {
    TrafficLightRoi tlr;
    tlr.traffic_light_id = id;
    tlr.traffic_light_type = id2expectRoi.at(id).traffic_light_type;
    out_rois.rois.push_back(tlr);
  }
}

bool TrafficLightFineDetectorNode::rosMsg2CvMat(
  const sensor_msgs::msg::Image::ConstSharedPtr image_msg, cv::Mat & image, std::string encode)
{
  try {
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, encode);
    image = cv_image->image;
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to convert sensor_msgs::msg::Image to cv::Mat \n%s", e.what());
    return false;
  }

  return true;
}

bool TrafficLightFineDetectorNode::fitInFrame(cv::Point & lt, cv::Point & rb, const cv::Size & size)
{
  const int width = static_cast<int>(size.width);
  const int height = static_cast<int>(size.height);
  {
    const int x_min = 0, x_max = width - 2;
    const int y_min = 0, y_max = height - 2;
    lt.x = std::min(std::max(lt.x, x_min), x_max);
    lt.y = std::min(std::max(lt.y, y_min), y_max);
  }
  {
    const int x_min = lt.x + 1, x_max = width - 1;
    const int y_min = lt.y + 1, y_max = height - 1;
    rb.x = std::min(std::max(rb.x, x_min), x_max);
    rb.y = std::min(std::max(rb.y, y_min), y_max);
  }

  return true;
}

bool TrafficLightFineDetectorNode::readLabelFile(
  const std::string & filepath, std::vector<int> & tlr_label_id_, int & num_class)
{
  std::ifstream labelsFile(filepath);
  if (!labelsFile.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open label file. [%s]", filepath.c_str());
    return false;
  }
  std::string label;
  int idx = 0;
  while (getline(labelsFile, label)) {
    if (label == "traffic_light" || label == "pedestrian_traffic_light") {
      tlr_label_id_.push_back(idx);
    }
    idx++;
  }
  num_class = idx;
  return tlr_label_id_.size() != 0;
}

}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightFineDetectorNode)
