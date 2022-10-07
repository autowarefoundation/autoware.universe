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

#include "pointcloud_preprocessor/blockage_diag/blockage_diag_nodelet.hpp"

#include "autoware_point_types/types.hpp"

#include <boost/circular_buffer.hpp>
#include <boost/thread/detail/platform_time.hpp>

#include <algorithm>
#include <numeric>

namespace pointcloud_preprocessor
{
using autoware_point_types::PointXYZIRADRT;
using diagnostic_msgs::msg::DiagnosticStatus;

BlockageDiagComponent::BlockageDiagComponent(const rclcpp::NodeOptions & options)
: Filter("BlockageDiag", options)
{
  {
    // initialize params:
    horizontal_ring_id_ = static_cast<uint>(declare_parameter("horizontal_ring_id", 12));
    blockage_ratio_threshold_ =
      static_cast<float>(declare_parameter("blockage_ratio_threshold", 0.1));
    vertical_bins_ = static_cast<uint>(declare_parameter("vertical_bins", 40));
    angle_range_deg_ = declare_parameter("angle_range", std::vector<double>{0.0, 360.0});
    lidar_model_ = static_cast<std::string>(declare_parameter("model", "Pandar40P"));
    blockage_count_threshold_ =
      static_cast<uint>(declare_parameter("blockage_count_threshold", 50));
  }

  updater_.setHardwareID("blockage_diag");
  updater_.add(
    std::string(this->get_namespace()) + ": blockage_validation", this,
    &BlockageDiagComponent::onBlockageChecker);
  updater_.setPeriod(0.1);
  one_shot_sobel_pub =
    image_transport::create_publisher(this, "blockage_diag/debug/one_shot_sobel");
  lidar_colorized_depth_map_pub_ =
    image_transport::create_publisher(this, "blockage_diag/debug/lidar_depth_map");
  blockage_mask_pub_ =
    image_transport::create_publisher(this, "blockage_diag/debug/blockage_mask_image");
  time_series_sobel_pub =
    image_transport::create_publisher(this, "blockage_diag/debug/time_series_sobel_image");

  ground_blockage_ratio_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "blockage_diag/debug/ground_blockage_ratio", rclcpp::SensorDataQoS());
  sky_blockage_ratio_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "blockage_diag/debug/sky_blockage_ratio", rclcpp::SensorDataQoS());
  processing_time_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "blockage_diag/debug/processing_time", rclcpp::SensorDataQoS());

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&BlockageDiagComponent::paramCallback, this, _1));
}

void BlockageDiagComponent::onBlockageChecker(DiagnosticStatusWrapper & stat)
{
  stat.add("ground_blockage_ratio", std::to_string(ground_blockage_ratio_));
  stat.add("ground_blockage_count", std::to_string(ground_blockage_count_));
  stat.add(
    "ground_blockage_range_deg", "[" + std::to_string(ground_blockage_range_deg_[0]) + "," +
                                   std::to_string(ground_blockage_range_deg_[1]) + "]");
  stat.add("sky_blockage_ratio", std::to_string(sky_blockage_ratio_));
  stat.add("sky_blockage_count", std::to_string(sky_blockage_count_));
  stat.add(
    "sky_blockage_range_deg", "[" + std::to_string(sky_blockage_range_deg_[0]) + "," +
                                std::to_string(sky_blockage_range_deg_[1]) + "]");

  // TODO(badai-nguyen): consider sky_blockage_ratio_ for DiagnosticsStatus." [todo]

  auto level = DiagnosticStatus::OK;
  std::string msg;
  if (ground_blockage_ratio_ < 0) {
    level = DiagnosticStatus::STALE;
    msg = "STALE";
  } else if (
    (ground_blockage_ratio_ > blockage_ratio_threshold_) &&
    (ground_blockage_count_ > blockage_count_threshold_)) {
    level = DiagnosticStatus::ERROR;
    msg = "ERROR";
  } else if (ground_blockage_ratio_ > 0.0f) {
    level = DiagnosticStatus::WARN;
    msg = "WARN";
  } else {
    level = DiagnosticStatus::OK;
    msg = "OK";
  }

  if ((ground_blockage_ratio_ > 0.0f) && (sky_blockage_ratio_ > 0.0f)) {
    msg = msg + ": LIDAR both blockage";
  } else if (ground_blockage_ratio_ > 0.0f) {
    msg = msg + ": LIDAR ground blockage";
  } else if (sky_blockage_ratio_ > 0.0f) {
    msg = msg + ": LIDAR sky blockage";
  }
  stat.summary(level, msg);
}

void BlockageDiagComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  auto start = std::chrono::system_clock::now();
  std::scoped_lock lock(mutex_);
  uint horizontal_bins = static_cast<uint>((angle_range_deg_[1] - angle_range_deg_[0]));
  uint vertical_bins = vertical_bins_;
  int ideal_horizontal_bins;
  float distance_coeffients;
  float horizontal_resolution_;
  if (lidar_model_ == "Pandar40P") {
    distance_coeffients = 327.67f;
    horizontal_resolution_ = 0.4f;
  } else if (lidar_model_ == "PandarQT") {
    distance_coeffients = 3276.75f;
    horizontal_resolution_ = 0.6f;
  }
  ideal_horizontal_bins =
    static_cast<uint>((angle_range_deg_[1] - angle_range_deg_[0]) / horizontal_resolution_);
  pcl::PointCloud<PointXYZIRADRT>::Ptr pcl_input(new pcl::PointCloud<PointXYZIRADRT>);
  pcl::fromROSMsg(*input, *pcl_input);
  std::vector<float> horizontal_bin_reference(ideal_horizontal_bins);
  std::vector<pcl::PointCloud<PointXYZIRADRT>> each_ring_pointcloud(vertical_bins);
  cv::Mat full_size_depth_map(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_16UC1, cv::Scalar(0));
  cv::Mat lidar_depth_map(cv::Size(ideal_horizontal_bins, vertical_bins), CV_16UC1, cv::Scalar(0));
  cv::Mat lidar_depth_map_8u(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));
  if (pcl_input->points.empty()) {
    ground_blockage_ratio_ = 1.0f;
    sky_blockage_ratio_ = 1.0f;
    if (ground_blockage_count_ <= 2 * blockage_count_threshold_) {
      ground_blockage_count_ += 1;
    }
    if (sky_blockage_count_ <= 2 * blockage_count_threshold_) {
      sky_blockage_count_ += 1;
    }
    ground_blockage_range_deg_[0] = angle_range_deg_[0];
    ground_blockage_range_deg_[1] = angle_range_deg_[1];
    sky_blockage_range_deg_[0] = angle_range_deg_[0];
    sky_blockage_range_deg_[1] = angle_range_deg_[1];
  } else {
    for (int i = 0; i < ideal_horizontal_bins; ++i) {
      horizontal_bin_reference.at(i) = angle_range_deg_[0] + i * horizontal_resolution_;
    }
    for (const auto p : pcl_input->points) {
      for (int horizontal_bin = 0;
           horizontal_bin < static_cast<int>(horizontal_bin_reference.size()); horizontal_bin++) {
        if (
          (p.azimuth / 100 >
           (horizontal_bin_reference.at(horizontal_bin) - horizontal_resolution_ / 2)) &&
          (p.azimuth / 100 <=
           (horizontal_bin_reference.at(horizontal_bin) + horizontal_resolution_ / 2))) {
          if (lidar_model_ == "Pandar40P") {
            full_size_depth_map.at<uint16_t>(p.ring, horizontal_bin) =
              UINT16_MAX - distance_coeffients * p.distance;
          } else if (lidar_model_ == "PandarQT") {
            full_size_depth_map.at<uint16_t>(vertical_bins - p.ring - 1, horizontal_bin) =
              UINT16_MAX - distance_coeffients * p.distance;
          }
        }
      }
    }
  }
  full_size_depth_map.convertTo(lidar_depth_map_8u, CV_8UC1, 1.0 / 300);
  cv::Mat no_return_mask(cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));
  cv::inRange(lidar_depth_map_8u, 0, 1, no_return_mask);
  cv::Mat erosion_dst;
  cv::Mat element = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(2 * erode_kernel_ + 1, 2 * erode_kernel_ + 1),
    cv::Point(erode_kernel_, erode_kernel_));
  cv::erode(no_return_mask, erosion_dst, element);
  cv::dilate(erosion_dst, no_return_mask, element);
  static boost::circular_buffer<cv::Mat> no_return_mask_buffer(blockage_buffer_frames_);
  cv::Mat no_return_mask_result(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));
  cv::Mat time_series_blockage_mask(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));
  cv::Mat no_return_mask_binarized(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));

  static uint blockage_frame_count = 0;
  blockage_frame_count++;
  if (blockage_buffering_interval_ != 0) {
    no_return_mask_binarized = no_return_mask / 255;
    if (blockage_frame_count == blockage_buffering_interval_) {
      no_return_mask_buffer.push_back(no_return_mask_binarized);
      blockage_frame_count = 0;
    }
    for (const auto & binary_mask : no_return_mask_buffer) {
      time_series_blockage_mask += binary_mask;
    }
    cv::inRange(
      time_series_blockage_mask, no_return_mask_buffer.size() - 1, no_return_mask_buffer.size(),
      no_return_mask_result);
  } else {
    no_return_mask.copyTo(no_return_mask_result);
  }
  cv::Mat ground_no_return_mask;
  cv::Mat sky_no_return_mask;
  no_return_mask(cv::Rect(0, 0, ideal_horizontal_bins, horizontal_ring_id_))
    .copyTo(sky_no_return_mask);
  no_return_mask(
    cv::Rect(0, horizontal_ring_id_, ideal_horizontal_bins, vertical_bins - horizontal_ring_id_))
    .copyTo(ground_no_return_mask);
  ground_blockage_ratio_ =
    static_cast<float>(cv::countNonZero(ground_no_return_mask)) /
    static_cast<float>(ideal_horizontal_bins * (vertical_bins - horizontal_ring_id_));
  sky_blockage_ratio_ = static_cast<float>(cv::countNonZero(sky_no_return_mask)) /
                        static_cast<float>(ideal_horizontal_bins * horizontal_ring_id_);

  if (ground_blockage_ratio_ > blockage_ratio_threshold_) {
    cv::Rect ground_blockage_bb = cv::boundingRect(ground_no_return_mask);
    ground_blockage_range_deg_[0] = static_cast<float>(ground_blockage_bb.x) + angle_range_deg_[0];
    ground_blockage_range_deg_[1] =
      static_cast<float>(ground_blockage_bb.x + ground_blockage_bb.width) + angle_range_deg_[0];

    if (ground_blockage_count_ <= 2 * blockage_count_threshold_) {
      ground_blockage_count_ += 1;
    }
  } else {
    ground_blockage_count_ = 0;
  }

  if (sky_blockage_ratio_ > blockage_ratio_threshold_) {
    cv::Rect sky_blockage_bx = cv::boundingRect(sky_no_return_mask);
    sky_blockage_range_deg_[0] = static_cast<float>(sky_blockage_bx.x) + angle_range_deg_[0];
    sky_blockage_range_deg_[1] =
      static_cast<float>(sky_blockage_bx.x + sky_blockage_bx.width) + angle_range_deg_[0];
    if (sky_blockage_count_ <= 2 * blockage_count_threshold_) {
      sky_blockage_count_ += 1;
    }
  } else {
    sky_blockage_count_ = 0;
  }

  /////////sobel
  cv::Mat sobeled_img(
    cv::Size(lidar_depth_map_8u.rows, lidar_depth_map_8u.cols), CV_8UC1, cv::Scalar(0));
  sobeled_img = lidar_depth_map_8u.clone();
  cv::inRange(lidar_depth_map_8u, 0, 1, sobeled_img);
  Sobel(sobeled_img, sobeled_img, CV_8UC1, 1, 0, 3);
  Sobel(sobeled_img, sobeled_img, CV_8UC1, 0, 1, 3);
  cv::Mat morpho_img = sobeled_img.clone();
  uint erode_kernel = 3;
  cv::Mat sobel_element = getStructuringElement(
    cv::MORPH_RECT, cv::Size(erode_kernel + 1, 2 * erode_kernel + 1),
    cv::Point(erode_kernel, erode_kernel));
  cv::dilate(sobeled_img, morpho_img, sobel_element);
  cv::erode(morpho_img, morpho_img, sobel_element);

  static boost::circular_buffer<cv::Mat> binarized_sobel_mask_buffer(sobel_frames_);

  sobeled_img = morpho_img.clone();
  cv::Mat time_series_sobel_mask(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));
  cv::Mat time_series_sobel_result(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));
  cv::Mat sobel_mask_binarized(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC1, cv::Scalar(0));

  static uint sobel_frame_count;
  sobel_frame_count++;
  if (sobel_buffering_interval_ != 0) {
    sobel_mask_binarized = sobeled_img / 255;
    if (sobel_frame_count == sobel_buffering_interval_) {
      binarized_sobel_mask_buffer.push_back(sobel_mask_binarized);  // ここコメントアウトで動く
      sobel_frame_count = 0;
    }
    for (const auto & binary_mask : binarized_sobel_mask_buffer) {
      time_series_sobel_mask += binary_mask;
    }
    //    cv::inRange(
    //      time_series_sobel_mask, binarized_sobel_mask_buffer.size() - 1,
    //      binarized_sobel_mask_buffer.size(), time_series_sobel_result);
    cv::inRange(
      time_series_sobel_mask, binarized_sobel_mask_buffer.size() - 1,
      binarized_sobel_mask_buffer.size(), time_series_sobel_result);
  } else {
    no_return_mask.copyTo(time_series_sobel_result);
  }

  cv::Mat one_shot_sobel_result_img(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC3, cv::Scalar(0));
  cv::applyColorMap(sobeled_img, one_shot_sobel_result_img, cv::COLORMAP_JET);
  //  one_shot_sobel_result_img =sobel_mask.clone();
  sensor_msgs::msg::Image::SharedPtr one_shot_sobel_result_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", one_shot_sobel_result_img).toImageMsg();
  RCLCPP_WARN_STREAM(
    get_logger(), "sobeled_img.type() is " << sobeled_img.type() << "sobeled_img.row is "
                                           << sobeled_img.rows << "sobeled_img.cols is "
                                           << sobeled_img.cols);
  RCLCPP_WARN_STREAM(
    get_logger(), "one_shot_sobeled_img.type() is "
                    << one_shot_sobel_result_img.type() << "sobeled_img.row is "
                    << one_shot_sobel_result_img.rows << "sobeled_img.cols is "
                    << one_shot_sobel_result_img.cols);
  one_shot_sobel_pub.publish(one_shot_sobel_result_msg);
  cv::Mat time_series_sobel_result_color_img(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_8UC3, cv::Scalar(0));
  time_series_sobel_result.convertTo(time_series_sobel_result_color_img, CV_8UC1, 255, 0);
  cv::applyColorMap(
    time_series_sobel_result_color_img, time_series_sobel_result_color_img, cv::COLORMAP_JET);
  sensor_msgs::msg::Image::SharedPtr time_series_sobel_result_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", time_series_sobel_result_color_img)
      .toImageMsg();
  time_series_sobel_result_msg->header = input->header;
  time_series_sobel_pub.publish(time_series_sobel_result_msg);
  /////////sobel
  cv::Mat colorized_full_size_depth_map(
    cv::Size(ideal_horizontal_bins, vertical_bins), CV_16UC1, cv::Scalar(0));
  colorized_full_size_depth_map = full_size_depth_map.clone();
  sensor_msgs::msg::Image::SharedPtr lidar_colorized_depth_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", colorized_full_size_depth_map)
      .toImageMsg();
  lidar_colorized_depth_msg->header = input->header;
  lidar_colorized_depth_map_pub_.publish(lidar_colorized_depth_msg);  // fullsizeをpubしているところ
  cv::Mat blockage_mask_colorized;
  cv::applyColorMap(no_return_mask_result, blockage_mask_colorized, cv::COLORMAP_JET);
  sensor_msgs::msg::Image::SharedPtr blockage_mask_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", blockage_mask_colorized).toImageMsg();
  blockage_mask_msg->header = input->header;
  blockage_mask_pub_.publish(blockage_mask_msg);
  tier4_debug_msgs::msg::Float32Stamped ground_blockage_ratio_msg;
  ground_blockage_ratio_msg.data = ground_blockage_ratio_;
  ground_blockage_ratio_msg.stamp = now();
  ground_blockage_ratio_pub_->publish(ground_blockage_ratio_msg);

  tier4_debug_msgs::msg::Float32Stamped sky_blockage_ratio_msg;
  sky_blockage_ratio_msg.data = sky_blockage_ratio_;
  sky_blockage_ratio_msg.stamp = now();
  sky_blockage_ratio_pub_->publish(sky_blockage_ratio_msg);

  pcl::toROSMsg(*pcl_input, output);
  output.header = input->header;
  auto end = std::chrono::system_clock::now();
  tier4_debug_msgs::msg::Float32Stamped processing_time_msg;
  processing_time_msg.data =
    std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
  processing_time_msg.stamp = now();
  processing_time_pub_->publish(processing_time_msg);
}

rcl_interfaces::msg::SetParametersResult BlockageDiagComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);
  if (get_param(p, "blockage_ratio_threshold", blockage_ratio_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new blockage_ratio_threshold to: %f.", blockage_ratio_threshold_);
  }
  if (get_param(p, "horizontal_ring_id", horizontal_ring_id_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new horizontal_ring_id to: %d.", horizontal_ring_id_);
  }
  if (get_param(p, "vertical_bins", vertical_bins_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new vertical_bins to: %d.", vertical_bins_);
  }
  if (get_param(p, "blockage_count_threshold", blockage_count_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new blockage_count_threshold to: %d.", blockage_count_threshold_);
  }
  if (get_param(p, "model", lidar_model_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new lidar model to: %s. ", lidar_model_.c_str());
  }
  if (get_param(p, "angle_range", angle_range_deg_)) {
    RCLCPP_DEBUG(
      get_logger(), " Setting new angle_range to: [%f , %f].", angle_range_deg_[0],
      angle_range_deg_[1]);
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}
}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::BlockageDiagComponent)
