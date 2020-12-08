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

#include "traffic_light_roi_visualizer/nodelet.hpp"

namespace traffic_light
{
TrafficLightRoiVisualizerNodelet::TrafficLightRoiVisualizerNodelet(const rclcpp::NodeOptions & options)
: Node("traffic_light_roi_visualizer_node", options)
{  
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  enable_fine_detection_ = this->declare_parameter("enable_fine_detection", false);

  if (enable_fine_detection_) {
    sync_with_rough_roi_.reset(new SyncWithRoughRoi(SyncPolicyWithRoughRoi(10), image_sub_, roi_sub_,  rough_roi_sub_));
    sync_with_rough_roi_->registerCallback(std::bind(&TrafficLightRoiVisualizerNodelet::imageRoughRoiCallback, this, _1, _2, _3));
  } else {
    sync_.reset(new Sync(SyncPolicy(10), image_sub_, roi_sub_));
    sync_->registerCallback(std::bind(&TrafficLightRoiVisualizerNodelet::imageRoiCallback, this, _1, _2));
  }
  
  auto timer_callback = std::bind(&TrafficLightRoiVisualizerNodelet::connectCb, this);
  const auto period_s = 0.1;
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);

  image_pub_ = image_transport::create_publisher(this, "output/image", rclcpp::QoS{1}.get_rmw_qos_profile());
}

void TrafficLightRoiVisualizerNodelet::connectCb()
{
  if (image_pub_.getNumSubscribers() == 0) {
    image_sub_.unsubscribe();
    roi_sub_.unsubscribe();
    if (enable_fine_detection_) rough_roi_sub_.unsubscribe();
  } else if (!image_sub_.getSubscriber()) {
    image_sub_.subscribe(this, "input/image", "raw", rclcpp::QoS{1}.get_rmw_qos_profile());
    roi_sub_.subscribe(this, "input/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
    if (enable_fine_detection_) {
      rough_roi_sub_.subscribe(this, "input/rough/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
    }
  }
}

bool TrafficLightRoiVisualizerNodelet::createRect(
  cv::Mat& image,
  const autoware_perception_msgs::msg::TrafficLightRoi& tl_roi,
  const cv::Scalar& color)
{
  cv::rectangle(
    image,
    cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset),
    cv::Point(tl_roi.roi.x_offset + tl_roi.roi.width, tl_roi.roi.y_offset + tl_roi.roi.height),
    color, 1, CV_AA, 0);
  cv::putText(
    image,
    std::to_string(tl_roi.id),
    cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset),
    cv::FONT_HERSHEY_SIMPLEX, 1.0, color, 1, CV_AA);
  return true;
}

void TrafficLightRoiVisualizerNodelet::imageRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const autoware_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_roi_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, input_image_msg->encoding);
    for (auto tl_roi : input_tl_roi_msg->rois) {
      createRect(cv_ptr->image, tl_roi, cv::Scalar(0,255,0));
    }
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Could not convert from '%s' to 'bgr8'.", input_image_msg->encoding.c_str());
  }
  image_pub_.publish(cv_ptr->toImageMsg());
}

void TrafficLightRoiVisualizerNodelet::imageRoughRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const autoware_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_roi_msg,
  const autoware_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_rough_roi_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, input_image_msg->encoding);
    for (auto tl_roi : input_tl_roi_msg->rois) {
      createRect(cv_ptr->image, tl_roi, cv::Scalar(0,0,255));
    }
    for (auto tl_rough_roi : input_tl_rough_roi_msg->rois) {
      createRect(cv_ptr->image, tl_rough_roi, cv::Scalar(0,255,0));
    }
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "Could not convert from '%s' to 'bgr8'.", input_image_msg->encoding.c_str());
  }
  image_pub_.publish(cv_ptr->toImageMsg());
}

}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::TrafficLightRoiVisualizerNodelet)
