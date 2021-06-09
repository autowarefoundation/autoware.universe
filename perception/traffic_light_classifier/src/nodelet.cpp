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
#include "traffic_light_classifier/nodelet.hpp"
#include <iostream>
#include <memory>
#include <vector>
#include <utility>

namespace traffic_light
{
TrafficLightClassifierNodelet::TrafficLightClassifierNodelet(const rclcpp::NodeOptions & options)
: Node("traffic_light_classifier_node", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  is_approximate_sync_ = this->declare_parameter("approximate_sync", false);
  if (is_approximate_sync_) {
    approximate_sync_.reset(new ApproximateSync(ApproximateSyncPolicy(10), image_sub_, roi_sub_));
    approximate_sync_->registerCallback(
      std::bind(&TrafficLightClassifierNodelet::imageRoiCallback, this, _1, _2));
  } else {
    sync_.reset(new Sync(SyncPolicy(10), image_sub_, roi_sub_));
    sync_->registerCallback(
      std::bind(&TrafficLightClassifierNodelet::imageRoiCallback, this, _1, _2));
  }


  tl_states_pub_ = this->create_publisher<autoware_perception_msgs::msg::TrafficLightStateArray>(
    "~/output/traffic_light_states", rclcpp::QoS{1});

  //
  auto timer_callback = std::bind(&TrafficLightClassifierNodelet::connectCb, this);
  const auto period_s = 0.1;
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);

  int classifier_type = this->declare_parameter(
    "classifier_type", static_cast<int>(TrafficLightClassifierNodelet::ClassifierType::HSVFilter));
  if (classifier_type == TrafficLightClassifierNodelet::ClassifierType::HSVFilter) {
    classifier_ptr_ = std::make_shared<ColorClassifier>(this);
  } else if (classifier_type == TrafficLightClassifierNodelet::ClassifierType::CNN) {
#if ENABLE_GPU
    classifier_ptr_ = std::make_shared<CNNClassifier>(this);
#else
    RCLCPP_ERROR(
      this->get_logger(), "please install CUDA, CUDNN and TensorRT to use cnn classifier");
#endif
  }
}

void TrafficLightClassifierNodelet::connectCb()
{
  // set callbacks only when there are subscribers to this node
  if (tl_states_pub_->get_subscription_count() == 0 &&
    tl_states_pub_->get_intra_process_subscription_count() == 0)
  {
    image_sub_.unsubscribe();
    roi_sub_.unsubscribe();
  } else if (!image_sub_.getSubscriber()) {
    image_sub_.subscribe(this, "~/input/image", "raw", rmw_qos_profile_sensor_data);
    roi_sub_.subscribe(this, "~/input/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
  }
}

void TrafficLightClassifierNodelet::imageRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const autoware_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_rois_msg)
{
  if (classifier_ptr_.use_count() == 0) {
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not convert from '%s' to 'rgb8'.",
      input_image_msg->encoding.c_str());
  }

  autoware_perception_msgs::msg::TrafficLightStateArray output_msg;

  for (size_t i = 0; i < input_rois_msg->rois.size(); ++i) {
    const sensor_msgs::msg::RegionOfInterest & roi = input_rois_msg->rois.at(i).roi;
    cv::Mat clipped_image(
      cv_ptr->image, cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height));

    std::vector<autoware_perception_msgs::msg::LampState> lamp_states;

    if (!classifier_ptr_->getLampState(clipped_image, lamp_states)) {
      RCLCPP_ERROR(this->get_logger(), "failed classify image, abort callback");
      return;
    }
    autoware_perception_msgs::msg::TrafficLightState tl_state;
    tl_state.id = input_rois_msg->rois.at(i).id;
    tl_state.lamp_states = lamp_states;
    output_msg.states.push_back(tl_state);
  }

  output_msg.header = input_image_msg->header;
  tl_states_pub_->publish(output_msg);
}

}  // namespace traffic_light

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::TrafficLightClassifierNodelet)
