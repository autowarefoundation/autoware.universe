/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "traffic_light_classifier/node.hpp"
#include <iostream>

namespace traffic_light
{
TrafficLightClassifierNode::TrafficLightClassifierNode()
: nh_(""),
  pnh_("~"),
  image_transport_(pnh_),
  image_sub_(image_transport_, "input/image", 1),
  roi_sub_(pnh_, "input/rois", 1),
  sync_(SyncPolicy(10), image_sub_, roi_sub_),
  approximate_sync_(ApproximateSyncPolicy(10), image_sub_, roi_sub_)
{
  pnh_.param<bool>("approximate_sync", is_approximate_sync_, false);
  if (is_approximate_sync_) {
    approximate_sync_.registerCallback(
      boost::bind(&TrafficLightClassifierNode::imageRoiCallback, this, _1, _2));
  } else {
    sync_.registerCallback(
      boost::bind(&TrafficLightClassifierNode::imageRoiCallback, this, _1, _2));
  }
  tl_states_pub_ = pnh_.advertise<autoware_perception_msgs::TrafficLightStateArray>(
    "output/traffic_light_states", 1);

  int classifier_type;
  pnh_.param<int>(
    "classifier_type", classifier_type, TrafficLightClassifierNode::ClassifierType::HSVFilter);
  if (classifier_type == TrafficLightClassifierNode::ClassifierType::HSVFilter) {
    classifier_ptr_ = std::make_shared<ColorClassifier>();
  } else if (classifier_type == TrafficLightClassifierNode::ClassifierType::CNN) {
#if ENABLE_GPU
    classifier_ptr_ = std::make_shared<CNNClassifier>();
#else
    ROS_ERROR("please install CUDA, CUDNN and TensorRT to use cnn classifier");
#endif
  }
}

void TrafficLightClassifierNode::imageRoiCallback(
  const sensor_msgs::ImageConstPtr & input_image_msg,
  const autoware_perception_msgs::TrafficLightRoiArrayConstPtr & input_rois_msg)
{
  if (classifier_ptr_.use_count() == 0) {
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", input_image_msg->encoding.c_str());
  }

  autoware_perception_msgs::TrafficLightStateArray output_msg;

  for (size_t i = 0; i < input_rois_msg->rois.size(); ++i) {
    const sensor_msgs::RegionOfInterest & roi = input_rois_msg->rois.at(i).roi;
    cv::Mat cliped_image(
      cv_ptr->image, cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height));

    std::vector<autoware_perception_msgs::LampState> lamp_states;

    if (!classifier_ptr_->getLampState(cliped_image, lamp_states)) {
      ROS_ERROR("failed classify image, abort callback");
      return;
    }
    autoware_perception_msgs::TrafficLightState tl_state;
    tl_state.id = input_rois_msg->rois.at(i).id;
    tl_state.lamp_states = lamp_states;
    output_msg.states.push_back(tl_state);
  }

  output_msg.header = input_image_msg->header;
  tl_states_pub_.publish(output_msg);
}

}  // namespace traffic_light
