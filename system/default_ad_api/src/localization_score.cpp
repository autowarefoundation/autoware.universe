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

#include "localization_score.hpp"

namespace default_ad_api
{

LocalizationScoreNode::LocalizationScoreNode(const rclcpp::NodeOptions & options)
: Node("localization_score", options),
  clock_(this->get_clock())
{
  using std::placeholders::_1;

  status_pub_hz_ = this->declare_parameter("status_pub_hz", 10.0);

  score_tp_.type = "transform_probability";
  score_nvtl_.type = "nearest_voxel_transformation_likelihood";

  // Publisher
  pub_localization_scores_ =
    this->create_publisher<LocalizationScores>("api/get/localization_scores", 1);

  // Subscriber
  sub_pose_with_covariance_ = this->create_subscription<PoseWithCovarianceStamped>(
    "/localization/pose_with_covariance", 1,
    std::bind(&LocalizationScoreNode::callbackPoseWithCovariance, this, _1));
  sub_transform_probability_ = this->create_subscription<Float32Stamped>(
    "/localization/pose_estimator/transform_probability", 1,
    std::bind(&LocalizationScoreNode::callbackTpScore, this, _1));
  sub_nearest_voxel_transformation_likelihood_ = this->create_subscription<Float32Stamped>(
    "/localization/pose_estimator/nearest_voxel_transformation_likelihood", 1,
    std::bind(&LocalizationScoreNode::callbackNvtlScore, this, _1));
  
  // Timer callback
  auto timer_callback = std::bind(&LocalizationScoreNode::callbackTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / status_pub_hz_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void LocalizationScoreNode::callbackPoseWithCovariance(const PoseWithCovarianceStamped::ConstSharedPtr msg_ptr){
  pose_covariance_ = msg_ptr->pose;
}
void LocalizationScoreNode::callbackTpScore(const Float32Stamped::ConstSharedPtr msg_ptr){
  score_tp_.value = msg_ptr->data; 
}
void LocalizationScoreNode::callbackNvtlScore(const Float32Stamped::ConstSharedPtr msg_ptr){
  score_nvtl_.value = msg_ptr->data; 
}

void LocalizationScoreNode::callbackTimer(){
  LocalizationScores localizatoin_scores_msg;

  localizatoin_scores_msg.header.frame_id = "map";
  localizatoin_scores_msg.header.stamp = clock_->now();
  localizatoin_scores_msg.pose_covariance = pose_covariance_;
  localizatoin_scores_msg.values.emplace_back(score_tp_);
  localizatoin_scores_msg.values.emplace_back(score_nvtl_);

  pub_localization_scores_->publish(localizatoin_scores_msg);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::LocalizationScoreNode)
