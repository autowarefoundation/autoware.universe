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

LocalizationScoreNode::LocalizationScoreNode(const rclcpp::NodeOptions & options) : Node("localization_score", options)
{
  using std::placeholders::_1;

   // Publisher
  pub_localization_scores_ = this->create_publisher<LocalizationScores>(
    "api/get/localization_scores", 1);

  // Subscriber
  sub_localization_scores_ =
    this->create_subscription<LocalizationScores>("/localization_scores", 1,
      std::bind(&LocalizationScoreNode::callbackLocalizationScores, this, _1));
}

void LocalizationScoreNode::callbackLocalizationScores(const LocalizationScores::ConstSharedPtr msg_ptr){
  pub_localization_scores_->publish(*msg_ptr);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::LocalizationScoreNode)
