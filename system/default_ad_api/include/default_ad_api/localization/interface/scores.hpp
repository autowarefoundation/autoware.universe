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

#ifndef DEFAULT_AD_API__LOCALIZATION__INTERFACE__VERSION_HPP_
#define DEFAULT_AD_API__LOCALIZATION__INTERFACE__VERSION_HPP_

#include "autoware_ad_api_msgs/msg/localization_scores.hpp"

namespace ad_api::interface::scores
{

struct T_sub
{
  using Message = autoware_ad_api_msgs::msg::LocalizationScores;
  static constexpr char name[] = "/localization_scores";
};

struct T_pub
{
  using Message = autoware_ad_api_msgs::msg::LocalizationScores;
  static constexpr char name[] = "api/interface/localization_scores";
};

}  // namespace ad_api::interface::scores

#endif  // DEFAULT_AD_API__LOCALIZATION__INTERFACE__VERSION_HPP_
