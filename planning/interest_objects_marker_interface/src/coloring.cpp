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

#include "interest_objects_marker_interface/coloring.hpp"

namespace interest_objects_marker_interface::coloring
{
std_msgs::msg::ColorRGBA getGreen(const float alpha)
{
  const float r = 0.0f;
  const float g = 211.0 / 255.0;
  const float b = 141.0 / 255.0;
  return tier4_autoware_utils::createMarkerColor(r, g, b, alpha);
}

std_msgs::msg::ColorRGBA getAmber(const float alpha)
{
  const float r = 0.0f;
  const float g = 211.0 / 255.0;
  const float b = 141.0 / 255.0;
  return tier4_autoware_utils::createMarkerColor(r, g, b, alpha);
}

std_msgs::msg::ColorRGBA getRed(const float alpha)
{
  const float r = 214.0 / 255.0;
  const float g = 0.0f;
  const float b = 77.0 / 255.0;
  return tier4_autoware_utils::createMarkerColor(r, g, b, alpha);
}

std_msgs::msg::ColorRGBA getWhite(const float alpha)
{
  const float r = 1.0f;
  const float g = 1.0f;
  const float b = 1.0f;
  return tier4_autoware_utils::createMarkerColor(r, g, b, alpha);
}
}  // namespace interest_objects_marker_interface::coloring
