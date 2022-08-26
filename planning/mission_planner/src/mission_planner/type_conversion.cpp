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

#include "type_conversion.hpp"

#include <vector>

namespace
{

using APIPrimitive = autoware_ad_api_msgs::msg::RoutePrimitive;
using HADPrimitive = autoware_auto_mapping_msgs::msg::MapPrimitive;
using APISegment = autoware_ad_api_msgs::msg::RouteSegment;
using HADSegment = autoware_auto_mapping_msgs::msg::HADMapSegment;

template <class RetT, class ArgT>
RetT Convert(const ArgT & arg);

template <class RetT, class ArgT>
std::vector<RetT> ConvertVector(const std::vector<ArgT> & args)
{
  std::vector<RetT> result;
  result.reserve(args.size());
  for (const auto & arg : args) {
    result.push_back(Convert<RetT, ArgT>(arg));
  }
  return result;
}

template <>
APIPrimitive Convert(const HADPrimitive & had)
{
  APIPrimitive api;
  api.id = had.id;
  api.type = had.primitive_type;
  return api;
}

template <>
HADPrimitive Convert(const APIPrimitive & api)
{
  HADPrimitive had;
  had.id = api.id;
  had.primitive_type = api.type;
  return had;
}

template <>
APISegment Convert(const HADSegment & had)
{
  APISegment api;
  api.alternatives = ConvertVector<APIPrimitive>(had.primitives);
  for (auto iter = api.alternatives.begin(); iter != api.alternatives.end(); ++iter) {
    if (iter->id == had.preferred_primitive_id) {
      api.preferred = *iter;
      api.alternatives.erase(iter);
      break;
    }
  }
  return api;
}

template <>
HADSegment Convert(const APISegment & api)
{
  HADSegment had;
  had.primitives = ConvertVector<HADPrimitive>(api.alternatives);
  had.primitives.push_back(Convert<HADPrimitive>(api.preferred));
  had.preferred_primitive_id = had.primitives.back().id;
  return had;
}

}  // namespace

namespace mission_planner::conversion
{

APIRoute CreateEmptyRoute(const rclcpp::Time & stamp)
{
  APIRoute api_route;
  api_route.header.stamp = stamp;
  return api_route;
}

APIRoute ConvertRoute(const HADRoute & had)
{
  APIRouteData data;
  data.start = had.start_pose;
  data.goal = had.goal_pose;
  data.segments = ConvertVector<APISegment>(had.segments);

  APIRoute api;
  api.header = had.header;
  api.data.push_back(data);
  return api;
}

HADRoute ConvertRoute(const APIRouteData & api)
{
  // The header is assigned by mission planner.
  HADRoute had;
  had.start_pose = api.start;
  had.goal_pose = api.goal;
  had.segments = ConvertVector<HADSegment>(api.segments);
  return had;
}

}  // namespace mission_planner::conversion
