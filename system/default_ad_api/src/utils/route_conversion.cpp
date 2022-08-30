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

#include "route_conversion.hpp"

#include <vector>

namespace
{

using APIPrimitive = autoware_ad_api_msgs::msg::RoutePrimitive;
using HADPrimitive = autoware_auto_mapping_msgs::msg::MapPrimitive;
using APISegment = autoware_ad_api_msgs::msg::RouteSegment;
using HADSegment = autoware_auto_mapping_msgs::msg::HADMapSegment;

template <class RetT, class ArgT>
RetT convert(const ArgT & arg);

template <class RetT, class ArgT>
std::vector<RetT> convert_vector(const std::vector<ArgT> & args)
{
  std::vector<RetT> result;
  result.reserve(args.size());
  for (const auto & arg : args) {
    result.push_back(convert<RetT, ArgT>(arg));
  }
  return result;
}

template <>
APIPrimitive convert(const HADPrimitive & had)
{
  APIPrimitive api;
  api.id = had.id;
  api.type = had.primitive_type;
  return api;
}

template <>
HADPrimitive convert(const APIPrimitive & api)
{
  HADPrimitive had;
  had.id = api.id;
  had.primitive_type = api.type;
  return had;
}

template <>
APISegment convert(const HADSegment & had)
{
  APISegment api;
  api.alternatives = convert_vector<APIPrimitive>(had.primitives);
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
HADSegment convert(const APISegment & api)
{
  HADSegment had;
  had.primitives = convert_vector<HADPrimitive>(api.alternatives);
  had.primitives.push_back(convert<HADPrimitive>(api.preferred));
  had.preferred_primitive_id = had.primitives.back().id;
  return had;
}

}  // namespace

namespace default_ad_api::conversion
{

ApiRoute create_empty_route(const rclcpp::Time & stamp)
{
  ApiRoute api_route;
  api_route.header.stamp = stamp;
  return api_route;
}

ApiRoute convert_route(const HadRoute & had)
{
  autoware_ad_api_msgs::msg::RouteData data;
  data.start = had.start_pose;
  data.goal = had.goal_pose;
  data.segments = convert_vector<APISegment>(had.segments);

  ApiRoute api;
  api.header = had.header;
  api.data.push_back(data);
  return api;
}

HadSetRoute convert_set_route(const ApiSetRoute & api)
{
  HadSetRoute had;
  had.header = api.header;
  had.goal = api.goal;
  had.segments = convert_vector<HADSegment>(api.segments);
  return had;
}

}  // namespace default_ad_api::conversion
