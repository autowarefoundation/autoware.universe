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

#include <memory>
#include <vector>

namespace
{

using autoware_adapi_v1_msgs::msg::RoutePrimitive;
using autoware_adapi_v1_msgs::msg::RouteSegment;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletSegment;

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
RoutePrimitive convert(const LaneletPrimitive & in)
{
  RoutePrimitive api;
  api.id = in.id;
  api.type = in.primitive_type;
  return api;
}

template <>
LaneletPrimitive convert(const RoutePrimitive & in)
{
  LaneletPrimitive out;
  out.id = in.id;
  out.primitive_type = in.type;
  return out;
}

template <>
RouteSegment convert(const LaneletSegment & in)
{
  RouteSegment api;
  api.alternatives = convert_vector<RoutePrimitive>(in.primitives);
  for (auto iter = api.alternatives.begin(); iter != api.alternatives.end(); ++iter) {
    if (iter->id == in.preferred_primitive.id) {
      api.preferred = *iter;
      api.alternatives.erase(iter);
      break;
    }
  }
  return api;
}

template <>
LaneletSegment convert(const RouteSegment & in)
{
  LaneletSegment out;
  out.preferred_primitive = convert<LaneletPrimitive>(in.preferred);
  out.primitives.push_back(out.preferred_primitive);
  for (const auto & primitive : in.alternatives) {
    out.primitives.push_back(convert<LaneletPrimitive>(primitive));
  }
  return out;
}

}  // namespace

namespace autoware::default_adapi::conversion
{

ExternalRoute create_empty_route(const rclcpp::Time & stamp)
{
  ExternalRoute external;
  external.header.stamp = stamp;
  return external;
}

ExternalRoute convert_route(const InternalRoute & internal)
{
  autoware_adapi_v1_msgs::msg::RouteData data;
  data.start = internal.start_pose;
  data.goal = internal.goal_pose;
  data.segments = convert_vector<RouteSegment>(internal.segments);

  ExternalRoute external;
  external.header = internal.header;
  external.data.push_back(data);
  return external;
}

ExternalState convert_state(const InternalState & internal)
{
  // clang-format off
  const auto convert = [](InternalState::_state_type state) {
    switch(state) {
      // TODO(Takagi, Isamu): Add adapi state.
      case InternalState::INITIALIZING: return ExternalState::UNSET; // NOLINT
      case InternalState::UNSET:        return ExternalState::UNSET;
      case InternalState::ROUTING:      return ExternalState::UNSET;
      case InternalState::SET:          return ExternalState::SET;
      case InternalState::REROUTING:    return ExternalState::CHANGING;
      case InternalState::ARRIVED:      return ExternalState::ARRIVED;
      case InternalState::ABORTED:      return ExternalState::SET;  // NOLINT
      case InternalState::INTERRUPTED:  return ExternalState::SET;
      default:                          return ExternalState::UNKNOWN;
    }
  };
  // clang-format on

  ExternalState external;
  external.stamp = internal.stamp;
  external.state = convert(internal.state);
  return external;
}

InternalClearRequest convert_request(const ExternalClearRequest &)
{
  auto internal = std::make_shared<InternalClearRequest::element_type>();
  return internal;
}

InternalLaneletRequest convert_request(const ExternalLaneletRequest & external)
{
  auto internal = std::make_shared<InternalLaneletRequest::element_type>();
  internal->header = external->header;
  internal->goal_pose = external->goal;
  internal->segments = convert_vector<LaneletSegment>(external->segments);
  internal->allow_modification = external->option.allow_goal_modification;
  return internal;
}

InternalWaypointRequest convert_request(const ExternalWaypointRequest & external)
{
  auto internal = std::make_shared<InternalWaypointRequest::element_type>();
  internal->header = external->header;
  internal->goal_pose = external->goal;
  internal->waypoints = external->waypoints;
  internal->allow_modification = external->option.allow_goal_modification;
  return internal;
}

ExternalResponse convert_response(const InternalResponse & internal)
{
  // TODO(Takagi, Isamu): check error code correspondence
  ExternalResponse external;
  external.success = internal.success;
  external.code = internal.code;
  external.message = internal.message;
  return external;
}

}  // namespace autoware::default_adapi::conversion
