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

#ifndef INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_
#define INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_

#include <interface_utils/rclcpp/service_server.hpp>
#include <rclcpp/rclcpp.hpp>

#include <utility>

namespace interface_utils
{

// Use a node pointer because shared_from_this cannot be used in constructor.
template <class SpecT, class NodeT, class CallbackT>
typename Service<SpecT>::SharedPtr create_service_impl(
  NodeT * node, CallbackT && callback, rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  auto wrapped = Service<SpecT>::wrap(callback, node->get_logger());
  auto service = node->template create_service<typename SpecT::Service>(
    SpecT::name, wrapped, rmw_qos_profile_services_default, group);
  return Service<SpecT>::make_shared(service);
}

template <class SpecT, class NodeT, class CallbackT>
typename Service<SpecT>::SharedPtr create_service(
  NodeT * node, CallbackT && callback, rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return create_service_impl<SpecT>(node, std::forward<CallbackT>(callback), group);
}

template <class SpecT, class NodeT>
typename Service<SpecT>::SharedPtr create_service(
  NodeT * node, typename Service<SpecT>::template CallbackType<NodeT> callback,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  return create_service_impl<SpecT>(node, std::bind(callback, node, _1, _2), group);
}

}  // namespace interface_utils

#endif  // INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_
