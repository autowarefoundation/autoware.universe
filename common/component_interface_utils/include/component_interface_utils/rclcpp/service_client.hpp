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

#ifndef COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_
#define COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_

#include <component_interface_utils/rclcpp/exceptions.hpp>
#include <rclcpp/node.hpp>

#include <tier4_system_msgs/msg/service_log.hpp>

#include <optional>
#include <string>
#include <utility>

namespace component_interface_utils
{

/// The wrapper class of rclcpp::Client for logging.
template <class SpecT>
class Client
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Client)
  using SpecType = SpecT;
  using WrapType = rclcpp::Client<typename SpecT::Service>;
  using ServiceLog = tier4_system_msgs::msg::ServiceLog;

  /// Constructor.
  template <class NodeT>
  Client(typename WrapType::SharedPtr client, NodeT * node)
  {
    client_ = client;  // to keep the reference count
    pub_ = node->template create_publisher<ServiceLog>("/service_log", 10);
    src_ = node->get_namespace() + std::string("/") + node->get_name();
  }

  /// Send request.
  typename WrapType::SharedResponse call(
    const typename WrapType::SharedRequest request, std::optional<double> timeout = std::nullopt)
  {
    if (!client_->service_is_ready()) {
      log(ServiceLog::ERROR_UNREADY);
      throw ServiceUnready(SpecT::name);
    }

    const auto future = this->async_send_request(request);
    if (timeout) {
      const auto duration = std::chrono::duration<double, std::ratio<1>>(timeout.value());
      if (future.wait_for(duration) != std::future_status::ready) {
        log(ServiceLog::ERROR_TIMEOUT);
        throw ServiceTimeout(SpecT::name);
      }
    }
    return future.get();
  }

  /// Send request.
  typename WrapType::SharedFuture async_send_request(typename WrapType::SharedRequest request)
  {
    return this->async_send_request(request, [](typename WrapType::SharedFuture) {});
  }

  /// Send request.
  template <class CallbackT>
  typename WrapType::SharedFuture async_send_request(
    typename WrapType::SharedRequest request, CallbackT && callback)
  {
#ifdef ROS_DISTRO_GALACTIC
    using rosidl_generator_traits::to_yaml;
#endif

    const auto wrapped = [this, callback](typename WrapType::SharedFuture future) {
      log(ServiceLog::CLIENT_RESPONSE, to_yaml(*future.get()));
      callback(future);
    };

    log(ServiceLog::CLIENT_REQUEST, to_yaml(*request));

#ifdef ROS_DISTRO_GALACTIC
    return client_->async_send_request(request, wrapped);
#else
    return client_->async_send_request(request, wrapped).future;
#endif
  }

private:
  RCLCPP_DISABLE_COPY(Client)
  typename WrapType::SharedPtr client_;
  rclcpp::Publisher<ServiceLog>::SharedPtr pub_;
  std::string src_;

  void log(ServiceLog::_type_type type, const std::string & yaml = "")
  {
    ServiceLog msg;
    // msg.stamp =
    msg.type = type;
    msg.name = SpecT::name;
    msg.node = src_;
    // msg.guid =
    msg.yaml = yaml;
    pub_->publish(msg);
  }
};

}  // namespace component_interface_utils

#endif  // COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_
