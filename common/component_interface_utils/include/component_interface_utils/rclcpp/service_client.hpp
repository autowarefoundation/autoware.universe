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

#include <rclcpp/client.hpp>

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

  /// Constructor.
  explicit Client(typename WrapType::SharedPtr client)
  {
    client_ = client;  // to keep the reference count
  }

  /// Send request.
  typename WrapType::SharedFuture async_send_request(typename WrapType::SharedRequest request)
  {
    return client_->async_send_request(request);
  }

  /// Send request.
  template <class CallbackT>
  typename WrapType::SharedFuture async_send_request(
    typename WrapType::SharedRequest request, CallbackT && callback)
  {
    return client_->async_send_request(request, std::forward<CallbackT>(callback));
  }

private:
  RCLCPP_DISABLE_COPY(Client)
  typename WrapType::SharedPtr client_;
};

}  // namespace component_interface_utils

#endif  // COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_CLIENT_HPP_
