// Copyright 2023 The Autoware Foundation
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
#ifndef AUTOWARE_AUTO_MSGS_ADAPTER__ADAPTER_BASE_INTERFACE_HPP_
#define AUTOWARE_AUTO_MSGS_ADAPTER__ADAPTER_BASE_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware_auto_msgs_adapter
{

class AdapterBaseInterface
{
public:
  using SharedPtrInterface = std::shared_ptr<AdapterBaseInterface>;
  using ConstSharedPtrInterface = const SharedPtrInterface;

  virtual ~AdapterBaseInterface() = default;
};

}  // namespace autoware_auto_msgs_adapter

#endif  // AUTOWARE_AUTO_MSGS_ADAPTER__ADAPTER_BASE_INTERFACE_HPP_
