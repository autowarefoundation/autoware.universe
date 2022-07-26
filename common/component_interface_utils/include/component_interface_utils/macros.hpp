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

#ifndef COMPONENT_INTERFACE_UTILS__MACROS_HPP_
#define COMPONENT_INTERFACE_UTILS__MACROS_HPP_

#define ROS_REQ_TYPE(Type) const Type::Request::SharedPtr
#define ROS_RES_TYPE(Type) const Type::Response::SharedPtr
#define ROS_MSG_TYPE(Type) const Type::ConstSharedPtr

#define ROS_SERVICE_ARG(Type, req, res) ROS_REQ_TYPE(Type) req, ROS_RES_TYPE(Type) res
#define ROS_MESSAGE_ARG(Type, msg) ROS_MSG_TYPE(Type) msg

#define API_SERVICE_ARG(Type, req, res) ROS_SERVICE_ARG(Type::Service, req, res)
#define API_MESSAGE_ARG(Type, msg) ROS_MESSAGE_ARG(Type::Message, msg)

#define BIND_SERVICE(this, func) [this](auto req, auto res) { func(req, res); }
#define BIND_MESSAGE(this, func) [this](auto msg) { func(msg); }

#endif  // COMPONENT_INTERFACE_UTILS__MACROS_HPP_
