// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the point_type_adapter_node class.

#ifndef POINT_TYPE_ADAPTER__POINT_TYPE_ADAPTER_NODE_HPP_
#define POINT_TYPE_ADAPTER__POINT_TYPE_ADAPTER_NODE_HPP_

#include "point_cloud2_intensity_wrapper.hpp"
#include "point_type_adapter/visibility_control.hpp"

#include <common/types.hpp>
#include <helper_functions/float_comparisons.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <limits>

namespace autoware
{
namespace tools
{
namespace point_type_adapter
{
/// \class PointTypeAdapterNode
/// \brief ROS 2 Node for converting PointCloud2 clouds with
/// different fields to Autoware.Auto default format
class POINT_TYPE_ADAPTER_PUBLIC PointTypeAdapterNode : public rclcpp::Node
{
public:
  /// \brief default constructor, initializes subs and pubs
  explicit PointTypeAdapterNode(const rclcpp::NodeOptions & options);

  /// \brief Converts CloudX to CloudXYZI
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_in_to_cloud_xyzi(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_in) const;

private:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using float32_t = autoware::common::types::float32_t;
  using float64_t = autoware::common::types::float64_t;

  using PointXYZI = common::types::PointXYZI;

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_ptr_cloud_output_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ptr_cloud_input_;

  /// \brief Callback for input cloud, converts and publishes.
  /// \throws std::exception if it cannot transform.
  void callback_cloud_input(const PointCloud2::SharedPtr msg_ptr);
};
}  // namespace point_type_adapter
}  // namespace tools
}  // namespace autoware

#endif  // POINT_TYPE_ADAPTER__POINT_TYPE_ADAPTER_NODE_HPP_
