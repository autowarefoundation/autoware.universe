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

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <common/types.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <memory>
#include <algorithm>
#include <exception>
#include <string>
#include <vector>
#include "point_type_adapter/point_type_adapter_node.hpp"

namespace
{
const std::uint32_t QOS_HISTORY_DEPTH = 10;
}

namespace autoware
{
namespace tools
{
namespace point_type_adapter
{
using common::types::PointXYZI;
using sensor_msgs::msg::PointCloud2;

PointTypeAdapterNode::PointTypeAdapterNode(const rclcpp::NodeOptions & options)
:  Node("point_type_adapter", options),
  pub_ptr_cloud_output_{this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "points_xyzi",
      rclcpp::QoS(rclcpp::KeepLast(::QOS_HISTORY_DEPTH)))}
  ,
  sub_ptr_cloud_input_(this->create_subscription<PointCloud2>(
      "points_raw",
      rclcpp::QoS(rclcpp::KeepLast(::QOS_HISTORY_DEPTH)),
      std::bind(&PointTypeAdapterNode::callback_cloud_input,
      this,
      std::placeholders::_1)))
{
}

void PointTypeAdapterNode::callback_cloud_input(const PointCloud2::SharedPtr msg_ptr)
{
  try {
    PointCloud2::SharedPtr cloud_out = cloud_in_to_cloud_xyzi(msg_ptr);
    pub_ptr_cloud_output_->publish(*cloud_out);
  } catch (std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(),
      ("Exception occured: " + std::string(ex.what())).c_str());
    rclcpp::shutdown();
  }
}

PointCloud2::SharedPtr PointTypeAdapterNode::cloud_in_to_cloud_xyzi(
  const PointCloud2::ConstSharedPtr cloud_in) const
{
  using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI>;
  PointCloud2::SharedPtr cloud_out_ptr = std::make_shared<PointCloud2>();

  auto fields_contain_field_with_name_and_datatype = [this](
    const std::vector<sensor_msgs::msg::PointField> & fields,
    const std::string & name,
    uint8_t datatype) {
      auto iter_search = std::find_if(
        fields.cbegin(), fields.cend(), [&name](
          const sensor_msgs::msg::PointField & field) {
          return field.name == name;
        });
      if (iter_search == fields.cend()) {
        // Given field doesn't exist within given point cloud.
        RCLCPP_ERROR(
          this->get_logger(),
          ("Field named \"" + name + "\" doesn't exist within given point cloud.").c_str());
        return false;
      }
      // Given field exists within given point cloud, check its type.
      return iter_search->datatype == datatype;
    };

  if (!fields_contain_field_with_name_and_datatype(
      cloud_in->fields, "x",
      sensor_msgs::msg::PointField::FLOAT32) ||
    !fields_contain_field_with_name_and_datatype(
      cloud_in->fields, "y",
      sensor_msgs::msg::PointField::FLOAT32) ||
    !fields_contain_field_with_name_and_datatype(
      cloud_in->fields, "z",
      sensor_msgs::msg::PointField::FLOAT32))
  {
    throw std::runtime_error("x,y,z fields either don't exist or they are not FLOAT32");
  }

  // Throws if "intensity" field doesn't exist
  // or the field isn't with uint8_t or float32_t datatypes
  IntensityIteratorWrapper intensity_iter_wrapper(*cloud_in);

  sensor_msgs::PointCloud2ConstIterator<float32_t> iter_x(*cloud_in, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> iter_y(*cloud_in, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> iter_z(*cloud_in, "z");


  CloudModifier cloud_modifier_out(*cloud_out_ptr, cloud_in->header.frame_id);
  cloud_out_ptr->header = cloud_in->header;

  cloud_modifier_out.reserve(cloud_in->width);

  while (iter_x != iter_x.end() ||
    iter_y != iter_y.end() ||
    iter_z != iter_z.end() ||
    !intensity_iter_wrapper.is_end())
  {
    PointXYZI point_xyzi{*iter_x, *iter_y, *iter_z,
      intensity_iter_wrapper.get_current_value<float32_t>()};
    cloud_modifier_out.push_back(point_xyzi);
    iter_x += 1;
    iter_y += 1;
    iter_z += 1;
    intensity_iter_wrapper.increase();
  }

  return cloud_out_ptr;
}

}  // namespace point_type_adapter
}  // namespace tools
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tools::point_type_adapter::PointTypeAdapterNode)
