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

#include "gtest/gtest.h"
#include "point_type_adapter/point_type_adapter_node.hpp"

#include <common/types.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <limits>
#include <memory>
#include <vector>

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

struct PointSvl
{
  float32_t x{0.0F};
  float32_t y{0.0F};
  alignas(float64_t) float32_t z{0.0F};
  alignas(std::uint64_t) std::uint8_t intensity{0};
  float64_t timestamp{0.0};
  friend bool operator==(const PointSvl & p1, const PointSvl & p2) noexcept
  {
    using autoware::common::helper_functions::comparisons::rel_eq;
    return rel_eq(p1.x, p2.x, std::numeric_limits<float32_t>::epsilon()) &&
           rel_eq(p1.y, p2.y, std::numeric_limits<float32_t>::epsilon()) &&
           rel_eq(p1.z, p2.z, std::numeric_limits<float32_t>::epsilon()) &&
           p1.intensity == p2.intensity &&
           rel_eq(p1.timestamp, p2.timestamp, std::numeric_limits<float64_t>::epsilon());
  }
};

TEST(TestPointTypeAdapter, TestCloudConverter)
{
  using PointXYZI = autoware::common::types::PointXYZI;
  using sensor_msgs::msg::PointCloud2;
  PointCloud2::SharedPtr cloud_svl_ptr = std::make_shared<PointCloud2>();
  using CloudModifierSvl = point_cloud_msg_wrapper::PointCloud2Modifier<PointSvl>;
  CloudModifierSvl cloud_modifier_svl(*cloud_svl_ptr, "frame_original");
  cloud_modifier_svl.push_back(PointSvl{3.0F, 4.0F, 5.0F, 100, 123456789});
  cloud_modifier_svl.push_back(PointSvl{6.0F, 8.0F, 10.0F, 200, 123456789});

  PointXYZI point_xyzi_0{3.0F, 4.0F, 5.0F, 100};
  PointXYZI point_xyzi_1{6.0F, 8.0F, 10.0F, 200};

  rclcpp::init(0, nullptr);
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("name_topic_cloud_in", "/dummy1");
  params.emplace_back("name_topic_cloud_out", "/dummy2");

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);
  autoware::tools::point_type_adapter::PointTypeAdapterNode point_type_adapter_node(node_options);

  PointCloud2::SharedPtr cloud_xyzi_ptr =
    point_type_adapter_node.cloud_in_to_cloud_xyzi(cloud_svl_ptr);
  rclcpp::shutdown();

  using CloudViewXyzi = point_cloud_msg_wrapper::PointCloud2View<PointXYZI>;
  CloudViewXyzi cloud_view_xyzi(*cloud_xyzi_ptr);
  EXPECT_EQ(cloud_xyzi_ptr->header, cloud_svl_ptr->header);
  EXPECT_EQ(cloud_xyzi_ptr->width, cloud_svl_ptr->width);
  EXPECT_EQ(cloud_xyzi_ptr->fields.size(), 4UL);
  EXPECT_EQ(cloud_view_xyzi.at(0), point_xyzi_0);
  EXPECT_EQ(cloud_view_xyzi.at(1), point_xyzi_1);
}
