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

#ifndef POINT_TYPE_ADAPTER__POINT_CLOUD2_INTENSITY_WRAPPER_HPP_
#define POINT_TYPE_ADAPTER__POINT_CLOUD2_INTENSITY_WRAPPER_HPP_

#include <common/types.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cstdint>

class IntensityIteratorWrapper
{
private:
  using float32_t = autoware::common::types::float32_t;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  sensor_msgs::PointCloud2ConstIterator<uint8_t> m_intensity_it_uint8;
  sensor_msgs::PointCloud2ConstIterator<float32_t> m_intensity_it_float32;
  decltype(sensor_msgs::msg::PointField::datatype) m_intensity_datatype;

public:
  explicit IntensityIteratorWrapper(const PointCloud2 & msg)
  : m_intensity_it_uint8(msg, "intensity"), m_intensity_it_float32(msg, "intensity")
  {
    auto intensity_field_it = std::find_if(
      std::cbegin(msg.fields), std::cend(msg.fields),
      [](const sensor_msgs::msg::PointField & field) { return field.name == "intensity"; });
    if (intensity_field_it == msg.fields.cend()) {
      throw std::runtime_error("Required field \"intensity\" doesn't exit in the point cloud.");
    }
    m_intensity_datatype = intensity_field_it->datatype;
    switch (m_intensity_datatype) {
      case sensor_msgs::msg::PointField::UINT8:
      case sensor_msgs::msg::PointField::FLOAT32:
        break;
      default:
        throw std::runtime_error(
          "Intensity type not supported: " + std::to_string(m_intensity_datatype));
    }
  }

  bool is_end()
  {
    switch (m_intensity_datatype) {
      // For some reason, the equality operator (==) does not work with PointCloud2ConstIterator
      case sensor_msgs::msg::PointField::UINT8:
        return !(m_intensity_it_uint8 != m_intensity_it_uint8.end());
      case sensor_msgs::msg::PointField::FLOAT32:
        return !(m_intensity_it_float32 != m_intensity_it_float32.end());
      default:
        throw std::runtime_error(
          "Intensity type not supported: " + std::to_string(m_intensity_datatype));
    }
  }

  void increase()
  {
    switch (m_intensity_datatype) {
      case sensor_msgs::msg::PointField::UINT8:
        ++m_intensity_it_uint8;
        break;
      case sensor_msgs::msg::PointField::FLOAT32:
        ++m_intensity_it_float32;
        break;
      default:
        throw std::runtime_error(
          "Intensity type not supported: " + std::to_string(m_intensity_datatype));
    }
  }

  template <typename PointFieldValueT>
  PointFieldValueT get_current_value()
  {
    switch (m_intensity_datatype) {
      case sensor_msgs::msg::PointField::UINT8:
        return *m_intensity_it_uint8;
      case sensor_msgs::msg::PointField::FLOAT32:
        return *m_intensity_it_float32;
      default:
        throw std::runtime_error(
          "Intensity type not supported: " + std::to_string(m_intensity_datatype));
    }
  }
};

#endif  // POINT_TYPE_ADAPTER__POINT_CLOUD2_INTENSITY_WRAPPER_HPP_
