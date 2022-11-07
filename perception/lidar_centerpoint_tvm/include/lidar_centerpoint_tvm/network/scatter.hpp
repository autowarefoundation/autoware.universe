// Copyright 2022 AutoCore Ltd., TIER IV, Inc.
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

#ifndef LIDAR_CENTERPOINT_TVM__NETWORK__SCATTER_HPP_
#define LIDAR_CENTERPOINT_TVM__NETWORK__SCATTER_HPP_

#include <common/types.hpp>
#include <vector>
#include <lidar_centerpoint_tvm/centerpoint_config.hpp>

namespace autoware
{
namespace perception
{
namespace lidar_centerpoint_tvm
{

using autoware::common::types::float32_t;

void scatterFeatures(
  const std::vector<float32_t> & pillar_features, const std::vector<int32_t> & coords,
  const std::size_t num_pillars, const CenterPointConfig & config,
  std::vector<float32_t> & scattered_features);

}  // namespace lidar_centerpoint_tvm
}  // namespace perception
}  // namespace autoware

#endif  // LIDAR_CENTERPOINT_TVM__NETWORK__SCATTER_HPP_
