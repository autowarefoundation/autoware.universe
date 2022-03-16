// Copyright 2022 Tier IV, Inc.
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

#ifndef POINTCLOUD_PREPROCESSOR__POINTCLOUD_RETURN_TYPE_HPP_
#define POINTCLOUD_PREPROCESSOR__POINTCLOUD_RETURN_TYPE_HPP_

namespace pointcloud_preprocessor
{
enum ReturnType : uint8_t {
  INVALID = 0,
  SINGLE_STRONGEST,
  SINGLE_LAST,
  DUAL_STRONGEST_FIRST,
  DUAL_STRONGEST_LAST,
  DUAL_WEAK_FIRST,
  DUAL_WEAK_LAST,
  DUAL_ONLY,
};

}  // namespace pointcloud_preprocessor

namespace return_type_cloud
{
struct PointXYZIRADT
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  float azimuth;
  float distance;
  std::uint8_t return_type;
  double time_stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace return_type_cloud

POINT_CLOUD_REGISTER_POINT_STRUCT(
  return_type_cloud::PointXYZIRADT,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(
    float, azimuth, azimuth)(float, distance, distance)(double, time_stamp, time_stamp)(
    std::uint8_t, return_type, return_type))

#endif  // POINTCLOUD_PREPROCESSOR__POINTCLOUD_RETURN_TYPE_HPP_
