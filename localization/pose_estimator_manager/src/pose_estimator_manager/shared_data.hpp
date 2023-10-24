// Copyright 2023 Autoware Foundation
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

#ifndef POSE_ESTIMATOR_MANAGER__SHARED_DATA_HPP_
#define POSE_ESTIMATOR_MANAGER__SHARED_DATA_HPP_

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pose_estimator_manager
{
template <typename T>
struct TrackableData
{
  TrackableData() : updated_(false) {}

  void set(const T & data)
  {
    data_ = data;
    updated_ = true;
  }

  void reset_update_flag() { updated_ = false; }

  const T operator()() const { return data_; }

  bool updated_;

private:
  T data_;
};

struct SharedData
{
public:
  using Image = sensor_msgs::msg::Image;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  SharedData() {}
  TrackableData<PoseCovStamped::ConstSharedPtr> eagleye_output_pose_cov_;
  TrackableData<Image::ConstSharedPtr> artag_input_image_;
  TrackableData<PointCloud2::ConstSharedPtr> ndt_input_points_;
  TrackableData<Image::ConstSharedPtr> yabloc_input_image_;

  void reset_update_flag()
  {
    eagleye_output_pose_cov_.reset_update_flag();
    artag_input_image_.reset_update_flag();
    ndt_input_points_.reset_update_flag();
    yabloc_input_image_.reset_update_flag();
  }
};

}  // namespace pose_estimator_manager
#endif  // POSE_ESTIMATOR_MANAGER__SHARED_DATA_HPP_