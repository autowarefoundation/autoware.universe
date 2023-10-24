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

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <optional>

namespace pose_estimator_manager
{
template <typename T>
struct TrackableData
{
  TrackableData(T initial_data) : updated_(false) { data_ = initial_data; }
  TrackableData() : updated_(false) {}

  void set(const T & data)
  {
    data_ = data;
    updated_ = true;
  }

  void reset_update_flag() { updated_ = false; }

  bool has_value() const { return data_.has_value(); }

  const T operator()() const { return data_.value(); }

  bool updated_;

private:
  std::optional<T> data_{std::nullopt};
};

struct SharedData
{
public:
  using Image = sensor_msgs::msg::Image;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using InitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;

  SharedData() {}

  // Used for sub manager
  TrackableData<PoseCovStamped::ConstSharedPtr> eagleye_output_pose_cov_;
  TrackableData<Image::ConstSharedPtr> artag_input_image_;
  TrackableData<PointCloud2::ConstSharedPtr> ndt_input_points_;
  TrackableData<Image::ConstSharedPtr> yabloc_input_image_;
  // Used for switch rule
  TrackableData<PoseCovStamped::ConstSharedPtr> localization_pose_cov_;
  TrackableData<PointCloud2::ConstSharedPtr> point_cloud_map_;
  TrackableData<HADMapBin::ConstSharedPtr> vector_map_;
  TrackableData<InitializationState::ConstSharedPtr> initialization_state_{
    std::make_shared<InitializationState>(
      InitializationState{}.set__state(InitializationState::UNINITIALIZED))};

  void reset_update_flag()
  {
    eagleye_output_pose_cov_.reset_update_flag();
    artag_input_image_.reset_update_flag();
    ndt_input_points_.reset_update_flag();
    yabloc_input_image_.reset_update_flag();

    localization_pose_cov_.reset_update_flag();
    point_cloud_map_.reset_update_flag();
    vector_map_.reset_update_flag();
    initialization_state_.reset_update_flag();
  }
};

}  // namespace pose_estimator_manager
#endif  // POSE_ESTIMATOR_MANAGER__SHARED_DATA_HPP_