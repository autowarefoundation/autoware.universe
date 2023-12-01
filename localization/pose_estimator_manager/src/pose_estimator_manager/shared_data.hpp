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

#include <memory>
#include <optional>

namespace pose_estimator_manager
{
template <typename T>
struct CallbackInvolvingVariable
{
  CallbackInvolvingVariable() {}
  explicit CallbackInvolvingVariable(T initial_data) : data(initial_data) {}

  void set(T value)
  {
    data = value;
    for (const auto & c : callbacks) {
      c(data.value());
    }
  }

  bool has_value() const { return data.has_value(); }

  const T operator()() const { return data.value(); }

  void set_callback(std::function<void(T)> callback) const { callbacks.push_back(callback); }

private:
  std::optional<T> data{std::nullopt};
  mutable std::vector<std::function<void(T)>> callbacks;
};

// template <typename T>
// struct TrackableData
// {
//   explicit TrackableData(T initial_data) : updated(false) { data = initial_data; }
//   TrackableData() : updated(false) {}

//   void set(const T & new_data)
//   {
//     data = new_data;
//     updated = true;
//   }

//   void reset_update_flag() { updated = false; }

//   bool has_value() const { return data.has_value(); }

//   const T operator()() const { return data.value(); }

//   bool updated;

// private:
//   std::optional<T> data{std::nullopt};
// };

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
  CallbackInvolvingVariable<PoseCovStamped::ConstSharedPtr> eagleye_output_pose_cov;
  CallbackInvolvingVariable<Image::ConstSharedPtr> artag_input_image;
  CallbackInvolvingVariable<PointCloud2::ConstSharedPtr> ndt_input_points;
  CallbackInvolvingVariable<Image::ConstSharedPtr> yabloc_input_image;
  // Used for switch rule
  CallbackInvolvingVariable<PoseCovStamped::ConstSharedPtr> localization_pose_cov;
  CallbackInvolvingVariable<PointCloud2::ConstSharedPtr> point_cloud_map;
  CallbackInvolvingVariable<HADMapBin::ConstSharedPtr> vector_map;
  CallbackInvolvingVariable<InitializationState::ConstSharedPtr> initialization_state{
    std::make_shared<InitializationState>(
      InitializationState{}.set__state(InitializationState::UNINITIALIZED))};

  void reset_update_flag()
  {
    // eagleye_output_pose_cov.reset_update_flag();
    // artag_input_image.reset_update_flag();
    // ndt_input_points.reset_update_flag();
    // yabloc_input_image.reset_update_flag();

    // localization_pose_cov.reset_update_flag();
    // point_cloud_map.reset_update_flag();
    // vector_map.reset_update_flag();
    // initialization_state.reset_update_flag();
  }
};

}  // namespace pose_estimator_manager
#endif  // POSE_ESTIMATOR_MANAGER__SHARED_DATA_HPP_
