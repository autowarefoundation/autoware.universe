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

namespace pose_estimator_arbiter
{
template <typename T>
struct CallbackInvokingVariable
{
  CallbackInvokingVariable() {}
  explicit CallbackInvokingVariable(T initial_data) : data(initial_data) {}

  void set_and_invoke(T value)
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

struct SharedData
{
public:
  using Image = sensor_msgs::msg::Image;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using InitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;

  SharedData() {}

  // Used for sub arbiter
  CallbackInvokingVariable<PoseCovStamped::ConstSharedPtr> eagleye_output_pose_cov;
  CallbackInvokingVariable<Image::ConstSharedPtr> artag_input_image;
  CallbackInvokingVariable<PointCloud2::ConstSharedPtr> ndt_input_points;
  CallbackInvokingVariable<Image::ConstSharedPtr> yabloc_input_image;
  // Used for switch rule
  CallbackInvokingVariable<PoseCovStamped::ConstSharedPtr> localization_pose_cov;
  CallbackInvokingVariable<PointCloud2::ConstSharedPtr> point_cloud_map;
  CallbackInvokingVariable<HADMapBin::ConstSharedPtr> vector_map;
  CallbackInvokingVariable<InitializationState::ConstSharedPtr> initialization_state{
    std::make_shared<InitializationState>(
      InitializationState{}.set__state(InitializationState::UNINITIALIZED))};
};

}  // namespace pose_estimator_arbiter
#endif  // POSE_ESTIMATOR_MANAGER__SHARED_DATA_HPP_
