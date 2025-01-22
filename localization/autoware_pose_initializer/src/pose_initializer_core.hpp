// Copyright 2022 The Autoware Contributors
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

#ifndef POSE_INITIALIZER_CORE_HPP_
#define POSE_INITIALIZER_CORE_HPP_

#include <autoware/component_interface_specs_universe/localization.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <autoware/universe_utils/ros/diagnostics_interface.hpp>
#include <autoware/universe_utils/ros/logger_level_configure.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <memory>

namespace autoware::pose_initializer
{
class PoseErrorCheckModule;
class StopCheckModule;
class LocalizationModule;
class GnssModule;
class EkfLocalizationTriggerModule;
class NdtLocalizationTriggerModule;

class PoseInitializer : public rclcpp::Node
{
public:
  explicit PoseInitializer(const rclcpp::NodeOptions & options);

private:
  using ServiceException = autoware::component_interface_utils::ServiceException;
  using Initialize = autoware::component_interface_specs_universe::localization::Initialize;
  using State = autoware::component_interface_specs_universe::localization::InitializationState;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  rclcpp::CallbackGroup::SharedPtr group_srv_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_reset_;
  autoware::component_interface_utils::Publisher<State>::SharedPtr pub_state_;
  autoware::component_interface_utils::Service<Initialize>::SharedPtr srv_initialize_;
  State::Message state_;
  std::array<double, 36> output_pose_covariance_{};
  std::array<double, 36> gnss_particle_covariance_{};
  std::unique_ptr<GnssModule> gnss_;
  std::unique_ptr<LocalizationModule> ndt_;
  std::unique_ptr<LocalizationModule> yabloc_;
  std::unique_ptr<StopCheckModule> stop_check_;
  std::unique_ptr<PoseErrorCheckModule> pose_error_check_;
  std::unique_ptr<EkfLocalizationTriggerModule> ekf_localization_trigger_;
  std::unique_ptr<NdtLocalizationTriggerModule> ndt_localization_trigger_;
  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;
  std::unique_ptr<autoware::universe_utils::DiagnosticsInterface> diagnostics_pose_reliable_;
  double stop_check_duration_;

  void change_node_trigger(bool flag, bool need_spin = false);
  void set_user_defined_initial_pose(
    const geometry_msgs::msg::Pose initial_pose, bool need_spin = false);
  void change_state(State::Message::_state_type state);
  void on_initialize(
    const Initialize::Service::Request::SharedPtr req,
    const Initialize::Service::Response::SharedPtr res);
  PoseWithCovarianceStamped get_gnss_pose();
};
}  // namespace autoware::pose_initializer

#endif  // POSE_INITIALIZER_CORE_HPP_
