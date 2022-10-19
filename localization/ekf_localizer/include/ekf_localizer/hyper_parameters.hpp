// Copyright 2022 Autoware Foundation
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

#ifndef EKF_LOCALIZER__HYPER_PARAMETERS_HPP_
#define EKF_LOCALIZER__HYPER_PARAMETERS_HPP_

#include <rclcpp/rclcpp.hpp>

class HyperParameters
{
public:
  explicit HyperParameters(rclcpp::Node * node)
  : show_debug_info_(node->declare_parameter("show_debug_info", false)),
    ekf_rate_(node->declare_parameter("predict_frequency", 50.0)),
    ekf_dt_(1.0 / std::max(ekf_rate_, 0.1)),
    tf_rate_(node->declare_parameter("tf_rate", 10.0)),
    enable_yaw_bias_estimation_(node->declare_parameter("enable_yaw_bias_estimation", true)),
    extend_state_step_(node->declare_parameter("extend_state_step", 50)),
    pose_frame_id_(node->declare_parameter("pose_frame_id", std::string("map"))),
    pose_additional_delay_(node->declare_parameter("pose_additional_delay", 0.0)),
    pose_gate_dist_(node->declare_parameter("pose_gate_dist", 10000.0)),
    pose_smoothing_steps_(node->declare_parameter("pose_smoothing_steps", 5)),
    twist_additional_delay_(node->declare_parameter("twist_additional_delay", 0.0)),
    twist_gate_dist_(node->declare_parameter("twist_gate_dist", 10000.0)),
    twist_smoothing_steps_(node->declare_parameter("twist_smoothing_steps", 2)),
    proc_stddev_yaw_c_(node->declare_parameter("proc_stddev_yaw_c", 0.005)),
    proc_stddev_vx_c_(node->declare_parameter("proc_stddev_vx_c", 5.0)),
    proc_stddev_wz_c_(node->declare_parameter("proc_stddev_wz_c", 1.0))
  {
  }

  const bool show_debug_info_;
  const double ekf_rate_;
  const double ekf_dt_;
  const double tf_rate_;
  const bool enable_yaw_bias_estimation_;
  const int extend_state_step_;
  const std::string pose_frame_id_;
  const double pose_additional_delay_;
  const double pose_gate_dist_;
  const int pose_smoothing_steps_;
  const double twist_additional_delay_;
  const double twist_gate_dist_;
  const int twist_smoothing_steps_;
  const double proc_stddev_yaw_c_;       //!< @brief  yaw process noise
  const double proc_stddev_vx_c_;        //!< @brief  vx process noise
  const double proc_stddev_wz_c_;        //!< @brief  wz process noise
};

#endif  // EKF_LOCALIZER__HYPER_PARAMETERS_HPP_
