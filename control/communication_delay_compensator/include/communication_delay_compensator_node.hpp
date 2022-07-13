

// Copyright 2022 The Autoware Foundation
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

#ifndef COMMUNICATION_DELAY_COMPENSATOR__COMMUNICATION_DELAY_COMPENSATOR_NODE_HPP
#define COMMUNICATION_DELAY_COMPENSATOR__COMMUNICATION_DELAY_COMPENSATOR_NODE_HPP

// Base Headers

#include "eigen3/Eigen/Core"

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

// Autoware Headers
#include "common/types.hpp"
#include "control_performance_analysis/msg/error_stamped.hpp"
#include "vehicle_models/vehicle_kinematic_error_model.hpp"

#include <vehicle_info_util/vehicle_info_util.hpp>

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/controller_error_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/delay_compensation_debug.hpp"
#include "autoware_auto_vehicle_msgs/msg/delay_compensation_refs.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"


// LIBRARY HEADERS
// #include "autoware_control_toolbox.hpp"
// #include "utils_delay_observer/delay_compensation_utils.hpp"
// #include "qfilters.hpp"
#include "communication_delay_compensator_core.hpp"
#include "node_denifitions/node_definitions.hpp"

// ROS headers
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace observers
{
using namespace std::chrono_literals;
using ControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;

/**
 * @brief longitudinal_controller reports vcurrent - vtarget.
 * lateral_controller reports current_yaw - target_yaw and current_lat_distance -
 * target_lat_distance
 *
 * */
using ControllerErrorReportMsg = autoware_auto_vehicle_msgs::msg::ControllerErrorReport;

using VelocityMsg = nav_msgs::msg::Odometry;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using vehicle_info_util::VehicleInfoUtil;
// using ErrorStampedControlPerfMsg = control_performance_analysis::msg::ErrorStamped;

// Parameters to pass around.
struct Parameters
{
	float64_t wheel_base{};
	float64_t cdob_ctrl_period{0.033};

	// Qfilter orders .
	int qfilter_lateral_error_cdob_order{1};
	int qfilter_lateral_dob_order{1};
	int qfilter_longitudinal_error_order{1};

	// Qfilter cut-off frequencies Hz. (low-pass).
	float64_t qfilter_lateral_error_cdob_freq{10.};
	float64_t qfilter_lateral_dob_freq{10.};
	float64_t qfilter_longitudinal_error_freq{10.};

	// Damping
	float64_t qfilter_lateral_dob_damping{1.};

	// First order vehicle state models.
	float64_t steering_tau{0.3};
	float64_t velocity_tau{0.3};
	float64_t acc_tau{0.3};
};

template<typename T>
void update_param(
	const std::vector<rclcpp::Parameter> &parameters, const std::string &name, T &value)
{
	auto it = std::find_if(
		parameters.cbegin(), parameters.cend(),
		[&name](const rclcpp::Parameter &parameter)
		{ return parameter.get_name() == name; });
	if (it != parameters.cend())
	{
		value = static_cast<T>(it->template get_value<T>());
	}
}

// The node class.
class CommunicationDelayCompensatorNode : public rclcpp::Node
{
 public:
	/**
	 * @brief constructor
	 */
	explicit CommunicationDelayCompensatorNode(const rclcpp::NodeOptions &node_options);

	/**
	 * @brief destructor
	 */
	~CommunicationDelayCompensatorNode() override = default;

 private:
	// Data Members
	Parameters params_node_{};

	//!< @brief timer to update after a given interval
	rclcpp::TimerBase::SharedPtr timer_;

	// Subscribers
	rclcpp::Subscription<ControlCommand>::SharedPtr sub_control_cmds_;

	//!< @brief subscription for current velocity
	rclcpp::Subscription<VelocityMsg>::SharedPtr sub_current_velocity_ptr_;

	//!< @brief subscription for current velocity
	rclcpp::Subscription<SteeringReport>::SharedPtr sub_current_steering_ptr_;

	//!< @brief subscription for current velocity error.
	rclcpp::Subscription<ControllerErrorReportMsg>::SharedPtr sub_current_long_error_ptr_;

	//!< @brief subscription for current lateral and heading errors.
	rclcpp::Subscription<ControllerErrorReportMsg>::SharedPtr sub_current_lat_errors_ptr_;

	//!< @brief subscription for current lateral and heading errors.
	// rclcpp::Subscription<ErrorStampedControlPerfMsg>::SharedPtr sub_control_perf_errors_ptr_;

	// Publishers
	rclcpp::Publisher<DelayCompensatatorMsg>::SharedPtr pub_delay_compensator_;
	rclcpp::Publisher<DelayCompensatorDebugMsg>::SharedPtr pub_delay_compensator_debug_;

	// Data Members for the delay-compensation
	// CDOB: Communication Disturbance Observer-based.
	std::unique_ptr<LateralCommunicationDelayCompensator> cdob_lateral_ptr_{};

	/**
	 * @brief vehicle model that simulates the physical input.
	 * */
	std::shared_ptr<observers::linear_vehicle_model_t> vehicle_model_ptr_;

	/**
	* @brief observer vehicle model for state estimation.
	* */
	std::shared_ptr<observers::linear_state_observer_model_t> dist_td_obs_vehicle_model_ptr_;

	/**
	* @brief observer vehicle model for input disturbance estimation estimation.
	* */
	std::shared_ptr<observers::LateralDisturbanceCompensator> dob_lateral_ptr_;

	// Pointers to the ROS topics.
	// Pointers for ros topic
	// Pointers to the model state variables inputs
	std::shared_ptr<nav_msgs::msg::Odometry> current_velocity_ptr_{nullptr};

	std::shared_ptr<SteeringReport> current_steering_ptr_{nullptr};
	std::shared_ptr<SteeringReport> prev_steering_ptr_{nullptr};

	// Pointer to the model inputs
	std::shared_ptr<ControlCommand> current_control_cmd_ptr_{nullptr};
	std::shared_ptr<ControlCommand> previous_control_cmd_ptr_{nullptr};

	// Pointers to messages.
	std::shared_ptr<DelayCompensatatorMsg> current_delay_ref_msg_ptr_{nullptr};
	std::shared_ptr<ControllerErrorReportMsg> current_lat_errors_ptr_{nullptr};
	std::shared_ptr<ControllerErrorReportMsg> current_long_errors_ptr_{nullptr};
	std::shared_ptr<ControllerErrorReportMsg> prev_lat_errors_ptr_{nullptr};
	std::shared_ptr<ControllerErrorReportMsg> prev_long_errors_ptr_{nullptr};

	// Steering related.
	float64_t current_curvature_{};
	float64_t prev_curvature_{};

	float64_t current_ideal_steering_{};
	float64_t prev_ideal_steering_{};

	float64_t previous_steering_angle_{};
	float64_t current_steering_angle_{};

	float64_t previous_velocity_{};
	float64_t current_velocity_{};
	float64_t previous_target_velocity_{1.};
	float64_t current_target_velocity_{1.};
	bool is_vehicle_stopped_{};
	// placeholders.
	state_vector_vehicle_t current_lat_measurements_{state_vector_vehicle_t::Zero()};

	// Debug messages
	std::shared_ptr<DelayCompensatorDebugMsg> current_delay_debug_msg_{nullptr};

	// Node Methods
	//!< initialize timer to work in real, simulation, and replay
	void initTimer(float64_t period_s);

	/**
	 * @brief compute and publish the compensating reference signals for the controllers with a
	 * constant control period
	 */
	void onTimer();

	/**
	 * @brief Subscription callbacks
	 */
	void onControlCommands(const ControlCommand::SharedPtr msg);

	/**
	 * @brief Subscription callbacks
	 */
	void onCurrentVelocity(const VelocityMsg::SharedPtr msg);

	/**
	 * @brief Subscription callbacks
	 */
	void onCurrentSteering(const SteeringReport::SharedPtr msg);

	/**
	 * @brief Subscription to computed velocity error
	 */
	void onCurrentLongitudinalError(const ControllerErrorReportMsg::SharedPtr msg);

	/**
	 * @brief Subscription to lateral reference errors ey, eyaw.
	 */
	void onCurrentLateralErrors(const ControllerErrorReportMsg::SharedPtr msg);

	/**
	 * @brief Subscription to control performance errors.
	 */
	// void onControlPerfErrors(const ErrorStampedControlPerfMsg::SharedPtr msg);

	/**
	 * @brief Publish message.
	 * */

	void publishCompensationReferences();

	/**
	 * @brief Check if data flows.
	 * */
	bool8_t isDataReady();

	/**
	 * @brief Default parameters of the parameters.
	 * */

	void readAndLoadParameters(observers::sLyapMatrixVecs &lyap_mats);

	/**
	 * @brief Dynamic update of the parameters.
	 * */
	OnSetParametersCallbackHandle::SharedPtr is_parameters_set_res_;

	rcl_interfaces::msg::SetParametersResult onParameterUpdate(
		const std::vector<rclcpp::Parameter> &parameters);

	/**
	 * @brief checks if vehicle is stopping.
	 * */
	bool8_t isVehicleStopping();

	/**
	 * @brief updates the vehicle model.
	 * */
	void updateVehicleModelsWithPreviousTargets();

	/**
 * @brief updates the vehicle model.
 * */
	void updateVehicleModelsWithCurrentTargets();
	/**
	 * @brief Sets the lateral delay compensator.
	 * */
	void setLateralCDOB_DOBs(sLyapMatrixVecs const &lyap_matsXY);

	void computeLateralCDOB();
};

}  // namespace observers
#endif  // COMMUNICATION_DELAY_COMPENSATOR__COMMUNICATION_DELAY_COMPENSATOR_NODE_HPP
