/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NONLINEAR_MPC_CORE__NMPC_CORE_HPP_
#define NONLINEAR_MPC_CORE__NMPC_CORE_HPP_

#include <memory>
#include <string>
#include <vector>
#include "active_model.hpp"
#include "data_and_parameter_container.hpp"
#include "initialization_lpv.hpp"
#include "nmpc_discretization.hpp"
#include "nmpc_optimization.hpp"
#include "nmpc_simulation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils/nmpc_utils.hpp"

namespace ns_nmpc_interface
{
/** @brief OSQP problem structure type definition. */
using optproblem_type = ns_opt::OptimizationProblemOSQP<Model::state_dim,
																												Model::input_dim,
																												ns_nmpc_interface::MPC_NUM_OF_PRED_STEPS>;

/**
 * @brief An interface to the NMPC algorithms and data structures.
 * */
class NonlinearMPCController
{

 public:
	// User initialization.
	NonlinearMPCController(Model::model_ptr_t const &model_ptr,
												 ns_data::data_nmpc_core_type_t const &data_nmpc_core,
												 ns_data::param_lpv_type_t const &params_lpv,
												 ns_data::ParamsOptimization const &params_opt);

	// Copy constructors and assignments.
	NonlinearMPCController(NonlinearMPCController const &other);

	NonlinearMPCController &operator=(NonlinearMPCController const &other);

	// Move constructor and assignment.
	// NonlinearMPCController(NonlinearMPCController &&other) noexcept;
	// NonlinearMPCController &operator=(NonlinearMPCController &&other) noexcept;

	// Destructor.
	~NonlinearMPCController() = default;

	// Setters
	/**
	 * @brief sets the raw MPCTrajectoryVectors pointer.
	 * @param MPCtrajs_raw trajectory class that keeps the raw trajectory received from the planner modules.
	 */
	void setMPCtrajectoryRawVectorsPtr(ns_data::MPCdataTrajectoryVectors const &MPCtrajs_raw);

	void setMPCtrajectorySmoothVectorsPtr(ns_data::MPCdataTrajectoryVectors const &MPCtrajs_smoothed);

	void setCurrentAvgMPCComputationTime(const double &avg_mpc_computation_time);

	void updateInitialStates_x0(Model::state_vector_t const &x0);

	/**
	 * @brief simulate the model equations given a control [vx, steering]_inputs and an initial state.
	 * @param u control signal to be applied,
	 * @param kappa current_curvature,
	 * @param dt simulation time step,
	 * @param xk the initial state to be propagated by the simulator.
	 * */
	void simulateOneStep(Model::input_vector_t const &u, Model::param_vector_t const &params, double const &dt,
											 Model::state_vector_t &xk) const;

	/**
	 * @brief simulate the model equations given a control [steering only], vx from trajectory planner and an
	 * initial state.
	 * @param u control signal to be applied,
	 * @param kappa current_curvature,
	 * @param dt simulation time step,
	 * @param xk the initial state to be propagated by the simulator.
	 * */

	void simulateOneStepVariableSpeed(Model::input_vector_t const &u,
																		Model::param_vector_t const &params,
																		const double &v0,
																		const double &v1,
																		double const &dt,
																		Model::state_vector_t &xk) const;

	/**
	 * @brief simulate a given control sequence successively and store the states in the data containers.
	 * */
	void simulateControlSequenceByPredictedInputs(Model::state_vector_t const &x0_predicted,
																								ns_nmpc_splines::InterpolatingSplinePCG const &piecewise_interpolator);

	void simulateControlSequenceUseVaryingSpeed(Model::state_vector_t const &x0_predicted,
																							ns_nmpc_splines::InterpolatingSplinePCG const &piecewise_interpolator);

	/**
	 * @brief Sets the reference states to be tracked. In this application, since lateral and heading error is
	 * required to be zero and we use the error dynamics, there is no reference state to track except the longitudinal speed.
	 * */
	void updateRefTargetStatesByTimeInterpolation(double const &current_avg_mpc_comp_time);

	// Interpolates the velocity based on the estimated trajectory path length.
	void updateScaledPredictedTargetStatesByArcLength(double const &current_predicted_s0);

	bool reInitializeTrajectories(ns_nmpc_splines::InterpolatingSplinePCG const &piecewise_interpolator);

	bool initializeTrajectories(ns_nmpc_splines::InterpolatingSplinePCG const &piecewise_interpolator,
															bool use_linear_initialization = false);

	bool linearTrajectoryInitialization(ns_nmpc_splines::InterpolatingSplinePCG const &piecewise_interpolator);

	void setCurrent_s0(double const &s0);

	void setCurrent_t0(double const &t0);

	void setCurrent_s0_predicted(double const &s0_predicted);

	void setLoggerName(std::string_view const &logger_name);



	// LPV control methods.
	/**
	 * @brief LPV control methods.
	 * @param u_model_solution_:  [vx, steering] inputs
	 * */
	[[maybe_unused]] void computeSteeringFeedbackControls(ns_nmpc_splines::InterpolatingSplinePCG const &piecewise_interpolator,
																												double const &dt,
																												Model::input_vector_t &u_solution);

	// NMPC solution medhods.
	bool solveNMPC_problem(ns_nmpc_splines::InterpolatingSplinePCG const &piecewise_interpolator);

	// Get the solution from OSQP and shift the trajectories.
	void readSolutionsFromOSQP();

	// Shift controls to predict the next reference trajectories.
	void shiftControls();

	void getControlSolutions(Model::input_vector_t &u_solution);  // [ax, steering_rate]

	/**
	 * @brief apply state constraints to the given index.
	 * */
	void applyStateConstraints(Eigen::Index const &idx, Model::state_vector_t &x);

	void applyControlConstraints(Eigen::Index const &idx, Model::input_vector_t &u);

	void applyControlConstraints(Model::input_vector_t &u);

	// Getters.
	void getRawDistanceAtIdx(size_t const &idx, double &s_distance) const;

	void getRawRelativeTimeAtIdx(size_t const &idx, double &t_time) const;

	// !<-@brief returns current smooth target trajectory yaw angle.
	[[nodiscard]] std::array<double, 3> getSmooth_XYYawAtCurrentDistance() const;

	void getRawVxAtDistance(double const &s0, double &vx) const;

	void getSmoothVxAtDistance(double const &s0, double &vx) const;

	/** @brief gets the base arc-length coordinates from the raw trajectory data. */
	void getPlannerTravelledDistanceVector(std::vector<double> &s_distance_vector) const;

	/** @brief gets the time-vx table from the smoothed trajectory data. */
	void getTimeSpeedVectsFromSmoothTraj(std::vector<std::vector<double>> &t_speed_vects) const;

	void getInitialState(Model::state_vector_t &x0) const;

	/**
	 * @brief gets the current rate of change of speed from the system equations.
	 * @param [in] current_long_speed current predicted speed
	 * @param [in] current_speed_input current predicted speed control value
	 * @param [out] vdot rate of change of speed
	 * */
	void getSpeedDynamics_vdot(
		const double &current_long_speed, const double &current_speed_input, double &vdot) const;

	/**
	 * @brief gets the current rate of change of steering from the system equations.
	 * @param [in] current_steering current predicted steering
	 * @param [in] current_steering_input current predicted steering control value
	 * @param [out] vdot rate of change of steering
	 * */
	void getSteeringDynamics_deltadot(const double &current_steering,
																		const double &current_steering_input,
																		double &delta_dot) const;

	// Given a speed trajectory, predict the travelled distance depending on the speed.
	void getPredictedArcLengthDistanceVector(std::vector<double> &s_predicted, double const &current_predicted_s0) const;

	// Gets the total cost (value function value from the OSQP object.)
	[[nodiscard]] double getObjectiveValue() const;

	// For visualization markers.
	[[nodiscard]] trajectory_data_t getCurrentTrajectoryData() const;

	[[nodiscard]] bool isInitialized() const;

 private:
	size_t K_mpc_steps{MPC_NUM_OF_PRED_STEPS};

	/**
	 * @brief Pointer to the active vehicle model.
	 * */
	Model::model_ptr_t model_ptr_{nullptr};  // shared pointer to the model_ptr_ to be used

	// MPC algorithm parameters.
	/**
	 * @brief Parameter and data struct that holds Eigen states, control and reference vectors along with the
	 * required parameters for the NMPC and member classes.
	 * */
	ns_data::data_nmpc_core_type_t data_nmpc_;
	ns_data::param_lpv_type_t params_lpv_;
	ns_data::ParamsOptimization params_opt_;

	/**
	 * @brief for initialization of the trajectories by LPV feedback.
	 * */
	LPVinitializer lpv_initializer_;

	// @<-brief an interface to the osqp problem.
	optproblem_type osqp_interface_;

	// Pointers to the node members.
	std::unique_ptr<ns_data::MPCdataTrajectoryVectors> current_MPCtraj_raw_vects_ptr_{nullptr};
	std::unique_ptr<ns_data::MPCdataTrajectoryVectors> current_MPCtraj_smooth_vects_ptr_{nullptr};

	// Class states.
	Model::input_vector_t u_solution_last_{};  // [vx, steering]_inputs

	double current_t0_{};
	double current_s0_{};
	double current_s0_predicted_{};
	double current_avg_mpc_computation_time_{};

	bool initialized_{false};

	// Node logger name.
	std::string node_logger_name_;
};

}  // namespace ns_nmpc_interface
#endif  // NONLINEAR_MPC_CORE__NMPC_CORE_HPP_
