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

#include "nonlinear_mpc_core/nmpc_core.hpp"
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

ns_nmpc_interface::NonlinearMPCController::NonlinearMPCController(Model::model_ptr_t model_ptr,
                                                                  ns_data::data_nmpc_core_type_t const &data_nmpc_core,
                                                                  ns_data::param_lpv_type_t const &params_lpv,
                                                                  ns_data::ParamsOptimization const &params_opt)
  : model_ptr_{std::move(model_ptr)},
    data_nmpc_{data_nmpc_core},
    params_lpv_(params_lpv),
    params_opt_(params_opt),
    lpv_initializer_(LPVinitializer(params_lpv.num_of_nonlinearities)),
    osqp_interface_(optproblem_type())
{}

// Copy constructors and assignments.
ns_nmpc_interface::NonlinearMPCController::NonlinearMPCController(
  const ns_nmpc_interface::NonlinearMPCController &other)
  : model_ptr_(other.model_ptr_),
    data_nmpc_(other.data_nmpc_),
    params_lpv_(other.params_lpv_),
    params_opt_(other.params_opt_),
    lpv_initializer_(other.lpv_initializer_),
    osqp_interface_(other.osqp_interface_)
{
}

ns_nmpc_interface::NonlinearMPCController &
ns_nmpc_interface::NonlinearMPCController::operator=(ns_nmpc_interface::NonlinearMPCController const &other)
{
  if (&other != this)
  {
    model_ptr_ = other.model_ptr_;

    data_nmpc_ = other.data_nmpc_;
    params_lpv_ = other.params_lpv_;
    params_opt_ = other.params_opt_;

    lpv_initializer_ = other.lpv_initializer_;
    osqp_interface_ = other.osqp_interface_;
  }

  return *this;
}

//// Move constructor and assignment.
// ns_nmpc_interface::NonlinearMPCController::NonlinearMPCController(
//        ns_nmpc_interface::NonlinearMPCController &&other) noexcept
//{
//
//    model_ptr_ = std::move(other.model_ptr_);
//
//    data_nmpc_ = std::move(other.data_nmpc_);
//    params_lpv_ = other.params_lpv_;
//    params_opt_ = other.params_opt_;
//
//    lpv_initializer_ = std::move(other.lpv_initializer_);
//    osqp_interface_ = other.osqp_interface_;
//
//}
//
// ns_nmpc_interface::NonlinearMPCController &ns_nmpc_interface::NonlinearMPCController::operator=(
//        ns_nmpc_interface::NonlinearMPCController &&other) noexcept
// {
//    if (&other != this)
//    {
//        model_ptr_ = std::move(other.model_ptr_);
//
//        data_nmpc_ = std::move(other.data_nmpc_);
//        params_lpv_ = other.params_lpv_;
//        params_opt_ = other.params_opt_;
//
//        lpv_initializer_ = std::move(other.lpv_initializer_);
//        osqp_interface_ = other.osqp_interface_;
//    }
//
//    return *this;

// }

void ns_nmpc_interface::NonlinearMPCController::setMPCtrajectoryRawVectorsPtr(
  const ns_data::MPCdataTrajectoryVectors &MPCtrajs_raw)
{
  current_MPCtraj_raw_vects_ptr_ =
    std::make_unique<ns_data::MPCdataTrajectoryVectors>(MPCtrajs_raw);
}

void ns_nmpc_interface::NonlinearMPCController::setMPCtrajectorySmoothVectorsPtr(
  const ns_data::MPCdataTrajectoryVectors &MPCtrajs_smoothed)
{
  current_MPCtraj_smooth_vects_ptr_ =
    std::make_unique<ns_data::MPCdataTrajectoryVectors>(MPCtrajs_smoothed);
}

void ns_nmpc_interface::NonlinearMPCController::getRawDistanceAtIdx(const size_t &idx, double &s_distance) const
{
  if (idx < current_MPCtraj_raw_vects_ptr_->s.size())
  {
    s_distance = current_MPCtraj_raw_vects_ptr_->s.at(idx);
  } else
  {
    RCLCPP_ERROR(rclcpp::get_logger(node_logger_name_),
                 "[mpc_nonlinear - nmpc_core] Out of distance index ... ");
  }
}

void ns_nmpc_interface::NonlinearMPCController::getRawRelativeTimeAtIdx(const size_t &idx, double &t_time) const
{
  if (idx < current_MPCtraj_raw_vects_ptr_->t.size())
  {
    t_time = current_MPCtraj_raw_vects_ptr_->t.at(idx);
  } else
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(node_logger_name_), "[mpc_nonlinear - nmpc_core] Out of time index ... ");
  }
}

void ns_nmpc_interface::NonlinearMPCController::updateInitialStates_x0(const Model::state_vector_t &x0)
{
  data_nmpc_.trajectory_data.X[0] = x0;

  // DEBUG
  //	ns_utils::print("After updating the initial states in NMPCCore");
  //	ns_eigen_utils::printEigenMat(data_nmpc_.trajectory_data.X[0]);
  // end of DEBUG
}

void ns_nmpc_interface::NonlinearMPCController::getPlannerTravelledDistanceVector(
  std::vector<double> &s_distance_vector) const
{
  s_distance_vector = current_MPCtraj_raw_vects_ptr_->s;
}

void ns_nmpc_interface::NonlinearMPCController::getInitialState(Model::state_vector_t &x0) const
{
  x0 = data_nmpc_.trajectory_data.X[0];
}

void ns_nmpc_interface::NonlinearMPCController::applyStateConstraints(const Eigen::Index &idx, Model::state_vector_t &x)
{
  x(idx) = ns_utils::clamp(x(idx), params_opt_.xlower(idx), params_opt_.xupper(idx));
}

void ns_nmpc_interface::NonlinearMPCController::applyControlConstraints(
  const Eigen::Index &idx, Model::input_vector_t &u)
{
  u(idx) = ns_utils::clamp(u(idx), params_opt_.ulower(idx), params_opt_.uupper(idx));
}

void ns_nmpc_interface::NonlinearMPCController::applyControlConstraints(Model::input_vector_t &u)
{
  for (Eigen::Index idx = 0; idx < u.size(); ++idx)
  {
    u(idx) = ns_utils::clamp(u(idx), params_opt_.ulower(idx), params_opt_.uupper(idx));
  }

}

void ns_nmpc_interface::NonlinearMPCController::simulateOneStep(const Model::input_vector_t &u,
                                                                const Model::param_vector_t &params,
                                                                double const &dt,
                                                                Model::state_vector_t &xk) const
{
  // One-step u and kappa are kept constant.
  ns_sim::simulateNonlinearModel_zoh(model_ptr_, u, params, dt, xk);
}

void ns_nmpc_interface::NonlinearMPCController::simulateOneStepVariableSpeed(
  Model::input_vector_t const &u, const Model::param_vector_t &params, const double &v0,
  const double &v1, double const &dt, Model::state_vector_t &xk) const
{
  ns_sim::simulateNonlinearModel_variableSpeed(model_ptr_, u, params, v0, v1, dt, xk);
}

void ns_nmpc_interface::NonlinearMPCController::simulateControlSequenceByPredictedInputs(
  Model::state_vector_t const &x0_predicted,
  ns_splines::InterpolatingSplinePCG const &piecewise_interpolator)
{
  size_t const &&nX = data_nmpc_.trajectory_data.nX();  // get the size of nX.

  // Define a state placeholder for the integrator.
  auto xk = x0_predicted;

  double kappa0{};  // curvature placeholder
  Model::param_vector_t params(Model::param_vector_t::Zero());

  for (size_t k = 1; k < nX; k++)
  {  // start from k=1, x0 is already set
    // Get s0 and interpolate the curvature at this distance point.
    double const &s0 = xk(3);  // always (k-1)th. x[k-1].

    // We don't need to use could_interpolate, as the interpolator is verified inside.
    if (bool const &&could_interpolate = piecewise_interpolator.Interpolate(s0, kappa0);!could_interpolate)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(node_logger_name_),
        "[nonlinear-mpc - simulate with predicted input ],"
        "the spline could not  interpolate ...");
      return;
    }

    params(0) = kappa0;
    params(1) = data_nmpc_.target_reference_states_and_controls.X[k - 1](6);  // target vx

    // get the previous input at (k-1). to compute the states at k.
    auto uk = data_nmpc_.trajectory_data.U[k - 1];  // [vx input - m/s, steering input - rad]

    // feedforward steering
    auto const &&uff = atan(kappa0 * data_nmpc_.wheel_base);
    uk(1) += uff;

    simulateOneStep(uk, params, data_nmpc_.mpc_prediction_dt, xk);

    // Make sure that steering stays in its limits.
    applyControlConstraints(uk);
    applyStateConstraints(7, xk);

    // Put xk at k to Xref.
    data_nmpc_.trajectory_data.X[k] = xk;  // x[k]
  }

  // DEBUG
  //    auto &&Xtemp = ns_eigen_utils::getTrajectory(data_nmpc_.trajectory_data.X);
  //    auto &&Utemp = ns_eigen_utils::getTrajectory(data_nmpc_.trajectory_data.U);
  //
  //    ns_utils::print("\nSimulated trajectories : ");
  //    ns_eigen_utils::printEigenMat(Xtemp.transpose()); //  [x, y, psi, s, ey, e_yaw, v, delta]
  //
  //    ns_utils::print("\nSimulated trajectories U : ");
  //    ns_eigen_utils::printEigenMat(Utemp.transpose());
  // end of debug
}

void ns_nmpc_interface::NonlinearMPCController::simulateControlSequenceUseVaryingSpeed(
  Model::state_vector_t const &x0_predicted,
  ns_splines::InterpolatingSplinePCG const &piecewise_interpolator)
{
  size_t const &&nX = data_nmpc_.trajectory_data.nX();  // get the size of nX

  // Define a state placeholder for the integrator.
  auto xk = x0_predicted;

  // For speed interpolation define.
  double td0 = current_t0_ + data_nmpc_.input_delay_time + current_avg_mpc_computation_time_;

  Model::param_vector_t params;
  params.setZero();

  for (size_t k = 1; k < nX - 1; k++)
  {  // start from k=1, x0 is already set
    double kappa0{};                       // curvature placeholder
    double v0{};
    double v1{};

    // Get s0 and interpolate for the curvature.
    double s0 = xk(3);  // always (k-1)th. x[k-1].

    // We don't need to use could_interpolate, as the interpolator is verified inside
    if (auto could_interpolate = piecewise_interpolator.Interpolate(s0, kappa0);!could_interpolate)
    {
      RCLCPP_ERROR(rclcpp::get_logger(node_logger_name_),
                   "[nonlinear-mpc - simulate varying speed sequence], the spline could not interpolate ...");
      return;
    }

    auto const &uk = data_nmpc_.trajectory_data.U[k - 1];

    // InterpolateInCoordinates the target v0, v1 on the trajectory.
    ns_utils::interp1d_linear(current_MPCtraj_smooth_vects_ptr_->t,
                              current_MPCtraj_smooth_vects_ptr_->vx,
                              td0,
                              v0);

    // use the mpc time step duration dt.
    ns_utils::interp1d_linear(current_MPCtraj_smooth_vects_ptr_->t, current_MPCtraj_smooth_vects_ptr_->vx,
                              td0 + data_nmpc_.mpc_prediction_dt, v1);

    params(0) = kappa0;
    params(1) = data_nmpc_.target_reference_states_and_controls.X[k - 1](6);

    simulateOneStepVariableSpeed(uk, params, v0, v1, data_nmpc_.mpc_prediction_dt, xk);

    // Put xk at k to Xref.
    data_nmpc_.trajectory_data.X[k] = xk;  // x[k]

    // ns_utils::print("[simulate_control_sequence]
    // Could interpolate : ", could_interpolate);
    // ns_eigen_utils::printEigenMat(trajectory_data_.U.at(k));
    //
    // ns_utils::print("\nSimulated one step ahead : ");
    // ns_eigen_utils::printEigenMat(trajectory_data_.X.at(k));
  }

  // DEBUG
  // auto && Xtemp = ns_eigen_utils::getTrajectory(data_nmpc_.trajectory_data.X);
  // auto && Utemp = ns_eigen_utils::getTrajectory(data_nmpc_.trajectory_data.U);

  //  ns_utils::print("\nSimulated trajectories : ");
  //  ns_eigen_utils::printEigenMat(Xtemp.transpose());  //  [x, y, psi, s, ey, e_yaw, v, delta]
  //
  //  ns_utils::print("\nSimulated trajectories U : ");
  //  ns_eigen_utils::printEigenMat(Utemp.transpose());
  // end of debug
}

bool ns_nmpc_interface::NonlinearMPCController::isInitialized() const
{ return initialized_; }

void ns_nmpc_interface::NonlinearMPCController::updateRefTargetStatesByTimeInterpolation(
  double const &current_avg_mpc_comp_time)
{
  // Prepare the target trajectory data
  // number of stored states in the horizon.
  auto const &&nX = data_nmpc_.target_reference_states_and_controls.nX();
  Model::state_vector_t xk;  // placeholder for the iterated states.

  // Prepare the start time for the MPC trajectory.
  double const &&t_mpc_start = current_t0_ + data_nmpc_.input_delay_time + current_avg_mpc_comp_time;
  double const &&t_mpc_ends = t_mpc_start + static_cast<double>(K_mpc_steps - 1) * data_nmpc_.mpc_prediction_dt;

  // to create a base time coordinate for the time-vx interpolator.
  auto const &&t_predicted_coords = ns_utils::linspace<double>(t_mpc_start, t_mpc_ends, K_mpc_steps);
  std::vector<double> vx_interpolated_vect;

  ns_splines::InterpolatingSplinePCG interpolator_time_speed(1);
  auto const &&is_interpolated = interpolator_time_speed.Interpolate(current_MPCtraj_smooth_vects_ptr_->t,
                                                                     current_MPCtraj_smooth_vects_ptr_->vx,
                                                                     t_predicted_coords,
                                                                     vx_interpolated_vect);

  if (!is_interpolated)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node_logger_name_),
                 "[mpc_nonlinear] UpdateScaledTargets couldn't interpolate the target speeds ...");
    return;
  }

  // Set the target states.
  for (size_t k = 0; k < nX; k++)
  {
    // set speed of the target states.
    xk.setZero();
    xk(6) = vx_interpolated_vect[k];

    // Scale the reference target velocity for simple feedforward-conditioning.
    data_nmpc_.target_reference_states_and_controls.X[k] = xk;
  }

  // DEBUG
  //  Get trajectories as a matrix and print for debugging purpose.
  //  auto &&Xtemp =ns_eigen_utils::getTrajectory(data_nmpc_.target_reference_states_and_controls.X)
  //
  //  ns_utils::print("\nComputed Update target trajectories : ");
  //  ns_eigen_utils::printEigenMat(Xtemp.transpose()); //  [x, y, psi, s, ey, epsi, v, delta]
  //
  //  ns_utils::print("Predicted Time Horizon ");
  //  ns_utils::print_container(t_predicted_coords);
  //
  //  ns_utils::print("Current speed vs current target speed:", data_nmpc_.trajectory_data.X[0](6),
  //  data_nmpc_.target_reference_states_and_controls.X[0](6));
  // end of debug
}

void ns_nmpc_interface::NonlinearMPCController::updateScaledPredictedTargetStatesByArcLength(
  double const &current_predicted_s0)
{
  // Prepare the target trajectory data;
  // number of stored states in the horizon.
  auto const &&nX = data_nmpc_.target_reference_states_and_controls.nX();
  Model::state_vector_t xk(Model::state_vector_t::Zero());  // place holder for the iterated integration.

  // Prepare the predicted road travelled distance vector.
  std::vector<double> s_predicted;
  getPredictedArcLengthDistanceVector(s_predicted, current_predicted_s0);

  //   Interpolate the longitudinal speed at the predicted s-distance on the path.
  /**
       * - get the current svec stored and use as the base coordinate for the interpolator.
       * - get the current vx_vector and use as the base data.
       **/

  // Use of Autoware spline.
  std::vector<double> vx_interpolated;

  // Alternatively we can use Eigen version of Autoware spline.
  ns_splines::InterpolatingSplinePCG spline_aw_eigen(1);  // linear interpolation.

  auto const &sbase = current_MPCtraj_raw_vects_ptr_->s;
  auto const &vxbase = current_MPCtraj_raw_vects_ptr_->vx;
  auto const &&is_interpolated = spline_aw_eigen.Interpolate(sbase, vxbase, s_predicted, vx_interpolated);

  if (!is_interpolated)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(node_logger_name_),
      "[mpc_nonlinear] UpdateScaledTargets couldn't interpolate the target states ...");
    return;
  }

  // Set the target states.
  for (size_t k = 0; k < nX; k++)
  {
    xk.setZero();
    xk(6) = vx_interpolated[k];  // set speed of the target states.

    // Scale the reference target velocity for simple feedforward-conditioning.
    data_nmpc_.target_reference_states_and_controls.X[k] = xk;

    // DEBUG
    // ns_utils::print("Scaled state as matrix : ");
    // ns_eigen_utils::printEigenMat(xkhat);
    //
    // ns_utils::print("Pushed scaled matrix : ");
    // ns_eigen_utils::printEigenMat(target_states_.X.at(k));
  }

  // DEBUG
  //  for (auto &sx:s_predicted)
  //  {
  //  std::cout << "Predicted s : " << sx << std::endl;
  //  }

  //  for (auto &vx:current_speed_vx_vec_ptr_)
  //  {
  //  std::cout << "Current speed vector in update targets : " << vx << std::endl;
  //  }

  //  for (size_t k = 0; k < nX; k++)
  //  {
  //  std::cout << "Current Interpolated speed vector in update targets :
  //  " << vx_interpolated[k] << " ";
  //  std::cout << " Scaled target vx " << target_states_.X.at(k)(6) << std::endl;
  //  }

  //  std::partial_sum(current_speed_vx_vec_ptr_.begin(), current_speed_vx_vec_ptr_.end(),
  //  std::ostream_iterator<int>(std::cout, " "));
  // end of debug
}

void ns_nmpc_interface::NonlinearMPCController::getPredictedArcLengthDistanceVector(std::vector<double> &s_predicted,
                                                                                    double const &current_predicted_s0) const
{
  /**
   * - Integrate the reference speed and find an estimated travelling distance on the path.
   * - InterpolateInCoordinates the reference speed given the estimated distance.
   **/

  // Get current estimate of the distance travelled and get estimate.
  std::vector<double> ds_steps;
  auto const &dt = data_nmpc_.mpc_prediction_dt;

  // Start from the current predicted path position (distance wrt the trajectory start).
  ds_steps.emplace_back(current_predicted_s0);  // start from the current distance station.

  // Get the current time to interpolate the speeds.
  // Prepare the start time for the MPC trajectory.
  double const &&t_mpc_start =
    current_t0_ + data_nmpc_.input_delay_time + current_avg_mpc_computation_time_;

  double const &&t_mpc_ends = t_mpc_start + static_cast<double>(K_mpc_steps - 1) * dt;

  // to create a base time coordinate for the time-vx interpolator.
  auto const &&t_predicted_coords = ns_utils::linspace<double>(t_mpc_start, t_mpc_ends, K_mpc_steps);
  std::vector<double> vx_interpolated_vect;

  ns_splines::InterpolatingSplinePCG time_speed_interpolator(1);

  if (bool const &&is_interpolated = time_speed_interpolator.Interpolate(current_MPCtraj_smooth_vects_ptr_->t,
                                                                         current_MPCtraj_smooth_vects_ptr_->vx,
                                                                         t_predicted_coords,
                                                                         vx_interpolated_vect);!is_interpolated)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node_logger_name_),
                 "[nonlinear_mpc]: getPredictedArcLengthDistanceVector spline interpolator failed to compute "
                 "the coefficients ...");
    return;
  }

  // Integrate the speeds by the trapezoidal rule and mpc_time step duration.
  std::transform(vx_interpolated_vect.cbegin(), vx_interpolated_vect.cend(), vx_interpolated_vect.cbegin() + 1,
                 std::back_inserter(ds_steps), [&dt](auto const &v0, auto const &v1)
                 { return (v0 + v1) * dt / 2; });

  // Integrate the ds_steps starting from current_s0_.
  std::partial_sum(ds_steps.begin(), ds_steps.end(), std::back_inserter(s_predicted));

  // Debug
  //    for (auto &ds:ds_steps)
  //    {
  //        ns_utils::print("predicted ds with a starting value set ", ds);
  //    }
  //
  //    ns_utils::print("\n");
  //    for (auto &sp:s_predicted)
  //    {
  //        ns_utils::print("partial sum of predicted ds with a starting value set ", sp);
  //    }

  //    for (size_t k = 0; k < t_predicted_coords.size(); k++)
  //    {
  //        ns_utils::print("Get predictedArcLengths given time time, ds and s_predicted :",
  //                        t_predicted_coords[k],
  //                        ds_steps[k],
  //                        s_predicted[k]);
  //    }
  // end of debug
}

bool ns_nmpc_interface::NonlinearMPCController::reInitializeTrajectories(
  ns_splines::InterpolatingSplinePCG const &piecewise_interpolator)
{
  auto const is_initialized_traj = linearTrajectoryInitialization(piecewise_interpolator);
  return is_initialized_traj;
}

bool ns_nmpc_interface::NonlinearMPCController::initializeTrajectories(
  ns_splines::InterpolatingSplinePCG const &piecewise_interpolator,
  bool use_linear_initialization)
{

  //    ns_utils::print("in intialization ");
  //    for (auto const &x: params_lpv_.lpvXcontainer)
  //    {
  //        ns_utils::print("LPV xs");
  //        ns_eigen_utils::printEigenMat(Eigen::MatrixXd(x));
  //
  //    }
  //
  //    for (auto const &y: params_lpv_.lpvYcontainer)
  //    {
  //        ns_utils::print("LPV ys");
  //        ns_eigen_utils::printEigenMat(Eigen::MatrixXd(y));
  //
  //    }

  // Initialize the trajectories.
  bool is_initialized_traj{false};
  if (use_linear_initialization)
  {
    is_initialized_traj = linearTrajectoryInitialization(piecewise_interpolator);

  } else
  {
    // ns_utils::print("calling feedback initialization ...");
    is_initialized_traj = lpv_initializer_.simulateWithFeedback(model_ptr_,
                                                                piecewise_interpolator,
                                                                params_lpv_,
                                                                params_opt_,
                                                                data_nmpc_);

  }

  // Get trajectories as a matrix and print for debugging purpose.
  auto &&Xtemp = ns_eigen_utils::getTrajectory(data_nmpc_.trajectory_data.X);
  auto &&Utemp = ns_eigen_utils::getTrajectory(data_nmpc_.trajectory_data.U);

  ns_utils::print("\n[nmpc_core in initialization] Initialized LPV trajectories :  [x, y, psi, s, ey, epsi, vx, delta, "
                  "vy]");
  ns_eigen_utils::printEigenMat(Xtemp.transpose());  //  [x, y, psi, s, ey, epsi, vx, delta, vy]

  ns_utils::print("\n [nmpc_core in initialization] Initialized LPV trajectories U : ");
  ns_eigen_utils::printEigenMat(Utemp.transpose());

  if (!is_initialized_traj)
  {
    return false;
  }

  /**
   * @brief  Once an initial reference trajectory is available we discretisize the model.  The discretization data
   * A[k], B[k],... are stored in the discretization data object.
   * */

  auto const &dt = data_nmpc_.mpc_prediction_dt;
  auto const is_discretisized = ns_discretization::multipleShootingTrajectory(model_ptr_,
                                                                              data_nmpc_.trajectory_data,
                                                                              data_nmpc_.target_reference_states_and_controls,
                                                                              piecewise_interpolator,
                                                                              dt,
                                                                              data_nmpc_.discretization_data);

  //  // Alternative methods for the discretizations.
  //  bool is_discretisized = ns_discretization::bilinearTransformation(model_ptr_,
  //                                                                    data_nmpc_.trajectory_data,
  //                                                                    piecewise_interpolator,
  //                                                                    dt,
  //                                                                    data_nmpc_.discretization_data);


  // DEBUG
  //  ns_utils::print("In the initialize trajectories");
  //  ns_utils::print("Initialized Discrete Matrices : ");
  //
  //  for (size_t k = 0; k < K_mpc_steps - 1; k++)
  //  {
  //    ns_utils::print("A[k]");
  //    ns_eigen_utils::printEigenMat(data_nmpc_.discretization_data.A.at(k));
  //
  //    ns_utils::print("B[k]");
  //    ns_eigen_utils::printEigenMat(data_nmpc_.discretization_data.B.at(k));
  //
  //    ns_utils::print("z[k]");
  //    ns_eigen_utils::printEigenMat(data_nmpc_.discretization_data.z.at(k));
  //  }
  //
  //  ns_utils::print("Are the trajectories and OSQP  initialized ? : ",
  //                  is_discretisized ? " initialized ..." : "not initialized ...");


  if (!is_discretisized)
  {
    return false;
  }

  bool is_initialized_osqp{false};
  if (!osqp_interface_.isInitialized() && is_initialized_traj && is_discretisized)
  {
    is_initialized_osqp =
      osqp_interface_.setUPOSQP_useTriplets(data_nmpc_.trajectory_data.X[0], data_nmpc_, params_opt_);

    if (is_initialized_osqp)
    {
      auto exit_code = osqp_interface_.testSolver();
      ns_utils::print("nmpc_core OSQP test solution run results : ", ToString(exit_code));

    } else
    {
      RCLCPP_ERROR(rclcpp::get_logger(node_logger_name_),
                   "[mpc_nonlinear - initialization] OSQP Matrices could "
                   "not be  initialized ...");

      return false;
    }
  }


  // DEBUG
  //  // Get trajectories as a matrix and print for debugging purpose.
  //  auto &&Xtemp = ns_eigen_utils::getTrajectory(data_nmpc_.trajectory_data.X);
  //  auto &&Utemp = ns_eigen_utils::getTrajectory(data_nmpc_.trajectory_data.U);
  //
  //  ns_utils::print("\n[in initialization] Initialized LPV trajectories :  [x, y, psi, s, ey, epsi, vx, delta, vy]");
  //  ns_eigen_utils::printEigenMat(Xtemp.transpose());  //  [x, y, psi, s, ey, epsi, vx, delta, vy]
  //
  //  ns_utils::print("\n [in initialization] Initialized LPV trajectories U : ");
  //  ns_eigen_utils::printEigenMat(Utemp.transpose());



  //    ns_utils::print("Are the trajectories and OSQP  initialized ? : ",
  //                    initialized_ ? " initialized ..." : "not initialized ...");
  // end of debug

  return false; //initialized_;
}

bool ns_nmpc_interface::NonlinearMPCController::linearTrajectoryInitialization(
  ns_splines::InterpolatingSplinePCG const &piecewise_interpolator)
{
  // Create the new s-coordinates from the predicted speed trajectory.
  std::vector<double> s_predicted_vect;
  auto s_predicted0 = current_s0_predicted_;
  getPredictedArcLengthDistanceVector(s_predicted_vect, s_predicted0);

  // Get the initial measured states.
  Model::state_vector_t xinitial_measured;
  getInitialState(xinitial_measured);

  Model::state_vector_t xk(Model::state_vector_t::Zero());
  Model::input_vector_t uk(Model::input_vector_t::Zero());

  // placeholders for kappa and s.
  double kappa0{};

  // Get predicted arc-length vector.
  std::vector<double> xpredicted;
  std::vector<double> ypredicted;
  std::vector<double> yaw_predicted;
  std::vector<double> ey_predicted;
  std::vector<double> eyaw_predicted;
  std::vector<double> vx_predicted;
  std::vector<double> ax_predicted;

  // Fill the empty predicted vectors.
  ns_utils::interp1d_map_linear(current_MPCtraj_smooth_vects_ptr_->s,
                                current_MPCtraj_smooth_vects_ptr_->x,
                                s_predicted_vect,
                                xpredicted);  // x

  ns_utils::interp1d_map_linear(current_MPCtraj_smooth_vects_ptr_->s,
                                current_MPCtraj_smooth_vects_ptr_->y,
                                s_predicted_vect,
                                ypredicted);  // y

  ns_utils::interp1d_map_linear(current_MPCtraj_smooth_vects_ptr_->s,
                                current_MPCtraj_smooth_vects_ptr_->yaw,
                                s_predicted_vect,
                                yaw_predicted);  // yaw

  ns_utils::interp1d_map_linear(current_MPCtraj_smooth_vects_ptr_->s,
                                current_MPCtraj_smooth_vects_ptr_->vx,
                                s_predicted_vect,
                                vx_predicted);  // Vx

  ns_utils::interp1d_map_linear(current_MPCtraj_smooth_vects_ptr_->s,
                                current_MPCtraj_smooth_vects_ptr_->ax,
                                s_predicted_vect,
                                ax_predicted);  // ax

  ey_predicted = ns_utils::linspace(xinitial_measured(4), 0.0, K_mpc_steps);  //  [x, y, psi, s, ey, epsi, delta]

  eyaw_predicted = ns_utils::linspace(xinitial_measured(5), 0.0, K_mpc_steps);

  // Target speeds
  ns_splines::InterpolatingSplinePCG interpolator_distance_speed(1);

  for (size_t k = 1; k < K_mpc_steps; k++)
  {
    // auto xprev = data_nmpc_.trajectory_data.X[k - 1];

    xk.setZero();
    xk(0) = xpredicted[k];        // x
    xk(1) = ypredicted[k];        // y
    xk(2) = yaw_predicted[k];     // psi
    xk(3) = s_predicted_vect[k];  // s
    xk(4) = ey_predicted[k];      // ey Initialize the errors to zero.
    xk(5) = eyaw_predicted[k];    // epsi
    xk(6) = vx_predicted[k];      // vx

    // InterpolateInCoordinates the curvature and compute the steering
    auto const sk = xk(3);

    if (bool const &could_interpolate = piecewise_interpolator.Interpolate(sk, kappa0); !could_interpolate)
    {
      RCLCPP_ERROR(rclcpp::get_logger(node_logger_name_),
                   "[nonlinear_mpc] Linear trajectory initialization, spline "
                   " interpolator failed to compute  the coefficients ...");

      return false;
    }

    double const &&steering_k = atan(kappa0 * data_nmpc_.wheel_base);

    xk(7) = steering_k;

    applyStateConstraints(6, xk);  // speed
    applyStateConstraints(7, xk);  // steering value.

    data_nmpc_.trajectory_data.X[k] = xk;

    // Initialize the controls.
    uk.setZero();

    double vx_target{};
    interpolator_distance_speed.Interpolate(current_MPCtraj_smooth_vects_ptr_->s,
                                            current_MPCtraj_smooth_vects_ptr_->vx,
                                            sk,
                                            vx_target);

    uk(0) = ax_predicted[k];    // predicted acceleration.
    uk(1) = xk(7);              // predicted feedforward steering

    applyControlConstraints(0, uk);
    applyControlConstraints(1, uk);

    data_nmpc_.trajectory_data.U[k - 1] = uk;
  }

  // Repeat the last control for the u[K-1]
  data_nmpc_.trajectory_data.U[K_mpc_steps - 1] = data_nmpc_.trajectory_data.U[K_mpc_steps - 2];

  return true;
}

void ns_nmpc_interface::NonlinearMPCController::setCurrent_s0(double const &s0)
{
  current_s0_ = s0;
}

void ns_nmpc_interface::NonlinearMPCController::setCurrent_t0(double const &t0)
{
  current_t0_ = t0;
}

void ns_nmpc_interface::NonlinearMPCController::setCurrent_s0_predicted(double const &s0_predicted)
{
  current_s0_predicted_ = s0_predicted;
}

void ns_nmpc_interface::NonlinearMPCController::setCurrentAvgMPCComputationTime(
  const double &avg_mpc_computation_time)
{
  current_avg_mpc_computation_time_ = avg_mpc_computation_time;
}

void ns_nmpc_interface::NonlinearMPCController::setLoggerName(std::string_view const &logger_name)
{
  node_logger_name_ = logger_name;
}

trajectory_data_t ns_nmpc_interface::NonlinearMPCController::getCurrentTrajectoryData() const
{
  return data_nmpc_.trajectory_data;
}

bool ns_nmpc_interface::NonlinearMPCController::solveNMPC_problem(
  ns_splines::InterpolatingSplinePCG const &piecewise_interpolator)
{
  // Discretisize.
  bool const &&is_discretisized = ns_discretization::multipleShootingTrajectory(model_ptr_,
                                                                                data_nmpc_.trajectory_data,
                                                                                data_nmpc_.target_reference_states_and_controls,
                                                                                piecewise_interpolator,
                                                                                data_nmpc_.mpc_prediction_dt,
                                                                                data_nmpc_.discretization_data);

  // Alternative methods for the discretizations.
  // bool is_discretisized = ns_discretization::bilinearTransformation(
  //   model_ptr_, data_nmpc_.trajectory_data, data_nmpc_.target_reference_states_and_controls,
  //   piecewise_interpolator, dt, data_nmpc_.discretization_data);

  // Update OSQP constraint Matrix Aequality parts only.
  bool const &&is_osqp_initialized = osqp_interface_.updateOSQP(data_nmpc_, params_opt_);

  if (!(is_discretisized && is_osqp_initialized))
  {
    RCLCPP_ERROR(rclcpp::get_logger(node_logger_name_),
                 "[nonlinear_mpc - solve_mpc] : Could not discretisize trajectories or "
                 "initialize the OSQP modules ...");
    return false;
  }

  // Call OSQP to solve.
  osqp::OsqpExitCode const &&exit_status = osqp_interface_.solve();

  if (exit_status == osqp::OsqpExitCode::kPrimalInfeasible)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node_logger_name_),
                 "[nonlinear_mpc - solve_mpc_problem] : OSQP solution is "
                 "not primal_feasible ...");
    return false;
  }

  if (exit_status != osqp::OsqpExitCode::kOptimal)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node_logger_name_),
                 "[nonlinear_mpc - solve_mpc_problem] : OSQP solution is not optimal ...");
    return false;
  }

  // Read solution for SQP iterations.
  readSolutionsFromOSQP();

  // DEBUG
  //  ns_utils::print("In the solve method : ");
  //  ns_utils::print(
  //    "Are system matrices discretisized      : ",
  //    is_discretisized ? " discretisized ..." : "not discretisized ...");
  //
  //  ns_utils::print("Initialized Discrete Matrices      : ");

  //  for (size_t k = 0; k < K_mpc_steps - 1; k++)
  //  {
  //  ns_utils::print("A[k]");
  //  ns_eigen_utils::printEigenMat(discretization_data_.A.at(k));
  //  ns_utils::print("B[k]");
  //  ns_eigen_utils::printEigenMat(discretization_data_.B.at(k));
  // ns_utils::print("z[k]");
  // ns_eigen_utils::printEigenMat(discretization_data_.z.at(k));
  // }

  //  ns_utils::print(
  //    "Is OSQP problem initialized?   : ",
  //    is_osqp_initialized ? " initialized ..." : "not initialized ...");
  //
  //  ns_utils::print("Is the updated OSQP problem solved?        : ", ToString(exit_status));

  // auto &&Xtemp = ns_eigen_utils::getTrajectory(data_nmpc_.trajectory_data.X);
  // auto &&Utemp = ns_eigen_utils::getTrajectory(data_nmpc_.trajectory_data.U);

  // ns_utils::print("\Solved trajectories : ");
  // ns_eigen_utils::printEigenMat(Xtemp.transpose());  //  [x, y, psi, s, ey, epsi, v, delta]

  // ns_utils::print("\nSolved Control trajectories U : ");
  // ns_eigen_utils::printEigenMat(Utemp.transpose());

  // ns_utils::print("State weights : ");
  // ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_opt_.Q));
  // end of debug

  return true;
}

[[maybe_unused]] void ns_nmpc_interface::NonlinearMPCController::computeSteeringFeedbackControls(
  ns_splines::InterpolatingSplinePCG const &piecewise_interpolator,
  double const &dt,
  Model::input_vector_t &u_solution)
{
  // Compute a steering feedback signal and store it in the trajectory_data.
  if (bool const &&is_feedback_computed = lpv_initializer_.computeSingleFeedbackControls(model_ptr_,
                                                                                         piecewise_interpolator,
                                                                                         params_lpv_,
                                                                                         params_opt_,
                                                                                         data_nmpc_,
                                                                                         dt);!is_feedback_computed)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node_logger_name_),
                 "[nonlinear_mpc_core] Could not compute the feedback signal ...");
    return;
  }

  // Release the feedback signal.
  data_nmpc_.trajectory_data.getFeedbackControls(u_solution);

  // Store the computed values.
  u_solution_last_ = u_solution;
}

void ns_nmpc_interface::NonlinearMPCController::readSolutionsFromOSQP()
{
  // Get the solution from OSQP and store them in the trajectory_data_.
  osqp_interface_.getSolution(params_opt_, data_nmpc_.trajectory_data);
}

void ns_nmpc_interface::NonlinearMPCController::shiftControls()
{
  /*   size_t const & nU = data_nmpc_.trajectory_data.nU();
  for (size_t k = 1; k < nU; k++) {
  data_nmpc_.trajectory_data.U[k - 1] = data_nmpc_.trajectory_data.U[k];
  } */

  for (auto &&it = data_nmpc_.trajectory_data.U.begin() + 1;
       it != data_nmpc_.trajectory_data.U.end(); ++it)
  {
    *std::prev(it) = *it;
  }

  // Repeat the K-1 at K.
  //data_nmpc_.trajectory_data.U[nU - 1] = data_nmpc_.trajectory_data.U[nU - 2];
  data_nmpc_.trajectory_data.U.rbegin()[0] = data_nmpc_.trajectory_data.U.rbegin()[0];
}

//void ns_nmpc_interface::NonlinearMPCController::shiftControls()
//{
//    size_t const &nU = data_nmpc_.trajectory_data.nU();
//    for (size_t k = 1; k < nU; k++)
//    {
//        data_nmpc_.trajectory_data.U[k - 1] = data_nmpc_.trajectory_data.U[k];
//    }
//
//    // Repeat the K-1 at K.
//    data_nmpc_.trajectory_data.U[nU - 1] = data_nmpc_.trajectory_data.U[nU - 2];
//}

void ns_nmpc_interface::NonlinearMPCController::getControlSolutions(
  Model::input_vector_t &u_solution)
{
  // Interpolates the OSQP solutions between U[0] and U[1].
  data_nmpc_.trajectory_data.getControlMPCSolutionsAtTime(0.0,  // params_ptr_->control_period,
                                                          data_nmpc_.mpc_prediction_dt,
                                                          u_solution);

  u_solution_last_ = u_solution;
}

void ns_nmpc_interface::NonlinearMPCController::getTimeSpeedVectsFromSmoothTraj(
  std::vector<std::vector<double>> &t_speed_vects) const
{
  t_speed_vects.emplace_back(current_MPCtraj_smooth_vects_ptr_->t);
  t_speed_vects.emplace_back(current_MPCtraj_smooth_vects_ptr_->vx);
}

double ns_nmpc_interface::NonlinearMPCController::getObjectiveValue() const
{
  return osqp_interface_.getObjectiveValue();
}

std::array<double, 3> ns_nmpc_interface::NonlinearMPCController::getSmooth_XYYawAtCurrentDistance()
const
{
  double smooth_x{};
  double smooth_y{};
  double smooth_yaw{};

  // Interpolate the smooth x coord.
  ns_utils::interp1d_linear(current_MPCtraj_smooth_vects_ptr_->s,
                            current_MPCtraj_smooth_vects_ptr_->x,
                            current_s0_,
                            smooth_x);

  // Interpolate the smooth y coord.
  ns_utils::interp1d_linear(current_MPCtraj_smooth_vects_ptr_->s,
                            current_MPCtraj_smooth_vects_ptr_->y,
                            current_s0_,
                            smooth_y);

  // Interpolate the smooth yaw angle.
  ns_utils::interp1d_linear(current_MPCtraj_smooth_vects_ptr_->s,
                            current_MPCtraj_smooth_vects_ptr_->yaw,
                            current_s0_,
                            smooth_yaw);

  return std::array<double, 3>{smooth_x, smooth_y, ns_utils::wrapToPi(smooth_yaw)};
}

void ns_nmpc_interface::NonlinearMPCController::getRawVxAtDistance(double const &s, double &vx) const
{
  ns_utils::interp1d_linear(current_MPCtraj_raw_vects_ptr_->s, current_MPCtraj_raw_vects_ptr_->vx, s, vx);
}

void ns_nmpc_interface::NonlinearMPCController::getSmoothVxAtDistance(double const &s, double &vx) const
{
  ns_utils::interp1d_linear(current_MPCtraj_smooth_vects_ptr_->s, current_MPCtraj_smooth_vects_ptr_->vx, s, vx);
}

void ns_nmpc_interface::NonlinearMPCController::getSpeedDynamics_vdot(const double &current_long_speed,
                                                                      const double &current_speed_input,
                                                                      double &vdot) const
{
  vdot = model_ptr_->getLongSpeedDynamics_vdot(current_long_speed, current_speed_input);
}

void ns_nmpc_interface::NonlinearMPCController::getSteeringDynamics_deltadot(const double &current_steering,
                                                                             const double &current_steering_input,
                                                                             double &delta_dot) const
{
  delta_dot = model_ptr_->getSteeringDynamics_deltadot(current_steering, current_steering_input);
}
