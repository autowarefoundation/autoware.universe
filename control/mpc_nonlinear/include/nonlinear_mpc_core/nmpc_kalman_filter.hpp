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

#ifndef NONLINEAR_MPC_CORE__NMPC_KALMAN_FILTER_HPP_
#define NONLINEAR_MPC_CORE__NMPC_KALMAN_FILTER_HPP_

#include <utility>

#include "active_model.hpp"
#include "nonlinear_mpc_core/data_and_parameter_container.hpp"
#include "nonlinear_mpc_core/nmpc_discretization.hpp"
#include "nonlinear_mpc_core/nmpc_simulation.hpp"
#include "utils_act/act_utils.hpp"
#include "common/types.hpp"
#include "helper_functions/angle_utils.hpp"

/**
 * @brief Linear Kalman filter applied to the incremental states x[k+1] - z[k] = Ad* \delta_x +  Bd*u[k]
 * */
namespace ns_filters
{

class KalmanUnscented
{
 public:
  using sigma_point_mat_t = Eigen::Matrix<double, Model::state_dim, ns_filters::UKF_NUM_OF_SIGMAS>;

  KalmanUnscented() = default;

  explicit KalmanUnscented(Model::model_ptr_t model, double const dt)
    : model_ptr_(std::move(model)), dt_{dt}
  {
    V_.setIdentity();
    W_.setIdentity();
    P_.setZero();
    Psqrt_.setZero();

    Ck_.setZero();
    Sk_.setZero();
    Kk_.setZero();

    sigmaPointsMat_x_.setZero();
    sigmaPointsMat_y_.setZero();
    sigmaPoints_fxfy_.setZero();

    x_est_mean_full_.setZero();
    y_est_mean_full_.setZero();
  }

  KalmanUnscented(KalmanUnscented const &other);

  KalmanUnscented &operator=(KalmanUnscented const &other);

  ~KalmanUnscented() = default;

  /**
   * @brief   implements the linear Kalman filter prediction and update steps.
   * @param previous_input u0 is the input signal sent to the vehicle.
   * @param current_curvature current curvature of the path,
   * @param x_measured full states including [x, y, ...]
   * */

  void getStateEstimate(Model::input_vector_t const &u0,
                        Model::param_vector_t const &params,
                        Model::state_vector_t const &x_measured,
                        Model::state_vector_t &xest);

  /**
   * @brief set covariance matrices, V, W, P.
   * @param Vsqrt process uncertainty variance (independent state assumption),
   * @param Wsqrt measurement uncertainty variance,
   * @param Psqrt updated variance .
   * */
  void updateParameters(ns_data::ParamsFilters const &params_filters);

  void Initialize_xest0(Model::state_vector_t const &xe0);

  [[nodiscard]] bool isInitialized() const
  { return is_initialized_; }

  void reset()
  {
    x_est_mean_full_.setZero();
    is_initialized_ = false;
  }

  void resetStateEstimate(size_t const &ind, double value)
  {
    x_est_mean_full_(ind) = value;
  }

 private:
  Model::model_ptr_t model_ptr_{nullptr};
  double dt_{};  // <-@brief model update time step.

  Model::state_matrix_t V_{Model::state_matrix_t::Identity()};  // !<-@brief process uncertainty covariance matrix.
  Model::state_matrix_t W_{Model::state_matrix_t::Identity()};  // !<-@brief measurement uncertainty covariance matrix.
  Model::state_matrix_t P_{Model::state_matrix_t::Zero()};  // !<-@brief initial and updated covariance.
  Model::state_matrix_t Psqrt_{Model::state_matrix_t::Zero()};  // !<-@brief square root of the covariance matrix P_.

  // !<-@brief Placeholders for measurement and cross covariances.
  Model::state_matrix_t Ck_{Model::state_matrix_t::Zero()};  // !<-@brief cross covariance
  Model::state_matrix_t Sk_{Model::state_matrix_t::Zero()};  // !<-@brief measurement covariance
  Model::state_matrix_t Kk_{Model::state_matrix_t::Zero()};  // !<-@brief current Kalman gain

  double alpha_{};
  double beta_{};
  double kappa_{};

  Eigen::Index xdim_{};                 // !<-@brief number of states,
  Eigen::Index num_of_sigma_points_{};  // !<-@brief number of sigma points.
  double lambda_{};  // !<-@brief scaling parameters used in the sigma point generation.
  double gamma_{};

  /**
   * @brief UKF mean and covariance weights.
   **/
  double Weight0_m_{};
  double Weight0_c_{};
  double Weight_common_{};

  // Placeholders
  sigma_point_mat_t sigmaPointsMat_x_{sigma_point_mat_t::Zero()};  // !<-@brief sigma points generated from the states.

  // !<-@brief sigma points generated from the measurements.
  sigma_point_mat_t sigmaPointsMat_y_{sigma_point_mat_t::Zero()};

  // !<-@brief sigma points placeholder for the propagated states and measurements.
  sigma_point_mat_t sigmaPoints_fxfy_{sigma_point_mat_t::Zero()};

  /**
   * @brief xest updated state estimates.
   * */

  // !<-@brief mean of full state for Jacobian computations.
  Model::state_vector_t x_est_mean_full_{Model::state_vector_t::Zero()};

  // !<-@brief mean of full state for the measurements.
  Model::state_vector_t y_est_mean_full_{Model::state_vector_t::Zero()};

  // !<-@brief whether the initial state estimate is initialized or not.
  bool is_initialized_{false};

  // Class methods.
  void computeSQRTofCovarianceMatrix();

  void KalmanUnscentedPredictionUpdateStep(const Model::input_vector_t &u0, Model::param_vector_t const &params);

  void KalmanUnscentedMeasurementUpdateStep(Model::state_vector_t const &x_measured);

  void computeWeightsAndCoeffs();

  void generateSigmaPoints(sigma_point_mat_t &sigma_points);

  /**
   * @brief propagates sigma points by integrating them one-step; xnext = \int '\dot{x}=f(x).
   **/
  void propagateSigmaPoints_fx(const Model::input_vector_t &u0, Model::param_vector_t const &params);
};

class KalmanUnscentedSQRT
{
 public:
  using sigma_point_mat_t = Eigen::Matrix<double, Model::state_dim, ns_filters::UKF_NUM_OF_SIGMAS>;

  using sigma_concat_mat_t = Eigen::Matrix<double, Model::state_dim,
                                           ns_filters::UKF_NUM_OF_SIGMAS + Model::state_dim - 1>;

  using sigma_concat_mat_t_T = Eigen::Matrix<double,
                                             ns_filters::UKF_NUM_OF_SIGMAS + Model::state_dim - 1,
                                             Model::state_dim>;

  KalmanUnscentedSQRT() = default;

  explicit KalmanUnscentedSQRT(Model::model_ptr_t model, double const dt);

  KalmanUnscentedSQRT(KalmanUnscentedSQRT const &other);

  KalmanUnscentedSQRT &operator=(KalmanUnscentedSQRT const &other);

  ~KalmanUnscentedSQRT() = default;

  /**
   * @brief   implements the linear Kalman filter prediction and update steps.
   * @param previous_input u0 is the input signal sent to the vehicle.
   * @param current_curvature current curvature of the path,
   * @param x_measured full states including [x, y, ...]
   * */
  void getStateEstimate(Model::input_vector_t const &u0, Model::param_vector_t const &params,
                        Model::state_vector_t const &x_measured, Model::state_vector_t &xest);

  /**
   * @brief set covariance matrices, V, W, S.
   * @param Vsqrt process uncertainty variance (independent state assumption),
   * @param Wsqrt measurement uncertainty variance,
   * @param Ssqrt updated variances .
   * */
  void updateParameters(ns_data::ParamsFilters const &params_filters);

  void Initialize_xest0(Model::state_vector_t const &xe0);

  [[nodiscard]] bool isInitialized() const
  { return is_initialized_; }

  void reset()
  {
    x_est_mean_full_.setZero();
    is_initialized_ = false;
  }

  void resetStateEstimate(size_t const &ind, double value)
  {
    x_est_mean_full_(ind) = value;
  }

 private:
  Model::model_ptr_t model_ptr_{nullptr};
  double dt_{};  // <-@brief model update time step.

  // !<-@brief process uncertainty sqrt covariance matrix.
  Model::state_matrix_t Vsqrt_{Model::state_matrix_t::Identity()};

  // !<-@brief measurement uncertainty sqrt covariance matrix.
  Model::state_matrix_t Wsqrt_{Model::state_matrix_t::Identity()};

  // Frequently re-assigned.
  // !<-@brief square root of the covariance matrix P_.
  Model::state_matrix_t Sx_sqrt_{Model::state_matrix_t::Zero()};

  // !<-@brief measurement covariance square root
  Model::state_matrix_t Sy_sqrt_{Model::state_matrix_t::Zero()};

  // !<-@brief Placeholders for measurement and cross covariances.
  Model::state_matrix_t Ck_{Model::state_matrix_t::Zero()};  // !<-@brief cross covariance

  Model::state_matrix_t Kk_{Model::state_matrix_t::Zero()};  // !<-@brief current Kalman gain

  double alpha_{};
  double beta_{};
  double kappa_{};

  Eigen::Index xdim_{};                 // !<-@brief number of states,
  Eigen::Index num_of_sigma_points_{};  // !<-@brief number of sigma points.
  double lambda_{};  // !<-@brief scaling parameters used in the sigma point generation.
  double gamma_{};

  /**
  * @brief UKF mean and covariance weights.
  **/
  double Weight0_m_{};
  double Weight0_c_{};
  double Weight_common_{};

  // Placeholders
  // !<-@brief sigma points generated from the states.
  sigma_point_mat_t sigmaPointsMat_x_{sigma_point_mat_t::Zero()};

  // !<-@brief sigma points generated from the measurements.
  sigma_point_mat_t sigmaPointsMat_y_{sigma_point_mat_t::Zero()};

  // !<-@brief sigma points placeholder for the propagated states and measurements.
  sigma_point_mat_t sigmaPoints_fxfy_{sigma_point_mat_t::Zero()};

  // !<-@brief place holder for concatenated matrices SigmaV = [Sigma, V]
  sigma_concat_mat_t sigma_and_Vsqrt{sigma_concat_mat_t::Zero()};
  sigma_concat_mat_t sigma_and_Wsqrt{sigma_concat_mat_t::Zero()};

  /**
   * @brief xest updated state estimates.
   * */

  // !<-@brief mean of full state for Jacobian computations.
  Model::state_vector_t x_est_mean_full_{Model::state_vector_t::Zero()};
  Model::state_vector_t y_est_mean_full_{
    Model::state_vector_t::Zero()};  // !<-@brief mean of full state for the measurements.

  // !<-@brief whether the initial state estimate is initialized or not.
  bool is_initialized_{false};

  // Class methods.

  void KalmanUnscentedPredictionUpdateStep(const Model::input_vector_t &u0, Model::param_vector_t const &params);

  void KalmanUnscentedMeasurementUpdateStep(Model::state_vector_t const &x_measured);

  void computeWeightsAndCoeffs();

  void generateSigmaPoints(sigma_point_mat_t &sigma_points);

  /**
   * @brief propagates sigma points by integrating them one-step; xnext = \int '\dot{x}=f(x).
   **/
  void propagateSigmaPoints_fx(const Model::input_vector_t &u0, Model::param_vector_t const &params);

  template<class Derived>
  void choleskyOneRankUpdate(Model::state_matrix_t &R, Eigen::MatrixBase<Derived> const &U, double const &weight);

  void choleskyUpdateFromSigmaPoints(Model::state_matrix_t &matrix_sqrt_tobe_updated,
                                     sigma_point_mat_t const &sigma_points,
                                     sigma_concat_mat_t &sigma_concatenated,
                                     double const &weight);
};

}  // namespace ns_filters
#endif  // NONLINEAR_MPC_CORE__NMPC_KALMAN_FILTER_HPP_
