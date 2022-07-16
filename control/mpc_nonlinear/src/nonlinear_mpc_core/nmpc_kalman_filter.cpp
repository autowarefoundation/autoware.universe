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

#include "nonlinear_mpc_core/nmpc_kalman_filter.hpp"

#include <utility>

namespace ns_filters
{

// Unscented Kalman filter methods.
KalmanUKFbase::KalmanUKFbase(Model::model_ptr_t model, const double dt) : model_ptr_(std::move(model)), dt_{dt}
{

}

KalmanUnscented::KalmanUnscented(const Model::model_ptr_t &model, double const dt)
  : KalmanUKFbase(model, dt)
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

KalmanUnscented::KalmanUnscented(KalmanUnscented const &other) : KalmanUKFbase(other),
                                                                 V_{other.V_},
                                                                 W_{other.W_},
                                                                 P_{other.P_},
                                                                 alpha_{other.alpha_},
                                                                 beta_{other.beta_},
                                                                 kappa_{other.kappa_}
{
  Psqrt_.setZero();
  Sk_.setZero();
  Ck_.setZero();
  computeWeightsAndCoeffs();
}

KalmanUnscented::KalmanUnscented(KalmanUnscented &&other) noexcept: KalmanUKFbase(other),
                                                                    V_{std::move(other.V_)},
                                                                    W_{std::move(other.W_)},
                                                                    P_{std::move(other.P_)},
                                                                    alpha_{other.alpha_},
                                                                    beta_{other.beta_},
                                                                    kappa_{other.kappa_}
{
  Psqrt_.setZero();
  Sk_.setZero();
  Ck_.setZero();

  computeWeightsAndCoeffs();

}

KalmanUnscented &KalmanUnscented::operator=(KalmanUnscented const &other)
{
  if (this != &other)
  {
    KalmanUKFbase::model_ptr_ = other.model_ptr_;
    KalmanUKFbase::dt_ = other.dt_;

    V_ = other.V_;
    W_ = other.W_;
    P_ = other.P_;

    Psqrt_.setZero();
    Sk_.setZero();
    Ck_.setZero();

    alpha_ = other.alpha_;
    beta_ = other.beta_;
    kappa_ = other.kappa_;

    computeWeightsAndCoeffs();
  }

  return *this;
}
KalmanUnscented &KalmanUnscented::operator=(KalmanUnscented &&other) noexcept
{
  if (this != &other)
  {
    model_ptr_ = other.model_ptr_;
    dt_ = other.dt_;

    V_ = std::move(other.V_);
    W_ = std::move(other.W_);
    P_ = std::move(other.P_);

    Psqrt_.setZero();
    Sk_.setZero();
    Ck_.setZero();

    alpha_ = other.alpha_;
    beta_ = other.beta_;
    kappa_ = other.kappa_;

    computeWeightsAndCoeffs();
  }

  return *this;
}

void KalmanUnscented::computeWeightsAndCoeffs()
{
  xdim_ = V_.rows();
  num_of_sigma_points_ = 2 * xdim_ + 1;
  lambda_ = alpha_ * alpha_ * (static_cast<double>(xdim_) + kappa_) - static_cast<double>(xdim_);
  gamma_ = std::sqrt(static_cast<double>(xdim_) + lambda_);

  /**
   * @brief compute the weights.
   *
   */

  Weight0_m_ = lambda_ / (static_cast<double>(xdim_) + lambda_);
  Weight0_c_ = lambda_ / (static_cast<double>(xdim_) + lambda_) + (1.0 - alpha_ * alpha_ + beta_);
  Weight_common_ = 1. / (2.0 * (static_cast<double>(xdim_) + lambda_));
}

void KalmanUnscented::updateParameters(ns_data::ParamsFilters const &params_filters)
{
  Model::state_matrix_t Vtemp(params_filters.Vsqrt);
  V_ = Vtemp * Vtemp;

  Model::state_matrix_t Wtep(params_filters.Wsqrt);
  W_ = Wtep * Wtep;

  Model::state_matrix_t Ptemp(params_filters.Psqrt);
  P_ = Ptemp * Ptemp;

  alpha_ = params_filters.ukf_alpha;
  beta_ = params_filters.ukf_beta;
  kappa_ = params_filters.ukf_kappa;

  computeWeightsAndCoeffs();

  /*   ns_utils::print("Unscented Kalman filter parameters V, W, P in Kalman filter : ");
      ns_eigen_utils::printEigenMat(V_);
      ns_eigen_utils::printEigenMat(W_);
      ns_eigen_utils::printEigenMat(P_);

      ns_utils::print("Uncented Kalman filter, alpha, beta, kappa", alpha_, beta_, kappa_); */
}

void KalmanUnscented::computeSQRTofCovarianceMatrix()
{
  Psqrt_ = Model::state_matrix_t(P_.llt().matrixL());

  // DEBUG
  // ns_utils::print("UKF Psqrt ");
  // end of debug
}

void KalmanUnscented::propagateSigmaPoints_fx(const Model::input_vector_t &u0, Model::param_vector_t const &params)
{
  sigmaPoints_fxfy_.setZero();

  for (Eigen::Index k = 0; k < num_of_sigma_points_; ++k)
  {
    auto xk = Model::state_vector_t(sigmaPointsMat_x_.col(k));
    ns_sim::simulateNonlinearModel_zoh(model_ptr_, u0, params, dt_, xk);

    sigmaPoints_fxfy_.col(k) = xk;
  }

  // DEBUG
  //    ns_utils::print("Integrated Sigma Points : ");
  //    ns_eigen_utils::printEigenMat(sigmaPoints_fxfy_);
  // end of debug
}

void KalmanUnscented::generateSigmaPoints(sigma_point_mat_t &sigma_points)
{
  sigma_points.setZero();
  sigma_points.col(0) = x_est_mean_full_;

  computeSQRTofCovarianceMatrix();

  for (Eigen::Index k = 1; k < xdim_ + 1; ++k)
  {
    sigma_points.col(k) = x_est_mean_full_ + gamma_ * Psqrt_.col(k - 1);
    sigma_points.col(k + xdim_) = x_est_mean_full_ - gamma_ * Psqrt_.col(k - 1);
  }

  // DEBUG
  //    ns_utils::print("\nComputed Sigma Points : ");
  //    ns_eigen_utils::printEigenMat(sigma_points);
  // end of debug
}

void KalmanUnscented::getStateEstimate(Model::input_vector_t const &u0,
                                       Model::param_vector_t const &params,
                                       Model::state_vector_t const &x_measured,
                                       Model::state_vector_t &xest)
{
  KalmanUnscentedPredictionUpdateStep(u0, params);
  KalmanUnscentedMeasurementUpdateStep(x_measured);

  xest = x_est_mean_full_;
}

void KalmanUnscented::KalmanUnscentedPredictionUpdateStep(const Model::input_vector_t &u0,
                                                          Model::param_vector_t const &params)
{
  generateSigmaPoints(sigmaPointsMat_x_);  // <-@brief generate sigma points of UKF
  propagateSigmaPoints_fx(u0, params);     // <-@brief propagate sigma points X =\int f(x)

  /**
   * @brief compute mean and covariance updating priors for the measurements.
   * mean_sigma = \sum W*f(sigma_points)
   * cov_sigma = \sum (f(sigma_points) - mean_sigma) * (f(sigma_points) - mean_sigma)'
   **/

  auto const &&first_column = Weight0_m_ * sigmaPoints_fxfy_.col(0);
  auto const &&right_columns = Weight_common_ * sigmaPoints_fxfy_.rightCols(num_of_sigma_points_ - 1);

  /**
   * @brief Update the prior of mean with the prediction update.
   *
   **/

  x_est_mean_full_ = first_column + right_columns.rowwise().sum();

  // <-@brief Update Covariance using the mean prior computed.
  auto const &&d_sigma_points = sigmaPoints_fxfy_.col(0) - x_est_mean_full_;
  Model::state_matrix_t Px0 = Weight0_c_ * d_sigma_points * d_sigma_points.transpose();

  for (Eigen::Index k = 1; k < num_of_sigma_points_; ++k)
  {
    auto const &&temp_dsigma = sigmaPoints_fxfy_.col(k) - x_est_mean_full_;
    Px0.noalias() = Px0 + Weight_common_ * temp_dsigma * temp_dsigma.transpose();
  }

  P_ = Px0 + V_;

  // DEBUG
  // ns_utils::print("First column operation : ");
  //   ns_eigen_utils::printEigenMat(first_column);

  //   ns_utils::print("Right columns operation : ");
  //   ns_eigen_utils::printEigenMat(right_columns);

  //   ns_utils::print("Rowise sum of x mean : ");
  //   ns_eigen_utils::printEigenMat(x_est_mean_full_);

  //   ns_utils::print("Covariance update Px0 : ");
  //   ns_eigen_utils::printEigenMat(Px0);

  //   ns_utils::print("Updated covariance in prediction update P_: ");
  //   ns_eigen_utils::printEigenMat(P_);

  //   ns_utils::print("W0m, W0c, W : ", Weight0_m_, Weight0_c_, Weight_common_);
  // end of debug
}

void KalmanUnscented::KalmanUnscentedMeasurementUpdateStep(const Model::state_vector_t &x_measured)
{
  generateSigmaPoints(sigmaPointsMat_y_);

  /**
   * @brief In the measurement update, if an observation function is used as
   * y=h(x), we need to propagate these sigma points using h(x). In this
   * implementation h(x) is an identity function, therefore the propagation step
   * is not used.
   **/

  /**
   * @brief compute mean and covariance updating priors for the measurements.
   * mean_sigma = \sum W*f(sigma_points)
   * cov_sigma = \sum (f(sigma_points) - mean_sigma) * (f(sigma_points) - mean_sigma)'
    **/

  auto &&first_column = Weight0_m_ * sigmaPointsMat_y_.col(0);
  auto &&right_columns = Weight_common_ * sigmaPointsMat_y_.rightCols(num_of_sigma_points_ - 1);

  /**
   * @brief Update the prior of mean with the prediction update.
   *
   **/

  y_est_mean_full_ = first_column + right_columns.rowwise().sum();

  // <-@brief Update Covariance using the mean prior computed.
  auto &&d_sigma_points_y = sigmaPointsMat_y_.col(0) - y_est_mean_full_;
  Model::state_matrix_t Stemp = Weight0_c_ * d_sigma_points_y * d_sigma_points_y.transpose();

  auto &&d_sigma_points_x = sigmaPointsMat_x_.col(0) - x_est_mean_full_;
  Model::state_matrix_t Ctemp = Weight0_c_ * d_sigma_points_x * d_sigma_points_y.transpose();

  for (Eigen::Index k = 1; k < num_of_sigma_points_; ++k)
  {
    auto &&temp_dsigma_y = sigmaPointsMat_y_.col(k) - y_est_mean_full_;
    Stemp.noalias() = Stemp + Weight_common_ * temp_dsigma_y * temp_dsigma_y.transpose();

    auto &&temp_dsigma_x = sigmaPointsMat_x_.col(k) - x_est_mean_full_;
    Ctemp.noalias() = Ctemp + Weight_common_ * temp_dsigma_x * temp_dsigma_y.transpose();
  }

  Ck_ = Ctemp;
  Sk_ = Stemp + W_;
  Kk_ = Ck_ * Sk_.inverse();

  // Update mean and covariance after the measurement update.
  P_ = P_ - Kk_ * Sk_ * Kk_.transpose();

  // Normalize the angle variables
  Model::state_vector_t innovation_error = (x_measured - y_est_mean_full_);

  /**
   * @brief Here the KF takes two angle differences which might go out the bounds. Angle wrapping might be remedy.
   * */

  innovation_error(2) = ns_utils::angleDistance(innovation_error(2)); // yaw error diffs.
  innovation_error(5) = ns_utils::angleDistance(innovation_error(5));  // yaw_error

  x_est_mean_full_ = x_est_mean_full_ + Kk_ * innovation_error;

  x_est_mean_full_(2) = ns_utils::angleDistance(x_est_mean_full_(2));
  x_est_mean_full_(5) = ns_utils::angleDistance(x_est_mean_full_(5));

  // Normalize after the update.
  x_est_mean_full_(2) =
    autoware::common::helper_functions::wrap_angle(x_est_mean_full_(2));  // normalize integrated e_yaw

  x_est_mean_full_(5) =
    autoware::common::helper_functions::wrap_angle(x_est_mean_full_(5));  // normalize integrated e_yaw

  // DEBUG
  /*   ns_utils::print("First column operation : ");
      ns_eigen_utils::printEigenMat(first_column);

      ns_utils::print("Right columns operation : ");
      ns_eigen_utils::printEigenMat(right_columns);

      ns_utils::print("Rowise sum of y mean : ");
      ns_eigen_utils::printEigenMat(y_est_mean_full_);

      ns_utils::print("Covariance update Sk : ");
      ns_eigen_utils::printEigenMat(Sk_);

      ns_utils::print("Updated covariance in prediction update P_: ");
      ns_eigen_utils::printEigenMat(P_);

      ns_utils::print("W0m, W0c, W : ", Weight0_m_, Weight0_c_, Weight_common_); */
  // end of debug
}

void KalmanUnscented::Initialize_xest0(Model::state_vector_t const &x0)
{
  x_est_mean_full_ = x0;
  is_initialized_ = true;
}

/////////////////////// UNSCENTED KALMAN SQRT VERSION ///////////////////////////
// ---------- SQRT version of Unscented Kalman Filter ----------------------

void KalmanUnscentedSQRT::computeWeightsAndCoeffs()
{
  xdim_ = Vsqrt_.rows();
  num_of_sigma_points_ = 2 * xdim_ + 1;
  lambda_ = alpha_ * alpha_ * (static_cast<double>(xdim_) + kappa_) - static_cast<double>(xdim_);
  gamma_ = std::sqrt(static_cast<double>(xdim_) + lambda_);

  /**
  * @brief compute the weights.
  *
  */

  Weight0_m_ = lambda_ / (static_cast<double>(xdim_) + lambda_);
  Weight0_c_ = lambda_ / (static_cast<double>(xdim_) + lambda_) + (1.0 - alpha_ * alpha_ + beta_);
  Weight_common_ = 1. / (2.0 * (static_cast<double>(xdim_) + lambda_));
}

KalmanUnscentedSQRT::KalmanUnscentedSQRT(Model::model_ptr_t model, const double dt)
  : KalmanUKFbase(std::move(model), dt)
{
  Vsqrt_.setIdentity();
  Wsqrt_.setIdentity();
  Sx_sqrt_.setZero();

  Ck_.setZero();
  Sy_sqrt_.setZero();
  Kk_.setZero();

  sigmaPointsMat_x_.setZero();
  sigmaPointsMat_y_.setZero();
  sigmaPoints_fxfy_.setZero();

  x_est_mean_full_.setZero();
  y_est_mean_full_.setZero();
}

KalmanUnscentedSQRT::KalmanUnscentedSQRT(KalmanUnscentedSQRT const &other)
  : KalmanUKFbase(other),
    Vsqrt_{other.Vsqrt_},
    Wsqrt_{other.Wsqrt_},
    Sx_sqrt_{other.Sx_sqrt_},
    alpha_{other.alpha_},
    beta_{other.beta_},
    kappa_{other.kappa_}
{
  // Initialize concatenated matrices
  sigma_and_Vsqrt.setZero();
  sigma_and_Vsqrt.rightCols(Vsqrt_.cols()) = Vsqrt_;

  sigma_and_Wsqrt.setZero();
  sigma_and_Wsqrt.rightCols(Wsqrt_.cols()) = Wsqrt_;

  Sy_sqrt_.setZero();
  Ck_.setZero();
  computeWeightsAndCoeffs();
}

KalmanUnscentedSQRT &KalmanUnscentedSQRT::operator=(KalmanUnscentedSQRT const &other)
{
  if (this != &other)
  {
    Vsqrt_ = other.Vsqrt_;
    Wsqrt_ = other.Wsqrt_;

    // Initialize concatenated matrices
    sigma_and_Vsqrt.setZero();
    sigma_and_Vsqrt.rightCols(Vsqrt_.cols()) = Vsqrt_;

    sigma_and_Wsqrt.setZero();
    sigma_and_Wsqrt.rightCols(Wsqrt_.cols()) = Wsqrt_;

    Sx_sqrt_ = other.Sx_sqrt_;
    Sy_sqrt_.setZero();
    Ck_.setZero();

    alpha_ = other.alpha_;
    beta_ = other.beta_;
    kappa_ = other.kappa_;

    computeWeightsAndCoeffs();
  }

  return *this;
}

void KalmanUnscentedSQRT::updateParameters(ns_data::ParamsFilters const &params_filters)
{
  Vsqrt_ = Model::state_matrix_t(params_filters.Vsqrt);  // Sqrt of covariance matrices
  Wsqrt_ = Model::state_matrix_t(params_filters.Wsqrt);
  Sx_sqrt_ = Model::state_matrix_t(params_filters.Psqrt);

  // Initialize concatenated matrices
  sigma_and_Vsqrt.setZero();
  sigma_and_Vsqrt.rightCols(Vsqrt_.cols()) = Vsqrt_;

  sigma_and_Wsqrt.setZero();
  sigma_and_Wsqrt.rightCols(Wsqrt_.cols()) = Wsqrt_;

  alpha_ = params_filters.ukf_alpha;
  beta_ = params_filters.ukf_beta;
  kappa_ = params_filters.ukf_kappa;

  computeWeightsAndCoeffs();

  /*  ns_utils::print("Unscented Kalman filter parameters V, W, P in Kalman filter : ");
      ns_eigen_utils::printEigenMat(V_);
      ns_eigen_utils::printEigenMat(W_);
      ns_eigen_utils::printEigenMat(P_);

      ns_utils::print("Uncented Kalman filter, alpha, beta, kappa", alpha_, beta_, kappa_); */
}

void KalmanUnscentedSQRT::propagateSigmaPoints_fx(const Model::input_vector_t &u0, Model::param_vector_t const &params)
{
  sigmaPoints_fxfy_.setZero();

  for (Eigen::Index k = 0; k < num_of_sigma_points_; ++k)
  {
    auto xk = Model::state_vector_t(sigmaPointsMat_x_.col(k));
    ns_sim::simulateNonlinearModel_zoh(model_ptr_, u0, params, dt_, xk);

    sigmaPoints_fxfy_.col(k) = xk;
  }

  // DEBUG
  //    ns_utils::print("Integrated Sigma Points : ");
  //    ns_eigen_utils::printEigenMat(sigmaPoints_fxfy_);
  // end of debug
}

void KalmanUnscentedSQRT::generateSigmaPoints(sigma_point_mat_t &sigma_points)
{
  sigma_points.setZero();
  sigma_points.col(0) = x_est_mean_full_;

  for (Eigen::Index k = 1; k < xdim_ + 1; ++k)
  {
    sigma_points.col(k) = x_est_mean_full_ + gamma_ * Sx_sqrt_.col(k - 1);
    sigma_points.col(k + xdim_) = x_est_mean_full_ - gamma_ * Sx_sqrt_.col(k - 1);
  }

  // DEBUG
  //    ns_utils::print("\nComputed Sigma Points : ");
  //    ns_eigen_utils::printEigenMat(sigma_points);
  // end of debug
}

void KalmanUnscentedSQRT::getStateEstimate(Model::input_vector_t const &u0,
                                           Model::param_vector_t const &params,
                                           Model::state_vector_t const &x_measured,
                                           Model::state_vector_t &xest)
{
  KalmanUnscentedPredictionUpdateStep(u0, params);
  KalmanUnscentedMeasurementUpdateStep(x_measured);

  xest = x_est_mean_full_;
}

template<class Derived>
void KalmanUnscentedSQRT::choleskyOneRankUpdate(Model::state_matrix_t &R,
                                                Eigen::MatrixBase<Derived> const &U,
                                                double const &weight)
{
  //    double rowsum{};
  //    Model::state_matrix_t R(Rsqrt);

  //    std::cout << "Printing input R ... in cholesky update ";
  //    ns_eigen_utils::printEigenMat(R);
  //
  //    std::cout << "Printing input U ... in cholesky update ";
  //    ns_eigen_utils::printEigenMat(U);

  //    // Check if R is upper triangular.
  //    for (auto i = 0; i < R.rows(); ++i)
  //    {
  //        rowsum += R(i, 0);
  //    }
  //
  //    if (std::fabs(rowsum - R(0, 0)) > std::numeric_limits<double>::epsilon())
  //    {
  //        std::cout << " Matrix is not Upper Triangular " << "\n";
  //        return Model::state_matrix_t();
  //    }

  auto const &&sign = ns_utils::sgn(weight);
  auto const &&weight_sqrt = std::sqrt(std::fabs(weight));
  auto const &&nrows = U.rows();

  for (auto j = 0; j < U.cols(); ++j)
  {
    Eigen::MatrixXd x(weight_sqrt * U.col(j));

    for (auto k = 0; k < nrows; ++k)
    {
      auto const &&r_squared = R(k, k) * R(k, k) + sign * x(k, 0) * x(k, 0);

      auto const &&r = r_squared < 0 ? std::numeric_limits<double>::max() : std::sqrt(r_squared);
      // ns_utils::print("  rsquared ", r_squared);

      auto const &&c = r / R(k, k);
      auto const &&s = x(k, 0) / R(k, k);
      R(k, k) = r;

      auto &&rhs_R = (R.row(k).rightCols(nrows - 1 - k) + sign * s * x.bottomRows(nrows - 1 - k).transpose()) / c;

      R.row(k).rightCols(nrows - 1 - k) = rhs_R.eval();

      auto &&rhs_x = c * x.bottomRows(nrows - k - 1) - s * R.row(k).rightCols(nrows - 1 - k).transpose();
      x.bottomRows(nrows - k - 1).noalias() = rhs_x.eval();
    }
  }

  //    std::cout << "Printing output R ... in cholesky update ";
  //    ns_eigen_utils::printEigenMat(R.eval());
  //
  //    std::cout << "Printing output Rsqrt ... in cholesky update ";
  //    ns_eigen_utils::printEigenMat(Rsqrt.eval());
}

void KalmanUnscentedSQRT::choleskyUpdateFromSigmaPoints(Model::state_matrix_t &matrix_sqrt_tobe_updated,
                                                        sigma_point_mat_t const &sigma_points,
                                                        sigma_concat_mat_t &sigma_concatenated,
                                                        double const &weight)
{
  // Get the first colum of the sigma points
  auto first_column = sigma_points.col(0);
  auto const &&d_sigma_points_first_col_excluded = sigma_points.rightCols(num_of_sigma_points_ - 1);
  sigma_concatenated.leftCols(num_of_sigma_points_ - 1) = std::sqrt(Weight_common_) * d_sigma_points_first_col_excluded;

  //    ns_utils::print("Concatenated Sigma Points : ");
  //    ns_eigen_utils::printEigenMat(sigma_concatenated);

  auto qr = sigma_concatenated.transpose().householderQr();

  matrix_sqrt_tobe_updated =
    Model::state_matrix_t(Eigen::MatrixXd(qr.matrixQR().triangularView<Eigen::Upper>()).topRows(Model::state_dim));

  //    ns_utils::print("R of QR decomposition ");
  //    ns_eigen_utils::printEigenMat(R);
  //    // ns_utils::print("size of R ", R.rows(), R.cols());

  // ns_utils::print("Calling Cholesky update: ");
  choleskyOneRankUpdate(matrix_sqrt_tobe_updated, first_column, weight);

  //    ns_utils::print("Cholesky update returned : ");
  //    ns_eigen_utils::printEigenMat(R);

  // DEBUG
  //    ns_utils::print(" Updated Square root  : ");
  //    ns_eigen_utils::printEigenMat(matrix_sqrt_tobe_updated);
  // End of DEBUG
}

void KalmanUnscentedSQRT::KalmanUnscentedPredictionUpdateStep(const Model::input_vector_t &u0,
                                                              Model::param_vector_t const &params)
{
  generateSigmaPoints(sigmaPointsMat_x_);  // <-@brief generate sigma points of UKF
  propagateSigmaPoints_fx(u0, params);     // <-@brief propagate sigma points X =\int f(x)

  /**
  * @brief compute mean and covariance updating priors for the measurements.
  * mean_sigma = \sum W*f(sigma_points)
  * cov_sigma = \sum (f(sigma_points) - mean_sigma) * (f(sigma_points) - mean_sigma)'
  **/

  auto &&first_column = Weight0_m_ * sigmaPoints_fxfy_.col(0);
  auto &&right_columns = Weight_common_ * sigmaPoints_fxfy_.rightCols(num_of_sigma_points_ - 1);

  /**
  * @brief Update the prior of mean with the prediction update.
  **/
  x_est_mean_full_ = first_column + right_columns.rowwise().sum();

  // <-@brief Update Covariance using the mean prior computed.
  sigma_point_mat_t &&d_sigma_points_x = sigmaPoints_fxfy_.colwise() - x_est_mean_full_;

  // SQUQRE ROOT IMPLEMENTATION
  //    ns_utils::print("In prediction update : ");
  //    ns_utils::print("Sx_sqrt before update: ");
  //    ns_eigen_utils::printEigenMat(Sx_sqrt_);

  // Update the sqrt of state covariance
  choleskyUpdateFromSigmaPoints(Sx_sqrt_, d_sigma_points_x, sigma_and_Vsqrt, Weight0_c_);

  //    ns_utils::print("Sx_sqrt after update: ");
  //    ns_eigen_utils::printEigenMat(Sx_sqrt_);

  // DEBUG
  /*   ns_utils::print("First column operation : ");
      ns_eigen_utils::printEigenMat(first_column);

      ns_utils::print("Right columns operation : ");
      ns_eigen_utils::printEigenMat(right_columns);

      ns_utils::print("Rowise sum of x mean : ");
      ns_eigen_utils::printEigenMat(x_est_mean_full_);

      ns_utils::print("Covariance update Px0 : ");
      ns_eigen_utils::printEigenMat(Px0);

      ns_utils::print("Updated covariance in prediction update P_: ");
      ns_eigen_utils::printEigenMat(P_);

      ns_utils::print("W0m, W0c, W : ", Weight0_m_, Weight0_c_, Weight_common_); */
  // end of debug
}

void KalmanUnscentedSQRT::KalmanUnscentedMeasurementUpdateStep(const Model::state_vector_t &x_measured)
{
  generateSigmaPoints(sigmaPointsMat_y_);

  /**
   * @brief In the measurement update, if an observation function is used as
   * y=h(x), we need to propagate these sigma points using h(x). In this
   * implementation h(x) is an identity function, therefore the propagation step
   * is not used.
   **/

  /**
   * @brief compute mean and covariance updating priors for the measurements.
   * mean_sigma = \sum W*f(sigma_points)
   * cov_sigma = \sum (f(sigma_points) - mean_sigma) * (f(sigma_points) - mean_sigma)'
   **/

  auto &&first_column = Weight0_m_ * sigmaPointsMat_y_.col(0);
  auto &&right_columns = Weight_common_ * sigmaPointsMat_y_.rightCols(num_of_sigma_points_ - 1);

  /**
   * @brief Update the prior of mean with the prediction update.
   *
   **/

  y_est_mean_full_ = first_column + right_columns.rowwise().sum();

  // <-@brief Update Covariance using the mean prior computed.
  sigma_point_mat_t &&d_sigma_points_y = sigmaPointsMat_y_.colwise() - y_est_mean_full_;

  // SQUQRE ROOT IMPLEMENTATION
  //    ns_utils::print("In measurement update : ");
  //    ns_utils::print("Sy_sqrt before update: ");
  //    ns_eigen_utils::printEigenMat(Sy_sqrt_);

  choleskyUpdateFromSigmaPoints(Sy_sqrt_, d_sigma_points_y, sigma_and_Wsqrt, Weight0_c_);

  //    ns_utils::print("Sy_sqrt_ after update: ");
  //    ns_eigen_utils::printEigenMat(Sy_sqrt_);
  Ck_.setZero();
  auto &&d_sigma_points_x = sigmaPointsMat_x_.colwise() - x_est_mean_full_;

  Ck_ = Weight0_c_ * d_sigma_points_x.col(0) * d_sigma_points_y.col(0).transpose();

  for (Eigen::Index k = 1; k < num_of_sigma_points_; ++k)
  {
    auto &&temp_dsigma_x = d_sigma_points_x.col(k);
    auto &&temp_dsigma_y = d_sigma_points_y.col(k);

    Ck_.noalias() = Ck_ + Weight_common_ * temp_dsigma_x * temp_dsigma_y.transpose();
  }

  Eigen::MatrixXd KkS(Sy_sqrt_.rows(), Sy_sqrt_.cols());
  // Eigen::MatrixXd Kk(Sx_sqrt_.rows(), Sx_sqrt_.cols());

  //    ns_utils::print("Ck : ");
  //    ns_eigen_utils::printEigenMat(Ck_);

  /**
   *  Solves RX=B by back-substitution.
   * */
  ns_eigen_utils::backSubstitution(Eigen::MatrixXd(Sy_sqrt_.transpose()), Ck_, KkS, 'l');

  Kk_.setZero();
  ns_eigen_utils::backSubstitution(Eigen::MatrixXd(Sy_sqrt_), KkS, Kk_, 'u');

  //    ns_utils::print("KkS : ");
  //    ns_eigen_utils::printEigenMat(KkS);
  //
  //    ns_utils::print("Kk_ : ");
  //    ns_eigen_utils::printEigenMat(Kk_);

  // Update Sqrt_ once more
  Eigen::MatrixXd U(Kk_ * Sy_sqrt_);

  //    ns_utils::print("U : ");
  //    ns_eigen_utils::printEigenMat(U);

  //    ns_utils::print(" In measurement update before updating  Sx_sqrt_ : ");
  //    ns_eigen_utils::printEigenMat(Sx_sqrt_);

  double const weight = -1.;
  //    Model::state_matrix_t Sxtemp(Sx_sqrt_);
  //
  //    ns_utils::print(" In measurement update before updating  Sxtemp : ");
  //    ns_eigen_utils::printEigenMat(Sxtemp);

  choleskyOneRankUpdate(Sx_sqrt_, U, weight);

  //    ns_utils::print(" In measurement update AFTER updating  Sxtemp : ");
  //    ns_eigen_utils::printEigenMat(Sxtemp.eval());

  //    ns_utils::print(" In measurement update AFTER updating  Sx_sqrt_ : ");
  //    ns_eigen_utils::printEigenMat(Sx_sqrt_);

  // Normalize the angle variables
  Model::state_vector_t innovation_error = (x_measured - y_est_mean_full_);

  innovation_error(2) = ns_utils::angleDistance(innovation_error(2)); // yaw error diffs.
  innovation_error(5) = ns_utils::angleDistance(innovation_error(5));  // yaw_error

  x_est_mean_full_ = x_est_mean_full_ + Kk_ * innovation_error;

  // Normalize after the update.
  x_est_mean_full_(2) = ns_utils::angleDistance(x_est_mean_full_(2));
  x_est_mean_full_(5) = ns_utils::angleDistance(x_est_mean_full_(5));

  // DEBUG
  /*   ns_utils::print("First column operation : ");
      ns_eigen_utils::printEigenMat(first_column);

      ns_utils::print("Right columns operation : ");
      ns_eigen_utils::printEigenMat(right_columns);

      ns_utils::print("Rowise sum of y mean : ");
      ns_eigen_utils::printEigenMat(y_est_mean_full_);

      ns_utils::print("Covariance update Sk : ");
      ns_eigen_utils::printEigenMat(Sk_);

      ns_utils::print("Updated covariance in prediction update P_: ");
      ns_eigen_utils::printEigenMat(P_);

      ns_utils::print("W0m, W0c, W : ", Weight0_m_, Weight0_c_, Weight_common_); */
  // end of debug
}

void KalmanUnscentedSQRT::Initialize_xest0(Model::state_vector_t const &x0)
{
  x_est_mean_full_ = x0;
  is_initialized_ = true;
}

} // namespace ns_filters