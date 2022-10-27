// Copyright 2022 The Autoware Foundation.
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

#include "control/state_space.hpp"

#include "control/transfer_functions.hpp"

#include <utility>

ns_control_toolbox::tf2ss::tf2ss(const ns_control_toolbox::tf & sys_tf, const double & Ts)
: Ts_{Ts}, N_{sys_tf.order()}
{
  // N_ = sys_tf.order(); // size of A will be order of the TF.
  auto const & nx = N_;
  A_ = Eigen::MatrixXd::Zero(nx, nx);
  B_ = Eigen::MatrixXd::Zero(nx, 1);
  C_ = Eigen::MatrixXd::Zero(1, nx);
  D_ = Eigen::MatrixXd::Zero(1, 1);

  Ad_ = Eigen::MatrixXd::Zero(nx, nx);
  Bd_ = Eigen::MatrixXd::Zero(nx, 1);
  Cd_ = Eigen::MatrixXd::Zero(1, nx);
  Dd_ = Eigen::MatrixXd::Zero(1, 1);

  Tsimilarity_mat_ = Eigen::MatrixXd::Identity(nx, nx);

  updateStateSpace(sys_tf);
}

void ns_control_toolbox::tf2ss::updateStateSpace(const ns_control_toolbox::tf & sys_tf)
{
  auto && num = sys_tf.num();
  auto && den = sys_tf.den();

  // Check the leading zeros to determine the order of the system, and strip the leading zeros.
  ns_utils::stripVectorZerosFromLeft(num);
  ns_utils::stripVectorZerosFromLeft(den);

  // Compare if the system is a proper.
  if (den.size() < num.size()) {
    throw std::invalid_argument("This system is not a proper system.");
  }

  if (den.size() == 1 && num.size() == 1) {
    throw std::invalid_argument("System is a static gain not a dynamic system");
  }

  // Compute the system matrices.
  computeSystemMatrices(num, den);

  // Discretisize.
  discretisize(Ts_);
}

ns_control_toolbox::tf2ss::tf2ss(
  const std::vector<double> & numerator, const std::vector<double> & denominator, const double & Ts)
: Ts_{Ts}
{
  auto num = numerator;
  auto den = denominator;

  // Check the leading zeros to determine the order of the system, and strip the leading zeros.
  ns_utils::stripVectorZerosFromLeft(num);
  ns_utils::stripVectorZerosFromLeft(den);

  N_ = static_cast<Eigen::Index>(std::max(num.size(), den.size())) - 1;

  // Compare if the system is a proper.
  if (den.size() < num.size()) {
    throw std::invalid_argument("This system is not a proper system.");
  }

  if (den.size() == 1 && num.size() == 1) {
    throw std::invalid_argument("System is a static gain not a dynamic system");
  }

  // Initialize the system matrices.
  auto const & nx = N_;
  A_ = Eigen::MatrixXd::Zero(nx, nx);
  B_ = Eigen::MatrixXd::Zero(nx, 1);
  C_ = Eigen::MatrixXd::Zero(1, nx);
  D_ = Eigen::MatrixXd::Zero(1, 1);

  Ad_ = Eigen::MatrixXd::Zero(nx, nx);
  Bd_ = Eigen::MatrixXd::Zero(nx, 1);
  Cd_ = Eigen::MatrixXd::Zero(1, nx);
  Dd_ = Eigen::MatrixXd::Zero(1, 1);

  Tsimilarity_mat_ = Eigen::MatrixXd::Identity(nx, nx);

  // Compute the system matrices.
  computeSystemMatrices(num, den);

  // Discretisize
  discretisize(Ts);
}

void ns_control_toolbox::tf2ss::computeSystemMatrices(
  const std::vector<double> & num, const std::vector<double> & den)
{
  auto const & nx = N_;  // static_cast<long>(den.size() - 1);       // Order of the system.

  // We can put system check function if the nx = 0 -- i.e throw exception.
  // B_ = Eigen::MatrixXd::Identity(nx, 1); // We assign B here and this not only an initialization.

  if (A_.rows() != nx && A_.cols() != nx) {
    A_ = Eigen::MatrixXd::Zero(nx, nx);
    B_ = Eigen::MatrixXd::Zero(nx, 1);
    C_ = Eigen::MatrixXd::Zero(1, nx);
    D_ = Eigen::MatrixXd::Zero(1, 1);

    Ad_ = Eigen::MatrixXd::Zero(nx, nx);
    Bd_ = Eigen::MatrixXd::Zero(nx, 1);
    Cd_ = Eigen::MatrixXd::Zero(1, nx);
    Dd_ = Eigen::MatrixXd::Zero(1, 1);

    Tsimilarity_mat_ = Eigen::MatrixXd::Identity(nx, nx);
  }

  B_ = Eigen::MatrixXd::Identity(nx, 1);
  Tsimilarity_mat_.setIdentity();

  // Zero padding the numerator.
  auto const & num_of_zero = den.size() - num.size();

  std::vector<double> zero_padded_num{num};

  if (num_of_zero > 0) {
    zero_padded_num = ns_utils::zero_pad_left_first_arg(num, den);
  }

  // Normalize the numerator and denominator
  auto && den_first_item = den[0];
  std::vector<double> normalized_den{den};

  // normalize the numerator
  if (std::fabs(den_first_item) > EPS) {
    std::transform(
      zero_padded_num.begin(), zero_padded_num.end(), zero_padded_num.begin(),
      [&den_first_item](auto const & x) { return x / den_first_item; });

    std::transform(
      normalized_den.begin(), normalized_den.end(), normalized_den.begin(),
      [&den_first_item](auto const & x) { return x / den_first_item; });
  } else {
    throw std::invalid_argument("The first item in the denominator cannot be zero ...");
  }

  if (nx > 0) {
    D_(0, 0) = zero_padded_num[0];
  }

  //	auto B = Eigen::MatrixXd::Identity(nx - 1, nx);
  //	ns_eigen_utils::printEigenMat(B);

  if (nx > 1) {
    A_.bottomRows(nx - 1) = Eigen::MatrixXd::Identity(nx - 1, nx);
  }

  // normalize the denominator and assign the first row of the A_matrix to the normalized
  // denominator's values excluding the first item of the denominator.
  for (size_t k = 1; k < normalized_den.size(); k++) {
    auto const & ind_eig = Eigen::Index{static_cast<long>(k - 1)};
    A_(0, ind_eig) = -1 * normalized_den[k];
    C_(0, ind_eig) = zero_padded_num[k] - zero_padded_num[0] * normalized_den[k];
  }

  // Balance the matrices.
  ns_control_toolbox::balance_a_matrix(A_, Tsimilarity_mat_);

  // Balance C_ and B_;
  double const & nB = B_.lpNorm<1>();
  double const & nC = C_.lpNorm<1>();

  // alpha is a conditioning number that multiplies the smaller normed vector, divides the larger
  // one.
  double const & alpha = ns_control_toolbox::balance_symmetric(nB, nC);
  // ns_utils::print("Alpha :", alpha);

  if (nB < nC) {
    // Apply similarity transformation
    B_ = Tsimilarity_mat_.inverse() * B_.eval() * alpha;
    C_ = C_.eval() * Tsimilarity_mat_ / alpha;
  } else {
    B_ = Tsimilarity_mat_.inverse() * B_.eval() / alpha;
    C_ = C_.eval() * Tsimilarity_mat_ * alpha;
  }
}

void ns_control_toolbox::tf2ss::print() const
{
  ns_utils::print("A : ");
  ns_eigen_utils::printEigenMat(A());

  ns_utils::print("B : ");
  ns_eigen_utils::printEigenMat(B());

  ns_utils::print("C : ");
  ns_eigen_utils::printEigenMat(C());

  ns_utils::print("D : ");
  ns_eigen_utils::printEigenMat(D());
}

void ns_control_toolbox::tf2ss::print_discrete_system() const
{
  ns_utils::print("Ad : ");
  ns_eigen_utils::printEigenMat(Ad());

  ns_utils::print("Bd : ");
  ns_eigen_utils::printEigenMat(Bd());

  ns_utils::print("Cd : ");
  ns_eigen_utils::printEigenMat(Cd());

  ns_utils::print("Dd : ");
  ns_eigen_utils::printEigenMat(Dd());
}

/**
 * @brief Discretisize the continuous time state-space model using Tustin approximation.
 * */
void ns_control_toolbox::tf2ss::discretisize(double const & Ts)
{
  auto const & nx = N_;

  // take inverse:
  auto const && I = Eigen::MatrixXd::Identity(nx, nx);

  auto const && mat1_ATs = I - A_ * Ts / 2.;
  auto const && inv1_ATs = mat1_ATs.inverse();

  Ad_ = inv1_ATs * (I + A_ * Ts / 2.);
  Bd_ = inv1_ATs * B_ * Ts;
  Cd_ = C_ * inv1_ATs;
  Dd_ = D_ + C_ * Bd_ / 2.;
}

// Getters for the system matrices.
// Discrete time state-space matrices.
Eigen::MatrixXd ns_control_toolbox::tf2ss::Ad() const { return Ad_; }

Eigen::MatrixXd ns_control_toolbox::tf2ss::Bd() const { return Bd_; }

Eigen::MatrixXd ns_control_toolbox::tf2ss::Cd() const { return Cd_; }

Eigen::MatrixXd ns_control_toolbox::tf2ss::Dd() const { return Dd_; }

// Continuous time state-space matrices.
Eigen::MatrixXd ns_control_toolbox::tf2ss::A() const { return A_; }

Eigen::MatrixXd ns_control_toolbox::tf2ss::B() const { return B_; }

Eigen::MatrixXd ns_control_toolbox::tf2ss::C() const { return C_; }

Eigen::MatrixXd ns_control_toolbox::tf2ss::D() const { return D_; }

/**
 * @brief simulated the discrete system matrices [Ad, Bd:Cd, Dd] for one step. Its state matrix as
 * an input is a column matrix [x;u]. This state matrix returns as [x; y] which is in the form of xy
 * = [A B;C D]xu.
 * */

double ns_control_toolbox::tf2ss::simulateOneStep(Eigen::MatrixXd & x0, const double & u) const
{
  // First compute the output y.
  double y = (Cd_ * x0.eval() + Dd_ * u)(0, 0);

  // Then update x0;
  x0 = Ad_ * x0.eval() + Bd_ * u;

  return y;
}

Eigen::MatrixXd ns_control_toolbox::tf2ss::T() const { return Tsimilarity_mat_; }

ns_control_toolbox::scalarFilters_ss::scalarFilters_ss(
  const ns_control_toolbox::tf & sys_tf, const double & Ts)
{
  // Low-pass filter case
  auto num = sys_tf.num();
  auto den = sys_tf.den();

  double a_{};
  double b_{};
  double c_{};
  double d_{};

  if (num.size() == 1) {  // low-pass filter
    auto a0 = num[0];
    auto b0 = den[0];
    auto b1 = den[1];

    // state space
    a_ = -b1 / b0;
    b_ = 1 / b0;
    c_ = a0;
    d_ = 0.;
  }

  if (num.size() == 2) {  // high-pass filter
    auto a0 = num[0];
    auto a1 = num[1];

    auto b0 = den[0];
    auto b1 = den[1];

    // state space
    a_ = -b1 / b0;
    b_ = 1 / b0;

    c_ = a1 - a0 * b1 / b0;
    d_ = a0 / b0;
  }

  auto inv_a = 1. / (1. - a_ * Ts / 2);

  ad_ = inv_a * (1. + a_ * Ts / 2.);
  bd_ = inv_a * b_ * Ts;
  cd_ = c_ * inv_a;
  dd_ = d_ + c_ * bd_ / 2.;
}
double ns_control_toolbox::scalarFilters_ss::simulateOneStep(const double & u)
{
  double const & y = cd_ * x0_ + dd_ * u;
  x0_ = ad_ * x0_ + bd_ * u;

  return y;
}

void ns_control_toolbox::scalarFilters_ss::print() const
{
  ns_utils::print("Scalar filter discrete-time state space");
  ns_utils::print("Ad, Bd, Cd, Dd : ", ad_, bd_, cd_, dd_);
}
