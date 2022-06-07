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

#ifndef AUTOWARE_CONTROL_TOOLBOX_TRANSFER_FUNCTIONS_HPP
#define AUTOWARE_CONTROL_TOOLBOX_TRANSFER_FUNCTIONS_HPP

#include "act_definitions.hpp"
#include "act_utils.hpp"
#include "act_utils_eigen.hpp"
#include "tf_algebra.hpp"
#include "visibility_control.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace ns_control_toolbox
{

// Multiplication Operator.
template <typename T>
struct TF_multiplication
{
  friend T operator*(T const & tf1, T const & tf2)
  {
    // Compute numerator multiplication.
    tf_factor num1{{tf1.num_vector_only()}};
    tf_factor num2{{tf2.num_vector_only()}};

    auto num_mult = num1 * num2;

    // Multiply the numerator constants.
    auto num_constant_mult = tf1.num_constant() * tf2.num_constant();

    // Compute denominator multiplication.
    tf_factor den1{{tf1.den_vector_only()}};
    tf_factor den2{{tf2.den_vector_only()}};

    tf_factor den_mult = den1 * den2;

    // Multiply the numerator constants.
    auto den_constant_mult = tf1.den_constant() * tf2.den_constant();

    return T{num_mult(), den_mult(), num_constant_mult, den_constant_mult};
  };

  friend T operator*=(T const & tf1, T const & tf2)
  {
    // Compute numerator multiplication.
    tf_factor num1{{tf1.num_vector_only()}};
    tf_factor num2{{tf2.num_vector_only()}};

    auto num_mult = num1 * num2;

    // Multiply the numerator constants.
    auto num_constant_mult = tf1.num_const() * tf2.num_constant();

    // Compute denominator multiplication.
    tf_factor den1{{tf1.den_vector_only()}};
    tf_factor den2{{tf2.den_vector_only()}};

    tf_factor den_mult = den1 * den2;

    // Multiply the numerator constants.
    auto den_constant_mult = tf1.den_const() * tf2.den_constant();

    return T{num_mult(), den_mult(), num_constant_mult, den_constant_mult};
  };
};

// All algebraic operations.
template <typename T>
struct TF_algebra : public TF_multiplication<T>
{
};

/**
 * @brief Transfer Function representation in descending power of  Laplace variable "s".
 * [s^n, s^{n-1} ... s^0]
 * @param num	: std::vector<double> numerator
 * @param den	: std::vector<double> denominator
 * @param num_factor: numerator coefficient
 * @param den_factor: denominator coefficient
 * */

// For visibility, we can set for each object : __attribute__((visibility("default"))) class_name
struct ACT_PUBLIC tf : TF_algebra<tf>
{
  // Constructors.
  tf() : num_{1.}, den_{1.} {}

  // Constructor from non-empty numerator and denominator std::vectors.
  tf(std::vector<double> num, std::vector<double> den) : num_{std::move(num)}, den_{std::move(den)}
  {
    ns_utils::stripVectorZerosFromLeft(num_);  // remove zeros from the left.
    ns_utils::stripVectorZerosFromLeft(den_);
  }

  // Required for TF*TF multiplication.
  tf(std::vector<double> num, std::vector<double> den, double num_constant, double den_constant)
  : num_{std::move(num)},
    den_{std::move(den)},
    num_constant_{num_constant},
    den_constant_{den_constant}
  {
    ns_utils::stripVectorZerosFromLeft(num_);  // remove zeros from the left.
    ns_utils::stripVectorZerosFromLeft(den_);
  }

  // Constructor from tf_factors
  tf(tf_factor const & num, tf_factor const & den);

  // Member functions.
  /**
   * @brief : Creates a string stream object of polynomial representation of given the vector,
   * */
  static size_t getPolynomialStringAndSize(
    std::vector<double> const & num_or_den, std::ostringstream & string_stream);

  void print() const;

  [[nodiscard]] std::vector<double> num() const;  // gets num that multiplied by its constant.

  [[nodiscard]] std::vector<double> den() const;

  [[nodiscard]] std::vector<double> num_vector_only()
    const;  // gets num without multiplied by its constant.

  [[nodiscard]] std::vector<double> den_vector_only() const;  //

  [[nodiscard]] double num_constant() const;

  [[nodiscard]] double den_constant() const;

  // Invert the transfer function.
  void inv();

  // Update num and den
  void update_num(std::vector<double> const & num);

  void update_num(tf_factor const & num);

  void update_den(std::vector<double> const & den);

  void update_den(tf_factor const & den);

  // In some cases, the TF might have variable multiplier of the form a*[num], b*den where a, b
  // vary.
  void update_num_den_coef(double const & num_constant, double const & den_constant);

  void update_num_coef(double const & num_constant);

  void update_den_coef(double const & den_constant);

  [[nodiscard]] int order() const
  {
    return static_cast<int>(std::max(num_.size(), den_.size())) - 1;
  }

private:
  // Data members
  std::vector<double> num_{1.};  // <-@brief numerator
  std::vector<double> den_{1.};  // <-@brief denominator

  double num_constant_{1.};
  double den_constant_{1.};
};

/**
 * @param Td	: time delay value in seconds.
 * @param N		: Order of Pade approximation.
 * */
tf ACT_PUBLIC pade(double const & Td, size_t const & order);

/**
 * @bried see pade()
 * @refitem Golub and Van Loan, Matrix Computations, 4rd edition, Chapter 9., Section 9.3.1 pp 530
 * */
tf ACT_PUBLIC padecoeff(double const & Td, size_t const & order);

}  // namespace ns_control_toolbox

#endif  // AUTOWARE_CONTROL_TOOLBOX_TRANSFER_FUNCTIONS_HPP
