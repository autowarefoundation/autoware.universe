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

#ifndef UTILS__ACT_UTILS_HPP_
#define UTILS__ACT_UTILS_HPP_

#include <eigen3/Eigen/Dense>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <utility>
#include <vector>

/**
 * @brief Overloads vector * scalar multiplication.
 *
 * */

template<typename T, typename std::enable_if_t<std::is_floating_point_v<T>, std::vector<T>> * = nullptr>
std::vector<T> operator*(std::vector<T> const &vec, T const &a
)
{
  std::vector<T> temp{vec};

  std::transform(temp.begin(), temp.end(), temp.begin(), [&](auto &xi)
  { return xi * a; });

  //	for (auto& x: temp)
  //	{
  //		x = x * a;
  //	}
  return temp;
}

template<typename T, typename std::enable_if_t<std::is_floating_point_v<T>, std::vector<T>> * = nullptr>

std::vector<T> &operator*=(std::vector<T> &vec, T const &a)
{
  std::transform(vec.begin(), vec.end(), vec.begin(), [&](auto &xi)
  { return xi * a; });

  //	for (auto& x: vec)
  //	{
  //		x = x * a;
  //	}

  return vec;
}

template<typename T, typename std::enable_if_t<std::is_floating_point_v<T>, std::vector<T>> * = nullptr>

std::vector<T> operator*(T const &a, std::vector<T> const &vec)
{
  std::vector<T> temp{vec};

  std::transform(temp.begin(), temp.end(), temp.begin(), [&](auto &xi)
  { return xi * a; });

  //	for (auto& x: temp)
  //	{
  //		x = x * a;
  //	}

  return temp;
}

template<typename T, typename std::enable_if_t<std::is_floating_point<T>::value, std::vector<T>> * = nullptr>

std::vector<T> &operator*=(T const &a, std::vector<T> &vec)
{
  std::transform(vec.begin(), vec.end(), vec.begin(), [&](auto &xi)
  { return xi * a; });

  //	for (auto& x: vec)
  //	{
  //		x = x * a;
  //	}

  return vec;
}

namespace ns_utils
{
/**
 * @brief signum function.
 * */
template<typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

/**
 * @brief Saturates given values
 * */
template<typename T>
constexpr T clamp(const T &val, const T &lower, const T &upper)
{
  return std::max(lower, std::min(val, upper));
}

template<typename T>
void deg2rad(T &angle)
{
  angle *= M_PI / 180;
}

/**
 * brief abs_diff for size_t
 * */
template<typename T>
T abs_diff(T a, T b)
{
  return a > b ? a - b : b - a;
}

/**
 * @brief angle wrapping w(x) = mod(x, +pi, 2*pi) - pi
 * */
template<typename T>
constexpr T angleDistance(T const &theta)
{

  auto mod_theta_2pi = std::fmod(theta + M_PI, 2 * M_PI) - M_PI;

  return mod_theta_2pi < -M_PI ? mod_theta_2pi + 2 * M_PI : mod_theta_2pi;
}

template<typename T>
constexpr T angleDistance(T const &theta, T const &theta_ref)
{
  auto const &&angle_diff = theta - theta_ref;
  auto mod_theta_2pi = std::fmod(angle_diff + M_PI, 2 * M_PI) - M_PI;

  return mod_theta_2pi < -M_PI ? mod_theta_2pi + 2 * M_PI : mod_theta_2pi;
}

template<typename T>
T wrapToPi(T const &angle)
{
  const std::complex<double> i(0, 1);
  auto complex_number = std::exp(i * angle);
  return std::arg(complex_number);
}

/**
 * @brief Unwraps a series of angles.
 * */

template<typename T, typename std::enable_if_t<std::is_floating_point_v<T>> * = nullptr>
void unWrap(std::vector<T> &vec)
{
  auto const &n = vec.size();
  auto constexpr EPS = std::numeric_limits<T>::epsilon();
  auto pm = vec[0];

  auto thr = M_PI - EPS;
  T po{};

  if (!vec.empty())
  {
    for (auto k = 1; k < n; ++k)
    {
      auto cp = vec[k] + po;
      auto dp = cp - pm;
      pm = cp;

      if (dp > thr)
      {
        while (dp > thr)
        {
          po -= 2 * M_PI;
          dp -= 2 * M_PI;
        }
      }

      if (dp < -thr)
      {
        while (dp < -thr)
        {
          po += 2 * M_PI;
          dp += 2 * M_PI;
        }
      }

      cp = vec[k] + po;
      pm = cp;
      vec[k] = cp;
    }
  }
}

template<typename T>
void convertEulerAngleToMonotonic(std::vector<T> *a)
{
  if (!a)
  {
    return;
  }
  for (unsigned int i = 1; i < a->size(); ++i)
  {
    const double da = a->at(i) - a->at(i - 1);
    a->at(i) = a->at(i - 1) + wrapToPi<T>(da);
  }
}

// ---------------- Taken from Kinematic Control package -------------------

// Right hand sided tangent and normal vectors
inline std::array<double, 2> getTangentVector(double const &yaw_angle)
{
  return std::array<double, 2>{cos(yaw_angle), sin(yaw_angle)};
}

inline std::array<double, 2> getNormalVector(double const &yaw_angle)
{
  return std::array<double, 2>{-sin(yaw_angle), cos(yaw_angle)};
}

// Normal vector in Counter Clockwise.
constexpr std::array<double, 2>

getNormalVectorCC(double const &yaw_angle)
{
  return std::array<double, 2>{sin(yaw_angle), -cos(yaw_angle)};
}

inline double computeLateralError(std::array<double, 2>
                                  const &closest_point_position,
                                  std::array<double, 2>
                                  const &vehicle_position,
                                  double const &vehicle_yaw_angle)
{
  // Normal vector of vehicle direction
  auto &&normal_vector = getNormalVector(vehicle_yaw_angle);

  // Vector to path point originating from the vehicle
  std::array<double, 2> vector_to_path_point{vehicle_position[0] - closest_point_position[0],
                                             vehicle_position[1] - closest_point_position[1]};

  double lateral_error = normal_vector[0] * vector_to_path_point[0] + normal_vector[1] * vector_to_path_point[1];

  return lateral_error;
}

// Scales x = a*xhat + b for the given intervals.
constexpr std::array<double, 2> get_scalers(std::array<double, 2> const &x_minmax,
                                            std::array<double, 2> const &xhat_minmax)
{
  auto a_num = x_minmax[1] - x_minmax[0];  // xupper - xmin
  auto a_den = xhat_minmax[1] - xhat_minmax[0];

  auto a = a_num / a_den;
  auto b = x_minmax[0] - a * xhat_minmax[0];

  return std::array<double, 2>{a, b};
}

template<typename T>
constexpr std::vector<T> linspace(const T &start, const T &end, const size_t &num_of_steps)
{
  // Prepare a container.
  // Compute step increment.
  T step_increment = (end - start) / static_cast<double>(num_of_steps - 1);
  std::vector<T> linear_vector(num_of_steps, 0);

  linear_vector[0] = start;

  // Transform interpolated_value.
  for (auto it = linear_vector.begin() + 1; it != linear_vector.end(); it++)
  {
    *it = *std::prev(it) + step_increment;
  }

  return linear_vector;
}

// Prints any std vector type.
template<template<typename, typename> class ContainerType, typename ValueType, typename AllocType>
void constexpr print_container(const ContainerType<ValueType, AllocType> &c)
{
  std::cout << "\n [";

  for (auto it = c.cbegin(); it != c.cend() - 1; ++it)
  {
    std::cout << std::setprecision(4) << *it << ", ";
  }

  auto it = c.end() - 1;
  std::cout << *it << "]\n\n";
}

// Type definition.
template<typename T>
using eigen_dynamic_type = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// BINARY SEARCH for interpolating functions. // search value in the given interval
/**
 * @brief Binary search method for the interpolating functions.
 * @tparam scalar type,
 * @param searched value,
 * @param tbase base coordinate vector.
 *
 * */
template<typename T>
size_t constexpr binary_index_search(T const &ti, std::vector<T> const &tbase)
{
  size_t left_ind = 0;                  // low number
  size_t right_ind = tbase.size() - 1;  // last index of the coordinate - high number

  // Check if ti corresponds to the final value of tbase
  if (double EPS = 1e-6;ti >= tbase[right_ind] && std::fabs(tbase[right_ind] - ti) <= EPS)
  {
    left_ind = right_ind - 1;
    return left_ind;
  }

  while (right_ind > left_ind + 1)
  {
    // size_t mid = (left_ind + right_ind  ) / 2;
    size_t mid = left_ind + (right_ind - left_ind) / 2;  // to prevent overflow.

    if (ti < tbase[mid])
    {
      right_ind = mid;
    } else
    {
      left_ind = mid;
    }
    // ti < tbase[mid] ? right_ind : left_ind = mid;
  }

  return left_ind;
}

/**
 * @brief Interpolates the data at the new coordinates given the base coordinate-data pair.
 * @param [in] sbase_coord the base coordinate, monotonic series,
 * @param [in] base_data the base data,
 * @param [in] snew_coords the new coordinates,
 * @param [out] new_data_to_be_interpolated.
 * */
void interp1d_map_linear(std::vector<double>
                         const &sbase_coord,
                         std::vector<double> const &base_data, std::vector<double>
                         const &snew_coords,
                         std::vector<double> &new_data_to_be_interpolated);

void interp1d_linear(std::vector<double>
                     const &sbase_coord,
                     std::vector<double> const &base_data,
                     double const &snew_coord,
                     double &new_data_to_be_interpolated);

// Cross Product
template<typename T>
constexpr std::vector<T> crossProduct(std::vector<T> const &va, std::vector<T> const &vb)
{
  // Check size and make 3d vector for cross product
  if (va.size() < 3)
  {
    va.emplace_back(0.0);
  }

  float i = va[1] * vb[2] - va[2] * vb[1];
  float j = va[2] * vb[0] - va[0] * vb[2];
  float k = va[0] * vb[1] - va[1] * vb[0];
  std::vector<T> vcross_product{i, j, k};

  return vcross_product;
}

// PRINT MESSAGES
template<typename T>
constexpr void print(T const &msg)
{
  std::cout << msg << std::endl;
}

template<typename T0, typename T1>
constexpr void print(T0 const &msg0, T1 const &msg1)
{
  std::cout << msg0 << "  " << msg1 << std::endl;
}

template<class T0, class... Args>
constexpr void print(T0 &first, Args const &... args)
{
  std::cout << first << " ";
  print(args...);
}

// ------------ TIME MODULES -----------------------
double tic();

double toc(double start);

constexpr double exponentialMovingAverage(double const &previous_avg, double const &period, double const &new_value)
{
  const double smoothing_factor = 2. / (period + 1.);  // 2/(EMA_length + 1)
  const double results = (new_value - previous_avg) * smoothing_factor + previous_avg;
  return results;
}

/**
 * @brief Strip zeros of a vector from left.
 * */
void stripVectorZerosFromLeft(std::vector<double> &num_or_den);

// JOIN VECTORS
template<typename T>
std::vector<T> constexpr join_vectors(std::vector<T> a, std::vector<T> b)
{
  std::vector<T> ab;
  ab.reserve(a.size() + b.size());

  ab.insert(ab.begin(), a.cbegin(), a.cend());
  ab.insert(ab.end(), b.cbegin(), b.cend());

  return ab;
}

template<typename T, class... Args>
std::vector<T> constexpr join_vectors(std::vector<T> first, Args const &... args)
{
  return join_vectors(first, join_vectors(args...));
}

// COMPARE and ZERO PAD
template<typename T>
std::vector<std::vector<T>> zero_pad_left_make_equal(std::vector<T> const &a,
                                                     std::vector<T> const &b)
{
  auto asize = a.size();
  auto bsize = b.size();

  auto diff = abs_diff(asize, bsize);

  // Create a new vector
  if (asize > bsize)  // zero-pad b from left
  {
    auto c = join_vectors(std::vector<T>(diff), b);
    return std::vector<std::vector<T>>{a, c};
  }
  if (bsize > asize)
  {
    auto c = join_vectors(std::vector<T>(diff), a);
    return std::vector<std::vector<T>>{c, b};
  }

  return std::vector<std::vector<T>>{a, b};
}

template<typename T>
std::vector<T> zero_pad_left_first_arg(std::vector<T> const &a, std::vector<T> const &b
)
{
  auto asize = a.size();
  auto bsize = b.size();

  auto diff = abs_diff(asize, bsize);

  if (bsize > asize)
  {
    auto &&c = join_vectors(std::vector<T>(diff), a);
    return
      c;
  }

  return
    a;
}

// Comparing data types.
template<class T, typename std::enable_if_t<std::is_integral_v<T>, bool> * = nullptr>
bool isEqual(T a, T b)
{
  return a == b;
}

template<class T, typename std::enable_if_t<std::is_floating_point_v<T>, bool> * = nullptr>
bool isEqual(T a, T b)
{
  return abs(a - b) < std::numeric_limits<T>::epsilon();
}

/**
 * @brief Fetching the underlying type from strongly typed Enum class.
 * */
template<typename E>
constexpr auto toUType(E e) noexcept
{
  return static_cast        <std::underlying_type_t<E>>(e);
}

/**
 * @brief Implements heuristic angle distance for the angles that are wrapped to pi
 * */
// template<typename T>
// constexpr T angleDistanceHeuristics(T const &target_angle, T const &reference_angle)
//{
//	auto th_t = target_angle;
//	auto th_r = reference_angle;
//
//	auto sign_result = sgn(th_t * th_r);
//
//	if (sign_result < 0)
//	{
//		sgn(th_t) < 0 ? th_t : th_r += 2 * M_PI;
//	}
//
//	return;
// }

}  // namespace ns_utils

#endif  // UTILS__NMPC_UTILS_HPP_
