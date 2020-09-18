/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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

#ifndef BEHAVIOR_VELOCITY_PLANNER_UTILIZATION_INTERPOLATE_H_
#define BEHAVIOR_VELOCITY_PLANNER_UTILIZATION_INTERPOLATE_H_
#include <cmath>
#include <iostream>
#include <vector>

namespace interpolation
{
// template <class T>
// bool splineInterpolateWithFixInterval(const T & input, const double interval, T * output);

class LinearInterpolate
{
public:
  LinearInterpolate() {}
  static bool interpolate(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const std::vector<double> & return_index, std::vector<double> & return_value);
  static bool interpolate(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const double & return_index, double & return_value);
};

class SplineInterpolate
{
  bool initialized_;
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;

public:
  SplineInterpolate();
  explicit SplineInterpolate(const std::vector<double> & x);
  void generateSpline(const std::vector<double> & x);
  double getValue(const double & s);
  std::vector<double> getValueVector(const std::vector<double> & s_v);
  bool interpolate(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const std::vector<double> & return_index, std::vector<double> & return_value);
};

/*
 * helper functions
 */
bool isIncrease(const std::vector<double> & x);
bool isValidInput(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index, std::vector<double> & return_value);
std::vector<double> calcEuclidDist(const std::vector<double> & x, const std::vector<double> & y);

}  // namespace interpolation

#endif  // BEHAVIOR_VELOCITY_PLANNER_UTILIZATION_INTERPOLATE_H_
