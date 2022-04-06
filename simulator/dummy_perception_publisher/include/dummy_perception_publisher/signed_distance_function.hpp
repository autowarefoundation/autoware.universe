// Copyright 2022 Tier IV, Inc.
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

#ifndef DUMMY_PERCEPTION_PUBLISHER__SIGNED_DISTANCE_FUNCTION_HPP_
#define DUMMY_PERCEPTION_PUBLISHER__SIGNED_DISTANCE_FUNCTION_HPP_

#include <tf2/LinearMath/Transform.h>

#include <memory>
#include <utility>
#include <vector>

namespace signed_distance_function
{

class AbstractSignedDistnaceFunction
{
public:
  virtual double operator()(double x, double y) const = 0;
  double getSphereTracingDist(
    double x_start, double y_start, double angle, double eps = 1e-3) const;
  virtual ~AbstractSignedDistnaceFunction() {}
};

class BoxSDF : public AbstractSignedDistnaceFunction
{
public:
  BoxSDF(double length, double width, tf2::Transform tf_global_to_local)
  : length_(length), width_(width), tf_global_to_local_(tf_global_to_local)
  {
  }
  double operator()(double x, double y) const override;

private:
  double length_;
  double width_;
  tf2::Transform tf_global_to_local_;
};

class CompisiteSDF : public AbstractSignedDistnaceFunction
{
public:
  explicit CompisiteSDF(std::vector<std::shared_ptr<AbstractSignedDistnaceFunction>> sdf_ptrs)
  : sdf_ptrs_(std::move(sdf_ptrs))
  {
  }
  double operator()(double x, double y) const override;

  size_t nearest_sdf_index(double x, double y) const;

private:
  std::vector<std::shared_ptr<AbstractSignedDistnaceFunction>> sdf_ptrs_;
};

}  // namespace signed_distance_function

#endif  // DUMMY_PERCEPTION_PUBLISHER__SIGNED_DISTANCE_FUNCTION_HPP_
