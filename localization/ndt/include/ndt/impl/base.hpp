// Copyright 2015-2019 Autoware Foundation
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

#ifndef NDT__IMPL__BASE_HPP_
#define NDT__IMPL__BASE_HPP_

#include "ndt/base.hpp"

template <class PointSource, class PointTarget>
NormalDistributionsTransformBase<PointSource, PointTarget>::NormalDistributionsTransformBase()
{
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformBase<PointSource, PointTarget>::setParam(
  const BaseParam & base_param)
{
  setTransformationEpsilon(base_param.trans_epsilon);
  setStepSize(base_param.step_size);
  setResolution(base_param.resolution);
  setMaximumIterations(base_param.max_iterations);
  setRegularizationScaleFactor(base_param.regularization_scale_factor);
}

template <class PointSource, class PointTarget>
typename NormalDistributionsTransformBase<PointSource, PointTarget>::BaseParam
NormalDistributionsTransformBase<PointSource, PointTarget>::getParam()
{
  BaseParam param;
  param.trans_epsilon = getTransformationEpsilon();
  param.step_size = getStepSize();
  param.resolution = getResolution();
  param.max_iterations = getMaximumIterations();
  param.regularization_scale_factor = getRegularizationScaleFactor();
  return param;
}

#endif  // NDT__IMPL__BASE_HPP_
