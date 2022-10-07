// Copyright 2022 Autoware Foundation
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

#ifndef NDT__UTILS_HPP_
#define NDT__UTILS_HPP_

#include <ndt/omp.hpp>
#include <ndt/pcl_generic.hpp>
#include <ndt/pcl_modified.hpp>

#include <fmt/format.h>

#include <memory>
#include <string>

template <typename PointSource, typename PointTarget>
std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> get_ndt(
  const NDTImplementType & ndt_implement_type)
{
  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> ndt_ptr;
  if (ndt_implement_type == NDTImplementType::PCL_GENERIC) {
    ndt_ptr.reset(new NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>);
    return ndt_ptr;
  }
  if (ndt_implement_type == NDTImplementType::PCL_MODIFIED) {
    ndt_ptr.reset(new NormalDistributionsTransformPCLModified<PointSource, PointTarget>);
    return ndt_ptr;
  }
  if (ndt_implement_type == NDTImplementType::OMP) {
    ndt_ptr.reset(new NormalDistributionsTransformOMP<PointSource, PointTarget>);
    return ndt_ptr;
  }

  const std::string s = fmt::format("Unknown NDT type {}", static_cast<int>(ndt_implement_type));
  throw std::runtime_error(s);
};

#endif  // NDT__UTILS_HPP_
