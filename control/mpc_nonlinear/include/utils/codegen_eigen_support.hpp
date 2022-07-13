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
#ifndef UTILS__CODEGEN_EIGEN_SUPPORT_HPP_
#define UTILS__CODEGEN_EIGEN_SUPPORT_HPP_

#include <Eigen/Core>
#include <cmath>
#include <cppad/cg.hpp>
#include <cppad/cg/support/cppadcg_eigen.hpp>

namespace Eigen
{
namespace internal
{
// !<-@brief Specialization of Eigen::internal::cast_impl for CppAD input types
template<typename Scalar>
struct cast_impl<CppAD::cg::CG<Scalar>, Scalar>
{
#if EIGEN_VERSION_AT_LEAST(3, 2, 90)
  EIGEN_DEVICE_FUNC
#endif

  static inline Scalar run(const CppAD::cg::CG<Scalar> & x) {return x.getValue();}
};
}  // namespace internal
}  // namespace Eigen

namespace CppAD
{
template<class Scalar>
bool isfinite(const cg::CG<Scalar> & x)
{
  return std::isfinite(x.getValue());
}

template<class Scalar>
bool isfinite(const AD<Scalar> & x)
{
  return isfinite(Value(x));
}
}  // namespace CppAD

#endif  // UTILS__CODEGEN_EIGEN_SUPPORT_HPP_
