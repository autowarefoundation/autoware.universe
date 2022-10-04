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

#include "ekf_localizer/covariance.hpp"

std::array<double, 36> ekfCovarianceToPoseMessageCovariance(const Matrix6d & P)
{
  const double p00 = P(0, 0);
  const double p01 = P(0, 1);
  const double p02 = P(0, 2);
  const double p10 = P(1, 0);
  const double p11 = P(1, 1);
  const double p12 = P(1, 2);
  const double p20 = P(2, 0);
  const double p21 = P(2, 1);
  const double p22 = P(2, 2);

  return std::array<double, 36>{
    p00, p01, 0.0, 0.0, 0.0, p02,
    p10, p11, 0.0, 0.0, 0.0, p12,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    p20, p21, 0.0, 0.0, 0.0, p22
  };
}

std::array<double, 36> ekfCovarianceToTwistMessageCovariance(const Matrix6d & P)
{
  const double p44 = P(4, 4);
  const double p45 = P(4, 5);
  const double p54 = P(5, 4);
  const double p55 = P(5, 5);

  return std::array<double, 36>{
    p44, 0.0, 0.0, 0.0, 0.0, p45,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    p54, 0.0, 0.0, 0.0, 0.0, p55
  };
}
