// Copyright 2024 Tier IV, Inc.
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

#include <autoware/motion_utils/trajectory_container/interpolator.hpp>

#include <gtest/gtest.h>

#include <optional>
#include <random>
#include <vector>

template <class Interpolator>
class TestInterpolator : public ::testing::Test
{
public:
  std::optional<Interpolator> interpolator;
  std::vector<double> bases;
  std::vector<double> values;

  void SetUp() override
  {
    // generate random values -1 to 1
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1, 1);
    bases.resize(10);
    values.resize(10);
    for (size_t i = 0; i < bases.size(); ++i) {
      bases[i] = static_cast<double>(i);
      values[i] = dis(gen);
    }
  }
};

using Interpolators = testing::Types<
  autoware::motion_utils::trajectory_container::interpolator::CubicSpline,
  autoware::motion_utils::trajectory_container::interpolator::AkimaSpline,
  autoware::motion_utils::trajectory_container::interpolator::Linear,
  autoware::motion_utils::trajectory_container::interpolator::NearestNeighbor<double>,
  autoware::motion_utils::trajectory_container::interpolator::Stairstep<double>>;

TYPED_TEST_SUITE(TestInterpolator, Interpolators, );

TYPED_TEST(TestInterpolator, compute)
{
  this->interpolator =
    typename TypeParam::Builder().set_bases(this->bases).set_values(this->values).build();
  for (size_t i = 0; i < this->bases.size(); ++i) {
    EXPECT_NEAR(this->values[i], this->interpolator->compute(this->bases[i]), 1e-6);
  }
}

// Instantiate test cases for all interpolators
template class TestInterpolator<
  autoware::motion_utils::trajectory_container::interpolator::CubicSpline>;
template class TestInterpolator<
  autoware::motion_utils::trajectory_container::interpolator::AkimaSpline>;
template class TestInterpolator<autoware::motion_utils::trajectory_container::interpolator::Linear>;
template class TestInterpolator<
  autoware::motion_utils::trajectory_container::interpolator::NearestNeighbor<double>>;
template class TestInterpolator<
  autoware::motion_utils::trajectory_container::interpolator::Stairstep<double>>;
