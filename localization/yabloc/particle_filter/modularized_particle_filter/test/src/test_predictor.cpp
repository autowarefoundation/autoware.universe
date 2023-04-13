// Copyright 2023 TIER IV, Inc.
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

#include "modularized_particle_filter/prediction/predictor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

namespace mpf = pcdless::modularized_particle_filter;

class PredictorTestSuite : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { rclcpp::shutdown(); }
};

// class TestPredictor : public mpf::Predictor
// {
// public:
//   TestPredictor() : Predictor() {}
// };

// TEST_F(PredictorTestSuite, predictionConstruct)
// {
//   auto node = std::make_shared<TestPredictor>();
//   EXPECT_DOUBLE_EQ(3.5, 3.5);
// }