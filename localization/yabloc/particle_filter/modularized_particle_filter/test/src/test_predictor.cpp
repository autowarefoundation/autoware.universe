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