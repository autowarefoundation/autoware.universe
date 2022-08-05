#include "modularized_particle_filter/prediction/predictor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

class PredictorTestSuite : public ::testing::Test
{
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }
  static void TearDownTestCase() { rclcpp::shutdown(); }
};

namespace mpf = modularized_particle_filter;

class TestPredictor : public mpf::Predictor
{
public:
  TestPredictor() : Predictor() {}

private:
};

TEST_F(PredictorTestSuite, predictionConstruct)
{
  auto node = std::make_shared<TestPredictor>();
  EXPECT_DOUBLE_EQ(3.5, 3.5);
}

TEST_F(PredictorTestSuite, retroactiveResampler)
{
  auto resampler = std::make_shared<mpf::RetroactiveResampler>(10, 100, false);
  EXPECT_DOUBLE_EQ(3.5, 3.5);
}