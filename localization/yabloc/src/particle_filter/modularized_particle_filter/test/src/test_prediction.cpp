#include "modularized_particle_filter/prediction/predictor.hpp"

#include <gtest/gtest.h>

TEST(PredictionTest, predictionConstruct)
{
  Predictor predictor{};
  EXPECT_DOUBLE_EQ(3.0, 3.0);
}

TEST(PredictionTest, simpleTest) { EXPECT_DOUBLE_EQ(3.5, 3.0); }