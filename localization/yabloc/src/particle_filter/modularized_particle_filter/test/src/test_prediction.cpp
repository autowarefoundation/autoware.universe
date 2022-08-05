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

// class TestPredictor : public mpf::Predictor
// {
// public:
//   TestPredictor() : Predictor() {}

// private:
// };

// TEST_F(PredictorTestSuite, predictionConstruct)
// {
//   auto node = std::make_shared<TestPredictor>();
//   EXPECT_DOUBLE_EQ(3.5, 3.5);
// }

using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
using OptParticleArray = std::optional<ParticleArray>;

constexpr int RESAMPLE_INTERVAL = 5;
constexpr int PARTICLE_COUNT = 10;
constexpr int HISTORY_SIZE = 10;

// Simple weighting without time retrogression
TEST(ResamplerTestSuite, simpleResampling)
{
  mpf::RetroactiveResampler resampler(RESAMPLE_INTERVAL, PARTICLE_COUNT, HISTORY_SIZE);

  ParticleArray predicted;
  predicted.header.stamp = rclcpp::Time(0);
  predicted.particles.resize(PARTICLE_COUNT);
  predicted.id = 0;
  for (int i = 0; i < PARTICLE_COUNT; ++i) predicted.particles.at(i).weight = 1;

  ParticleArray::SharedPtr weighted = std::make_shared<ParticleArray>();
  weighted->particles.resize(PARTICLE_COUNT);

  // First resampling must fail
  {
    OptParticleArray opt_array = resampler.resampling(predicted);
    EXPECT_FALSE(opt_array.has_value());
  }

  // Update by uniform distribution
  {
    // Weight
    weighted->id = 0;
    for (int i = 0; i < PARTICLE_COUNT; ++i) weighted->particles.at(i).weight = 0.5;
    OptParticleArray opt_array1 = resampler.retroactiveWeighting(predicted, weighted);
    EXPECT_TRUE(opt_array1.has_value());

    // All weights must be equal
    for (const auto & p : opt_array1->particles) EXPECT_NEAR(p.weight, 1.0 / PARTICLE_COUNT, 1e-3);

    // Resample
    predicted = opt_array1.value();
    predicted.header.stamp = rclcpp::Time(RESAMPLE_INTERVAL + 1, 0);
    OptParticleArray opt_array2 = resampler.resampling(predicted);
    EXPECT_TRUE(opt_array2.has_value());
    predicted = opt_array2.value();
    EXPECT_EQ(predicted.id, 1);
  }

  // Update half and half
  {
    // Weight
    weighted->id = 0;
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
      auto & p = predicted.particles.at(i);
      auto & q = weighted->particles.at(i);
      if (i < PARTICLE_COUNT / 2) {
        p.pose.position.x = 1;
        q.weight = 2.0;
      } else {
        p.pose.position.x = -1;
        q.weight = 1.0;
      }
    }
    OptParticleArray opt_array1 = resampler.retroactiveWeighting(predicted, weighted);
    EXPECT_TRUE(opt_array1.has_value());

    // All weights must be equal
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
      const auto & p = opt_array1->particles.at(i);
      if (i < PARTICLE_COUNT / 2) {
        EXPECT_NEAR(p.weight, 2.0 / 1.5 / PARTICLE_COUNT, 1e-3f);
      } else {
        EXPECT_NEAR(p.weight, 1.0 / 1.5 / PARTICLE_COUNT, 1e-3f);
      }
    }

    // Resample
    predicted = opt_array1.value();
    predicted.header.stamp = rclcpp::Time(2 * (RESAMPLE_INTERVAL + 1), 0);
    OptParticleArray opt_array2 = resampler.resampling(predicted);
    EXPECT_TRUE(opt_array2.has_value());
    predicted = opt_array2.value();
    EXPECT_EQ(predicted.id, 2);

    int centroid = 0;
    for (const auto & p : predicted.particles) centroid += p.pose.position.x;
    EXPECT_TRUE(centroid > 0);
  }
}

TEST(ResamplerTestSuite, ourOfHistory)
{
  mpf::RetroactiveResampler resampler(RESAMPLE_INTERVAL, PARTICLE_COUNT, HISTORY_SIZE);
  ParticleArray predicted;
  ParticleArray::SharedPtr weighted = std::make_shared<ParticleArray>();

  predicted.header.stamp = rclcpp::Time(0);
  predicted.id = 0;
  weighted->id = 0;
  predicted.particles.resize(PARTICLE_COUNT);
  weighted->particles.resize(PARTICLE_COUNT);
  for (auto & p : predicted.particles) p.weight = 1;
  for (auto & p : weighted->particles) p.weight = 1;

  // First resampling must fail
  {
    OptParticleArray opt_array = resampler.resampling(predicted);
    EXPECT_FALSE(opt_array.has_value());
  }

  // Invalid generation
  {
    weighted->id = -1;
    auto opt = resampler.retroactiveWeighting(predicted, weighted);
    EXPECT_FALSE(opt.has_value());
  }

  // Future generation
  {
    weighted->id = 1;
    auto opt = resampler.retroactiveWeighting(predicted, weighted);
    EXPECT_FALSE(opt.has_value());
  }

  // Repease resampling to fill all history
  for (int t = 0; t < HISTORY_SIZE; ++t) {
    predicted.header.stamp = rclcpp::Time((RESAMPLE_INTERVAL + 1) * (t + 1), 0);
    auto opt = resampler.resampling(predicted);
    EXPECT_TRUE(opt.has_value());
    EXPECT_EQ(opt->id, t + 1);
    predicted = opt.value();
  }

  // Too old generation
  {
    weighted->id = 0;
    auto opt = resampler.retroactiveWeighting(predicted, weighted);
    EXPECT_FALSE(opt.has_value());
  }
}