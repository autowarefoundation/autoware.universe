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

#include "modularized_particle_filter/prediction/resampler.hpp"

#include <rclcpp/time.hpp>

#include <gtest/gtest.h>

namespace mpf = pcdless::modularized_particle_filter;
using Particle = modularized_particle_filter_msgs::msg::Particle;
using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
using OptParticleArray = std::optional<ParticleArray>;

constexpr int RESAMPLE_INTERVAL = 5;
constexpr int PARTICLE_COUNT = 10;
constexpr int HISTORY_SIZE = 10;

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
    auto opt = resampler.retroactive_weighting(predicted, weighted);
    EXPECT_FALSE(opt.has_value());
  }

  // Future generation
  {
    weighted->id = 1;
    auto opt = resampler.retroactive_weighting(predicted, weighted);
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
    auto opt = resampler.retroactive_weighting(predicted, weighted);
    EXPECT_FALSE(opt.has_value());
  }
}

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
    OptParticleArray opt_array1 = resampler.retroactive_weighting(predicted, weighted);
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
    OptParticleArray opt_array1 = resampler.retroactive_weighting(predicted, weighted);
    EXPECT_TRUE(opt_array1.has_value());

    // All weight must match with following exepection
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

TEST(ResamplerTestSuite, resamplingWithRetrogression)
{
  mpf::RetroactiveResampler resampler(RESAMPLE_INTERVAL, PARTICLE_COUNT, HISTORY_SIZE);

  ParticleArray predicted;
  predicted.header.stamp = rclcpp::Time(0);
  predicted.particles.resize(PARTICLE_COUNT);
  predicted.id = 0;

  for (int i = 0; i < PARTICLE_COUNT; ++i) {
    auto & p = predicted.particles.at(i);
    p.weight = 1.0;
    if (i < PARTICLE_COUNT / 2)
      p.pose.position.x = 1;
    else
      p.pose.position.x = -1;
  }

  // First resampling must fail
  {
    OptParticleArray opt_array = resampler.resampling(predicted);
    EXPECT_FALSE(opt_array.has_value());
  }

  // Fill all history with biased weighted particles
  for (int p = 0; p < HISTORY_SIZE; ++p) {
    predicted.header.stamp = rclcpp::Time((RESAMPLE_INTERVAL + 1) * (p + 1), 0);
    auto opt_array = resampler.resampling(predicted);
    EXPECT_TRUE(opt_array.has_value());
    predicted = opt_array.value();
    EXPECT_EQ(predicted.id, p + 1);
  }

  // Update by ancient measurement
  {
    double before_centroid = 0;
    for (const auto & p : predicted.particles) {
      before_centroid += p.pose.position.x * p.weight;
    }

    // Weight
    ParticleArray::SharedPtr weighted = std::make_shared<ParticleArray>();
    weighted->particles.resize(PARTICLE_COUNT);
    weighted->id = 1;  // ancient generation id
    for (int i = 0; i < PARTICLE_COUNT; ++i) {
      auto & q = weighted->particles.at(i);
      if (i < PARTICLE_COUNT / 2) {
        q.weight = 2.0;
      } else {
        q.weight = 1.0;
      }
    }
    OptParticleArray opt_array = resampler.retroactive_weighting(predicted, weighted);
    EXPECT_TRUE(opt_array.has_value());
    predicted = opt_array.value();

    double after_centroid = 0;
    for (const auto & p : predicted.particles) {
      after_centroid += p.pose.position.x * p.weight;
    }
    EXPECT_TRUE(after_centroid > before_centroid);
  }
}