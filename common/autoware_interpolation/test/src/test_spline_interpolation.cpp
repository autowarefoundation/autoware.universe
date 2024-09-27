// Copyright 2021 Tier IV, Inc.
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

#include "autoware/interpolation/spline_interpolation.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

constexpr double epsilon = 1e-6;

using autoware::interpolation::SplineInterpolation;

TEST(spline_interpolation, spline)
{
  {  // straight: query_keys is same as base_keys
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> base_values{0.0, 1.5, 3.0, 4.5, 6.0};
    const std::vector<double> query_keys = base_keys;
    const std::vector<double> ans = base_values;

    const auto query_values = autoware::interpolation::spline(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // straight: query_keys is random
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> base_values{0.0, 1.5, 3.0, 4.5, 6.0};
    const std::vector<double> query_keys{0.0, 0.7, 1.9, 4.0};
    const std::vector<double> ans{0.0, 1.05, 2.85, 6.0};

    const auto query_values = autoware::interpolation::spline(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // curve: query_keys is same as base_keys
    const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
    const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
    const std::vector<double> query_keys = base_keys;
    const std::vector<double> ans = base_values;

    const auto query_values = autoware::interpolation::spline(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // curve: query_keys is random
    const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
    const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
    const std::vector<double> query_keys{0.0, 8.0, 18.0};
    const std::vector<double> ans{-0.076114, 1.001217, 1.573640};

    const auto query_values = autoware::interpolation::spline(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // straight: size of base_keys is 2 (edge case in the implementation)
    const std::vector<double> base_keys{0.0, 1.0};
    const std::vector<double> base_values{0.0, 1.5};
    const std::vector<double> query_keys = base_keys;
    const std::vector<double> ans = base_values;

    const auto query_values = autoware::interpolation::spline(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // straight: size of base_keys is 3 (edge case in the implementation)
    const std::vector<double> base_keys{0.0, 1.0, 2.0};
    const std::vector<double> base_values{0.0, 1.5, 3.0};
    const std::vector<double> query_keys = base_keys;
    const std::vector<double> ans = base_values;

    const auto query_values = autoware::interpolation::spline(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // curve: query_keys is random. size of base_keys is 3 (edge case in the implementation)
    const std::vector<double> base_keys{-1.5, 1.0, 5.0};
    const std::vector<double> base_values{-1.2, 0.5, 1.0};
    const std::vector<double> query_keys{-1.0, 0.0, 4.0};
    const std::vector<double> ans{-0.808769, -0.077539, 1.035096};

    const auto query_values = autoware::interpolation::spline(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // When the query keys changes suddenly (edge case of spline interpolation).
    const std::vector<double> base_keys = {0.0, 1.0, 1.0001, 2.0, 3.0, 4.0};
    const std::vector<double> base_values = {0.0, 0.0, 0.1, 0.1, 0.1, 0.1};
    const std::vector<double> query_keys = {0.0, 1.0, 1.5, 2.0, 3.0, 4.0};
    const std::vector<double> ans = {0.0, 0.0, 158.738293, 0.1, 0.1, 0.1};

    const auto query_values = autoware::interpolation::spline(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }
}

TEST(spline_interpolation, splineByAkima)
{
  {  // straight: query_keys is same as base_keys
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> base_values{0.0, 1.5, 3.0, 4.5, 6.0};
    const std::vector<double> query_keys = base_keys;
    const std::vector<double> ans = base_values;

    const auto query_values =
      autoware::interpolation::splineByAkima(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // straight: query_keys is random
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> base_values{0.0, 1.5, 3.0, 4.5, 6.0};
    const std::vector<double> query_keys{0.0, 0.7, 1.9, 4.0};
    const std::vector<double> ans{0.0, 1.05, 2.85, 6.0};

    const auto query_values =
      autoware::interpolation::splineByAkima(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // curve: query_keys is same as base_keys
    const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
    const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
    const std::vector<double> query_keys = base_keys;
    const std::vector<double> ans = base_values;

    const auto query_values =
      autoware::interpolation::splineByAkima(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // curve: query_keys is random
    const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
    const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
    const std::vector<double> query_keys{0.0, 8.0, 18.0};
    const std::vector<double> ans{-0.0801, 1.110749, 1.4864};

    const auto query_values =
      autoware::interpolation::splineByAkima(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // straight: size of base_keys is 2 (edge case in the implementation)
    const std::vector<double> base_keys{0.0, 1.0};
    const std::vector<double> base_values{0.0, 1.5};
    const std::vector<double> query_keys = base_keys;
    const std::vector<double> ans = base_values;

    const auto query_values =
      autoware::interpolation::splineByAkima(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // straight: size of base_keys is 3 (edge case in the implementation)
    const std::vector<double> base_keys{0.0, 1.0, 2.0};
    const std::vector<double> base_values{0.0, 1.5, 3.0};
    const std::vector<double> query_keys = base_keys;
    const std::vector<double> ans = base_values;

    const auto query_values =
      autoware::interpolation::splineByAkima(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // curve: query_keys is random. size of base_keys is 3 (edge case in the implementation)
    const std::vector<double> base_keys{-1.5, 1.0, 5.0};
    const std::vector<double> base_values{-1.2, 0.5, 1.0};
    const std::vector<double> query_keys{-1.0, 0.0, 4.0};
    const std::vector<double> ans{-0.8378, -0.0801, 0.927031};

    const auto query_values =
      autoware::interpolation::splineByAkima(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // When the query keys changes suddenly (edge case of spline interpolation).
    const std::vector<double> base_keys = {0.0, 1.0, 1.0001, 2.0, 3.0, 4.0};
    const std::vector<double> base_values = {0.0, 0.0, 0.1, 0.1, 0.1, 0.1};
    const std::vector<double> query_keys = {0.0, 1.0, 1.5, 2.0, 3.0, 4.0};
    const std::vector<double> ans = {0.0, 0.0, 0.1, 0.1, 0.1, 0.1};

    const auto query_values =
      autoware::interpolation::splineByAkima(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }
}

TEST(spline_interpolation, SplineInterpolation)
{
  {
    // curve: query_keys is random
    const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
    const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
    const std::vector<double> query_keys{0.0, 8.0, 18.0};
    const std::vector<double> ans{-0.076114, 1.001217, 1.573640};

    SplineInterpolation s(base_keys, base_values);
    const std::vector<double> query_values = s.getSplineInterpolatedValues(query_keys);

    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {
    // getSplineInterpolatedDiffValues
    const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
    const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
    const std::vector<double> query_keys{0.0, 8.0, 12.0, 18.0};
    const std::vector<double> ans{0.671343, 0.049289, 0.209471, -0.253746};

    SplineInterpolation s(base_keys, base_values);
    const std::vector<double> query_values = s.getSplineInterpolatedDiffValues(query_keys);

    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {
    // getSplineInterpolatedQuadDiffValues
    const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
    const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
    const std::vector<double> query_keys{0.0, 8.0, 12.0, 18.0};
    const std::vector<double> ans{-0.155829, 0.043097, -0.011143, -0.049611};

    SplineInterpolation s(base_keys, base_values);
    const std::vector<double> query_values = s.getSplineInterpolatedQuadDiffValues(query_keys);

    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }
}
