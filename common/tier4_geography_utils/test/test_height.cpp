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

#include "tier4_geography_utils/height.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

// Test case to verify if same source and target datums return original height
TEST(Tier4GeographyUtils, SameSourceTargetDatum)
{
  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;
  const std::string datum = "WGS84";

  double converted_height =
    tier4_geography_utils::convert_height(height, latitude, longitude, datum, datum);

  EXPECT_DOUBLE_EQ(height, converted_height);
}

// Test case to verify valid source and target datums
TEST(Tier4GeographyUtils, ValidSourceTargetDatum)
{
  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;

  double converted_height =
    tier4_geography_utils::convert_height(height, latitude, longitude, "WGS84", "EGM2008");

  // As this is a conversion, just verify that it does not return the original height
  // For precise testing, you'll need the exact expected values
  EXPECT_NE(height, converted_height);
}

// Test case to verify invalid source and target datums
TEST(Tier4GeographyUtils, InvalidSourceTargetDatum)
{
  const double height = 10.0;
  const double latitude = 35.0;
  const double longitude = 139.0;

  EXPECT_THROW(
    tier4_geography_utils::convert_height(height, latitude, longitude, "INVALID1", "INVALID2"),
    std::invalid_argument);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
