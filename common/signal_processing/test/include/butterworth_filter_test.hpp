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

#ifndef SIGNAL_PROCESSING_TEST_INCLUDE_BUTTERWORTH_FILTER_TEST_HPP_
#define SIGNAL_PROCESSING_TEST_INCLUDE_BUTTERWORTH_FILTER_TEST_HPP_

#include "gtest/gtest.h"
#include "signal_processing/butterworth.hpp"

// The fixture for testing class Foo.
class ButterWorthTestFixture : public ::testing::Test
{
protected:
  // You can remove any or all of the following functions if their bodies would
  // be empty.

  ButterWorthTestFixture()
  {
    // You can do set-up work for each test here.
  }

  ~ButterWorthTestFixture() override
  {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override
  {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  void TearDown() override
  {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Class members declared here can be used by all tests in the test suite
  // for Foo.
};

#endif  // SIGNAL_PROCESSING_TEST_INCLUDE_BUTTERWORTH_FILTER_TEST_HPP_
