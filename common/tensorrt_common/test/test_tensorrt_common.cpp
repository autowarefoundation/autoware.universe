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

#include "tensorrt_common/tensorrt_common.hpp"

#include <gtest/gtest.h>

// test get_input_dims function
TEST(TrtCommonTest, TestGetInputDims)
{
  std::string onnx_file_path =
    "src/universe/autoware.universe/common/tensorrt_common/data/yolov5s.onnx";
  nvinfer1::Dims input_dims = tensorrt_common::get_input_dims(onnx_file_path);
  ASSERT_GT(input_dims.nbDims, 0);
}

// test is_valid_precision_string function
TEST(TrtCommonTest, TestIsValidPrecisionString)
{
  std::string valid_precision = "fp16";
  std::string invalid_precision = "invalid_precision";
  ASSERT_TRUE(tensorrt_common::is_valid_precision_string(valid_precision));
  ASSERT_FALSE(tensorrt_common::is_valid_precision_string(invalid_precision));
}

// In the future, more test cases will be written to test the functionality of TrtCommon class

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
