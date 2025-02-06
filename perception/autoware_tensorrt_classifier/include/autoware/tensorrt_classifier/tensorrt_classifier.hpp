// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__TENSORRT_CLASSIFIER__TENSORRT_CLASSIFIER_HPP_
#define AUTOWARE__TENSORRT_CLASSIFIER__TENSORRT_CLASSIFIER_HPP_

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/stream_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware/tensorrt_common/tensorrt_conv_calib.hpp>
#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::tensorrt_classifier
{
using autoware::cuda_utils::CudaUniquePtr;
using autoware::cuda_utils::CudaUniquePtrHost;
using autoware::cuda_utils::makeCudaStream;
using autoware::cuda_utils::StreamUniquePtr;

/**
 * @class TrtClassifier
 * @brief TensorRT CLASSIFIER for faster inference
 */
class TrtClassifier
{
public:
  /**
   * @brief Construct TrtClassifier.
   * @param[in] mode_path ONNX model_path
   * @param[in] precision precision for inference
   * @param[in] calibration_images path for calibration files (only require for quantization)
   * @param[in] mean mean for preprocess
   * @param[in] std std for preprocess
   * @param[in] calib_config calibration configuration
   * @param[in] cuda whether use cuda gpu for preprocessing
   */
  TrtClassifier(
    const std::string & model_path, const std::string & precision,
    const std::vector<float> & mean = {123.675, 116.28, 103.53},
    const std::vector<float> & std = {58.395, 57.12, 57.375},
    const std::string & calibration_images = "",
    const tensorrt_common::CalibrationConfig & calibration_config =
      tensorrt_common::CalibrationConfig("MinMax", false, false, 0.0),
    const bool cuda = false);
  /**
   * @brief Deconstruct TrtClassifier
   */
  ~TrtClassifier();

  /**
   * @brief run inference including pre-process and post-process
   * @param[out] objects results for object detection
   * @param[in] images batched images
   */
  bool doInference(
    const std::vector<cv::Mat> & images, std::vector<int> & results,
    std::vector<float> & probabilities);

  /**
   * @brief allocate buffer for preprocess on GPU
   * @param[in] width original image width
   * @param[in] height original image height
   * @warning if we don't allocate buffers using it, "preprocessGpu" allocates buffers at the
   * beginning
   */
  void initPreprocessBuffer(int width, int height);

  /**
   * @brief get batch size
   * @return batch size
   */
  [[nodiscard]] int getBatchSize() const;

private:
  /**
   * @brief run preprocess including resizing, letterbox, BGR2RGB, NHWC2NCHW and toFloat on CPU
   * @param[in] images batching images
   */
  void preprocess_opt(const std::vector<cv::Mat> & images);

  /**
   * @brief run preprocess on GPU
   * @param[in] images batching images
   * @warning Current support is only a single batch image
   */
  void preprocessGpu(const std::vector<cv::Mat> & images);

  bool feedforwardAndDecode(
    const std::vector<cv::Mat> & images, std::vector<int> & results,
    std::vector<float> & probabilities);

  std::unique_ptr<tensorrt_common::TrtConvCalib> trt_common_;

  std::vector<float> input_h_;
  CudaUniquePtr<float[]> input_d_;

  size_t out_elem_num_;
  size_t out_elem_num_per_batch_;
  CudaUniquePtr<float[]> out_prob_d_;

  StreamUniquePtr stream_{makeCudaStream()};
  // mean for preprocessing
  std::vector<float> mean_;
  // std for preprocessing
  std::vector<float> std_;
  std::vector<float> inv_std_;
  // flg for preprocessing on GPU
  bool m_cuda;
  // host buffer for preprocessing on GPU
  unsigned char * h_img_;
  // device buffer for preprocessing on GPU
  unsigned char * d_img_;
  int src_width_;
  int src_height_;
  int batch_size_;
  CudaUniquePtrHost<float[]> out_prob_h_;
};
}  // namespace autoware::tensorrt_classifier

#endif  // AUTOWARE__TENSORRT_CLASSIFIER__TENSORRT_CLASSIFIER_HPP_
