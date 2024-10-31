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

#include "cuda_utils/cuda_check_error.hpp"
#include "cuda_utils/cuda_unique_ptr.hpp"

#include <autoware/tensorrt_yolov10/calibrator.hpp>
#include <autoware/tensorrt_yolov10/preprocess.hpp>
#include <autoware/tensorrt_yolov10/tensorrt_yolov10.hpp>
#include <experimental/filesystem>

#include <assert.h>

#include <algorithm>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#define PRINT_DEBUG_INFO printf("line:%d\n", __LINE__);

namespace
{
static void trimLeft(std::string & s)
{
  s.erase(s.begin(), find_if(s.begin(), s.end(), [](int ch) { return !isspace(ch); }));
}

static void trimRight(std::string & s)
{
  s.erase(find_if(s.rbegin(), s.rend(), [](int ch) { return !isspace(ch); }).base(), s.end());
}

std::string trim(std::string & s)
{
  trimLeft(s);
  trimRight(s);
  return s;
}

bool fileExists(const std::string & file_name, bool verbose)
{
  if (!std::experimental::filesystem::exists(std::experimental::filesystem::path(file_name))) {
    if (verbose) {
      std::cout << "File does not exist : " << file_name << std::endl;
    }
    return false;
  }
  return true;
}

std::vector<std::string> loadListFromTextFile(const std::string & filename)
{
  assert(fileExists(filename, true));
  std::vector<std::string> list;

  std::ifstream f(filename);
  if (!f) {
    std::cout << "failed to open " << filename << std::endl;
    assert(0);
  }

  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) {
      continue;
    } else {
      list.push_back(trim(line));
    }
  }

  return list;
}

std::vector<std::string> loadImageList(const std::string & filename, const std::string & prefix)
{
  std::vector<std::string> fileList = loadListFromTextFile(filename);
  for (auto & file : fileList) {
    if (fileExists(file, false)) {
      continue;
    } else {
      std::string prefixed = prefix + file;
      if (fileExists(prefixed, false))
        file = prefixed;
      else
        std::cerr << "WARNING: couldn't find: " << prefixed << " while loading: " << filename
                  << std::endl;
    }
  }
  return fileList;
}

}  // anonymous namespace

namespace autoware::tensorrt_yolov10
{
TrtYolov10::TrtYolov10(
  const std::string & model_path, const std::string & precision, const int num_class,
  const float score_threshold, const tensorrt_common::BuildConfig build_config,
  const bool use_gpu_preprocess, const uint8_t gpu_id, std::string calibration_image_list_path,
  const double norm_factor, [[maybe_unused]] const std::string & cache_dir,
  const tensorrt_common::BatchConfig & batch_config, const size_t max_workspace_size)
: gpu_id_(gpu_id), is_gpu_initialized_(false)
{
  if (!setCudaDeviceId(gpu_id_)) {
    return;
  }
  is_gpu_initialized_ = true;

  num_class_ = num_class;
  score_threshold_ = score_threshold;

  src_width_ = -1;
  src_height_ = -1;
  norm_factor_ = norm_factor;
  batch_size_ = batch_config[2];
  multitask_ = 0;
  stream_ = makeCudaStream();

  if (precision == "int8") {
    if (build_config.clip_value <= 0.0) {
      if (calibration_image_list_path.empty()) {
        throw std::runtime_error(
          "calibration_image_list_path should be passed to generate int8 engine "
          "or specify values larger than zero to clip_value.");
      }
    } else {
      // if clip value is larger than zero, calibration file is not needed
      calibration_image_list_path = "";
    }

    int max_batch_size = batch_size_;
    nvinfer1::Dims input_dims = tensorrt_common::get_input_dims(model_path);
    std::vector<std::string> calibration_images;
    if (calibration_image_list_path != "") {
      calibration_images = loadImageList(calibration_image_list_path, "");
    }
    tensorrt_yolov10::ImageStream stream(max_batch_size, input_dims, calibration_images);
    fs::path calibration_table{model_path};
    std::string ext = "";
    if (build_config.calib_type_str == "Entropy") {
      ext = "EntropyV2-";
    } else if (
      build_config.calib_type_str == "Legacy" || build_config.calib_type_str == "Percentile") {
      ext = "Legacy-";
    } else {
      ext = "MinMax-";
    }

    ext += "calibration.table";
    calibration_table.replace_extension(ext);
    fs::path histogram_table{model_path};
    ext = "histogram.table";
    histogram_table.replace_extension(ext);

    std::unique_ptr<nvinfer1::IInt8Calibrator> calibrator;
    if (build_config.calib_type_str == "Entropy") {
      calibrator.reset(
        new tensorrt_yolov10::Int8EntropyCalibrator(stream, calibration_table, norm_factor_));

    } else if (
      build_config.calib_type_str == "Legacy" || build_config.calib_type_str == "Percentile") {
      const double quantile = 0.999999;
      const double cutoff = 0.999999;
      calibrator.reset(new tensorrt_yolov10::Int8LegacyCalibrator(
        stream, calibration_table, histogram_table, norm_factor_, true, quantile, cutoff));
    } else {
      calibrator.reset(
        new tensorrt_yolov10::Int8MinMaxCalibrator(stream, calibration_table, norm_factor_));
    }
    trt_common_ = std::make_unique<tensorrt_common::TrtCommon>(
      model_path, precision, std::move(calibrator), batch_config, max_workspace_size, build_config);
  } else {
    trt_common_ = std::make_unique<tensorrt_common::TrtCommon>(
      model_path, precision, nullptr, batch_config, max_workspace_size, build_config);
  }
  trt_common_->setup();

  if (!trt_common_->isInitialized()) {
    return;
  }
  num_class_ = num_class;
  score_threshold_ = score_threshold;

  // GPU memory allocation
  const auto input_dims = trt_common_->getBindingDimensions(0);
  const auto input_size =
    std::accumulate(input_dims.d + 1, input_dims.d + input_dims.nbDims, 1, std::multiplies<int>());

  const auto output_dims = trt_common_->getBindingDimensions(1);
  input_d_ = cuda_utils::make_unique<float[]>(batch_config[2] * input_size);
  max_detections_ = output_dims.d[1];
  out_elem_num_ = std::accumulate(
    output_dims.d + 1, output_dims.d + output_dims.nbDims, 1, std::multiplies<int>());
  out_elem_num_ = out_elem_num_ * batch_config[2];
  out_elem_num_per_batch_ = static_cast<int>(out_elem_num_ / batch_config[2]);
  out_d_ = cuda_utils::make_unique<float[]>(out_elem_num_);
  out_h_ = cuda_utils::make_unique_host<float[]>(out_elem_num_, cudaHostAllocPortable);

  if (use_gpu_preprocess) {
    use_gpu_preprocess_ = true;
    image_buf_h_ = nullptr;
    image_buf_d_ = nullptr;
  } else {
    use_gpu_preprocess_ = false;
  }
}

TrtYolov10::~TrtYolov10()
{
  if (use_gpu_preprocess_) {
    if (image_buf_h_) {
      image_buf_h_.reset();
    }
    if (image_buf_d_) {
      image_buf_d_.reset();
    }
    if (argmax_buf_d_) {
      argmax_buf_d_.reset();
    }
  }
}

bool TrtYolov10::setCudaDeviceId(const uint8_t gpu_id)
{
  cudaError_t cuda_err = cudaSetDevice(gpu_id);
  if (cuda_err != cudaSuccess) {
    return false;
  } else {
    return true;
  }
}

// void TrtYolov10::initPreprocessBuffer(int width, int height)
// {
//   // if size of source input has been changed...
//   if (src_width_ != -1 || src_height_ != -1) {
//     if (width != src_width_ || height != src_height_) {
//       // Free cuda memory to reallocate
//       if (image_buf_h_) {
//         image_buf_h_.reset();
//       }
//       if (image_buf_d_) {
//         image_buf_d_.reset();
//       }
//     }
//   }
//   src_width_ = width;
//   src_height_ = height;
//   if (use_gpu_preprocess_) {
//     auto input_dims = trt_common_->getBindingDimensions(0);
//     bool const hasRuntimeDim = std::any_of(
//       input_dims.d, input_dims.d + input_dims.nbDims,
//       [](int32_t input_dim) { return input_dim == -1; });
//     if (hasRuntimeDim) {
//       input_dims.d[0] = batch_size_;
//     }
//     if (!image_buf_h_) {
//       trt_common_->setBindingDimensions(0, input_dims);
//       scales_.clear();
//     }
//     const float input_height = static_cast<float>(input_dims.d[2]);
//     const float input_width = static_cast<float>(input_dims.d[3]);
//     if (!image_buf_h_) {
//       const float scale = std::min(input_width / width, input_height / height);
//       for (int b = 0; b < batch_size_; b++) {
//         scales_.emplace_back(scale);
//       }
//       image_buf_h_ = cuda_utils::make_unique_host<unsigned char[]>(
//         width * height * 3 * batch_size_, cudaHostAllocWriteCombined);
//       image_buf_d_ = cuda_utils::make_unique<unsigned char[]>(width * height * 3 * batch_size_);
//     }
//     if (multitask_) {
//       size_t argmax_out_elem_num = 0;
//       for (int m = 0; m < multitask_; m++) {
//         const auto output_dims =
//           trt_common_->getBindingDimensions(m + 2);  // 0 : input, 1 : output for detections
//         const float scale = std::min(
//           output_dims.d[3] / static_cast<float>(width),
//           output_dims.d[2] / static_cast<float>(height));
//         int out_w = static_cast<int>(width * scale);
//         int out_h = static_cast<int>(height * scale);
//         // size_t out_elem_num = std::accumulate(
//         // output_dims.d + 1, output_dims.d + output_dims.nbDims, 1, std::multiplies<int>());
//         // out_elem_num = out_elem_num * batch_size_;
//         size_t out_elem_num = out_w * out_h * batch_size_;
//         argmax_out_elem_num += out_elem_num;
//       }
//       argmax_buf_h_ =
//         cuda_utils::make_unique_host<unsigned char[]>(argmax_out_elem_num,
//         cudaHostAllocPortable);
//       argmax_buf_d_ = cuda_utils::make_unique<unsigned char[]>(argmax_out_elem_num);
//     }
//   }
// }

// void TrtYolov10::printProfiling(void)
// {
//   trt_common_->printProfiling();
// }

void TrtYolov10::preProcess(cv::Mat * img, int length, float * factor, std::vector<float> & data)
{
  // Create a new cv::Mat object for storing the image after conversion
  cv::Mat mat;

  // Get the dimensions and number of channels of the input image
  int rh = img->rows;        // Height of the input image
  int rw = img->cols;        // Width of the input image

  // Convert the input image from BGR to RGB color space
  cv::cvtColor(*img, mat, cv::COLOR_BGR2RGB);

  // Determine the size of the new square image (largest dimension of the input image)
  int maxImageLength = rw > rh ? rw : rh;

  // Create a new square image filled with zeros (black) with dimensions maxImageLength x
  // maxImageLength
  cv::Mat maxImage = cv::Mat::zeros(maxImageLength, maxImageLength, CV_8UC3);

  // Set all pixels to 255 (white)
  maxImage = maxImage * 255;

  // Define a Region of Interest (ROI) that covers the entire original image
  cv::Rect roi(0, 0, rw, rh);

  // Copy the original image into the ROI of the new square image
  mat.copyTo(cv::Mat(maxImage, roi));

  // Create a new cv::Mat object for storing the resized image
  cv::Mat resizeImg;

  // Resize the square image to the specified dimensions (length x length)
  cv::resize(maxImage, resizeImg, cv::Size(length, length), 0.0f, 0.0f, cv::INTER_LINEAR);

  // Calculate the scaling factor and store it in the 'factor' variable
  *factor = (float)((float)maxImageLength / (float)length);

  // Convert the resized image to floating-point format with values in range [0, 1]
  resizeImg.convertTo(resizeImg, CV_32FC3, 1 / 255.0);

  // Update the height, width, and number of channels for the resized image
  rh = resizeImg.rows;
  rw = resizeImg.cols;
  int rc = resizeImg.channels();

  // Extract each channel of the resized image and store it in the 'data' vector
  for (int i = 0; i < rc; ++i) {
    // Extract the i-th channel and store it in the appropriate part of the 'data' vector
    cv::extractChannel(resizeImg, cv::Mat(rh, rw, CV_32FC1, data.data() + i * rh * rw), i);
  }
}

void TrtYolov10::preprocess(const std::vector<cv::Mat> & images, std::vector<float> & inputData)
{
  //todo: support multi images. now assume that infer only one image.
  for (cv::Mat image : images) {
    int length = 640;  // 模型输入的大小
    factor_ = 1.0;

    preProcess(&image, length, &factor_, inputData);
    
    // // check sum
    // float sum = std::accumulate(inputData.begin(), inputData.end(), 0.0f);
    // printf("sum=%.2f\n", sum);
  }
}

// void TrtYolov10::preprocess(const std::vector<cv::Mat> & images)
// {
//   const auto batch_size = images.size();
//   auto input_dims = trt_common_->getBindingDimensions(0);
//   input_dims.d[0] = batch_size;
//   trt_common_->setBindingDimensions(0, input_dims);
//   const float input_height = static_cast<float>(input_dims.d[2]);
//   const float input_width = static_cast<float>(input_dims.d[3]);
//   std::vector<cv::Mat> dst_images;
//   scales_.clear();
//   for (const auto & image : images) {
//     cv::Mat dst_image;
//     const float scale = std::min(input_width / image.cols, input_height / image.rows);
//     scales_.emplace_back(scale);
//     const auto scale_size = cv::Size(image.cols * scale, image.rows * scale);
//     cv::resize(image, dst_image, scale_size, 0, 0, cv::INTER_CUBIC);
//     const auto bottom = input_height - dst_image.rows;
//     const auto right = input_width - dst_image.cols;
//     copyMakeBorder(dst_image, dst_image, 0, bottom, 0, right, cv::BORDER_CONSTANT, {114, 114,
//     114}); dst_images.emplace_back(dst_image);
//   }
//   const auto chw_images = cv::dnn::blobFromImages(
//     dst_images, norm_factor_, cv::Size(), cv::Scalar(), false, false, CV_32F);

//   const auto data_length = chw_images.total();
//   input_h_.reserve(data_length);
//   const auto flat = chw_images.reshape(1, data_length);
//   input_h_ = chw_images.isContinuous() ? flat : flat.clone();
//   CHECK_CUDA_ERROR(cudaMemcpy(
//     input_d_.get(), input_h_.data(), input_h_.size() * sizeof(float), cudaMemcpyHostToDevice));
//   // No Need for Sync
// }

bool TrtYolov10::doInference(const std::vector<cv::Mat> & images, ObjectArrays & objects)
{
  if (!setCudaDeviceId(gpu_id_)) {
    return false;
  }

  if (!trt_common_->isInitialized()) {
    return false;
  }

  std::vector<float> input_data(640 * 640 * 3);
  preprocess(images, input_data);
  CHECK_CUDA_ERROR(cudaMemcpy(
    input_d_.get(), input_data.data(), 3 * 640 * 640 * sizeof(float), cudaMemcpyHostToDevice));

  // preprocess(images);

  return feedforward(images, objects);
}

bool TrtYolov10::feedforward(const std::vector<cv::Mat> & images, ObjectArrays & objects)
{
  //   PRINT_DEBUG_INFO
  std::vector<void *> buffers = {input_d_.get(), out_d_.get()};

  trt_common_->enqueueV2(buffers.data(), *stream_, nullptr);

  //   printf("out_elem_num_:%d\n", (int)out_elem_num_);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_h_.get(), out_d_.get(), sizeof(float) * out_elem_num_, cudaMemcpyDeviceToHost, *stream_));

  cudaStreamSynchronize(*stream_);
  objects.clear();

  const auto batch_size = images.size();
  for (size_t i = 0; i < batch_size; ++i) {
    // auto image_size = images[i].size();
    float * batch_prob = out_h_.get() + (i * out_elem_num_per_batch_);

    ObjectArray object_array = postprocess(batch_prob, factor_);
    objects.emplace_back(object_array);
  }

  return true;
}

ObjectArray TrtYolov10::postprocess(float * result, float factor)
{
  ObjectArray object_array;
  std::vector<cv::Rect> positionBoxes;  // Stores bounding boxes of detected objects
  std::vector<int> classIds;            // Stores class IDs for detected objects
  std::vector<float> confidences;       // Stores confidence scores for detected objects

  // yolov10m out: 1 x 300 x 6
  for (int i = 0; i < max_detections_; i++) {
    int s = 6 * i;

    float score = result[s + 4];
    // printf("i:%d,score:%.3f\n",i,(float)result[s + 4]);

    if (score > score_threshold_) {
      printf("i:%d,score:%.3f\n", i, (float)result[s + 4]);

      // Extract the coordinates and dimensions of the bounding box (normalized values)
      float cx = result[s + 0];  // Center x-coordinate
      float cy = result[s + 1];  // Center y-coordinate
      float dx = result[s + 2];  // Bottom-right x-coordinate
      float dy = result[s + 3];  // Bottom-right y-coordinate

      // Convert normalized coordinates and dimensions to pixel values using the scaling factor
      int x = (int)((cx)*factor);              // Top-left x-coordinate of the bounding box
      int y = (int)((cy)*factor);              // Top-left y-coordinate of the bounding box
      int width = (int)((dx - cx) * factor);   // Width of the bounding box
      int height = (int)((dy - cy) * factor);  // Height of the bounding box

      // Create a cv::Rect object to represent the bounding box
      cv::Rect box(x, y, width, height);

      // Store the bounding box, class ID, and confidence score in the corresponding vectors
      positionBoxes.push_back(box);
      classIds.push_back(
        (int)result[s + 5]);  // Class ID is stored at position s + 5 in the 'result' array
      confidences.push_back((
        float)result[s + 4]);  // Confidence score is stored at position s + 4 in the 'result' array

      Object object;
      object.x_offset = x;
      object.y_offset = y;
      object.width = width;
      object.height = height;
      object.score = score;
      object.type = int(result[s + 5]);
      object_array.emplace_back(object);
    }
  }

  return object_array;
}

}  // namespace autoware::tensorrt_yolov10
