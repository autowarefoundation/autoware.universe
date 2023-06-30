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

#include <tensorrt_yolox/calibrator.hpp>
#include <tensorrt_yolox/tensorrt_yolox.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

static void leftTrim(std::string & s)
{
  s.erase(s.begin(), find_if(s.begin(), s.end(), [](int ch) { return !isspace(ch); }));
}

static void rightTrim(std::string & s)
{
  s.erase(find_if(s.rbegin(), s.rend(), [](int ch) { return !isspace(ch); }).base(), s.end());
}

std::string trim(std::string s)
{
  leftTrim(s);
  rightTrim(s);
  return s;
}

bool fileExists(const std::string fileName, bool verbose)
{
  if (!std::experimental::filesystem::exists(std::experimental::filesystem::path(fileName))) {
    if (verbose) std::cout << "File does not exist : " << fileName << std::endl;
    return false;
  }
  return true;
}

std::vector<std::string> loadListFromTextFile(const std::string filename)
{
  assert(fileExists(filename, true));
  std::vector<std::string> list;

  std::ifstream f(filename);
  if (!f) {
    std::cout << "failed to open " << filename;
    assert(0);
  }

  std::string line;
  while (std::getline(f, line)) {
    if (line.empty())
      continue;

    else
      list.push_back(trim(line));
  }

  return list;
}

std::vector<std::string> loadImageList(const std::string filename, const std::string prefix)
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

namespace tensorrt_yolox
{
TrtYoloX::TrtYoloX(
  const std::string & model_path, const std::string & precision, const int num_class,
  const float score_threshold, const float nms_threshold, tensorrt_common::BuildConfig build_config,
  const bool cuda, const std::string & calibration_image_list, const double scale,
  [[maybe_unused]] const std::string & cache_dir, const tensorrt_common::BatchConfig & batch_config,
  const size_t max_workspace_size)
{
  m_src_width = -1;
  m_src_height = -1;
  m_batch_size = batch_config[2];
  if (precision == "int8") {
    int max_batch_size = m_batch_size;
    nvinfer1::Dims input_dims = tensorrt_common::get_input_dims(model_path);
    std::vector<std::string> calibration_images = loadImageList(calibration_image_list, "");
    tensorrt_yolox::ImageStream stream(max_batch_size, input_dims, calibration_images);
    fs::path calibration_table{model_path};
    std::string calibName = "";
    std::string ext = "";
    if (build_config.calibType == nvinfer1::CalibrationAlgoType::kENTROPY_CALIBRATION) {
      ext = "EntropyV2-";
    } else if (build_config.calibType == nvinfer1::CalibrationAlgoType::kLEGACY_CALIBRATION) {
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
    if (build_config.calibType == nvinfer1::CalibrationAlgoType::kENTROPY_CALIBRATION) {
      calibrator.reset(
        new tensorrt_yolox::Int8EntropyCalibrator(stream, calibration_table, 1.0 / scale));
    } else if (build_config.calibType == nvinfer1::CalibrationAlgoType::kLEGACY_CALIBRATION) {
      double quantile = 0.999999;
      double cutoff = 0.999999;
      calibrator.reset(new tensorrt_yolox::Int8LegacyCalibrator(
        stream, calibration_table, histogram_table, 1.0 / scale, true, quantile, cutoff));
    } else {
      calibrator.reset(
        new tensorrt_yolox::Int8MinMaxCalibrator(stream, calibration_table, 1.0 / scale));
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

  // Judge whether decoding output is required
  // Plain models require decoding, while models with EfficientNMS_TRT module don't.
  // If getNbBindings == 5, the model contains EfficientNMS_TRT
  switch (trt_common_->getNbBindings()) {
    case 2:
      // Specified model is plain one.
      // Bindings are: [input, output]
      needs_output_decode_ = true;
      // The following three values are considered only if the specified model is plain one
      num_class_ = num_class;
      score_threshold_ = score_threshold;
      nms_threshold_ = nms_threshold;
      break;
    case 5:
      // Specified model contains Efficient NMS_TRT.
      // Bindings are[input, detection_classes, detection_scores, detection_boxes, num_detections]
      needs_output_decode_ = false;
      break;
    default:
      std::stringstream s;
      s << "\"" << model_path << "\" is unsuppoerted format";
      std::runtime_error{s.str()};
  }

  // GPU memory allocation
  const auto input_dims = trt_common_->getBindingDimensions(0);
  const auto input_size =
    std::accumulate(input_dims.d + 1, input_dims.d + input_dims.nbDims, 1, std::multiplies<int>());
  if (needs_output_decode_) {
    const auto output_dims = trt_common_->getBindingDimensions(1);
    input_d_ = cuda_utils::make_unique<float[]>(batch_config[2] * input_size);
    out_elem_num_ = std::accumulate(
      output_dims.d + 1, output_dims.d + output_dims.nbDims, 1, std::multiplies<int>());
    out_elem_num_ = out_elem_num_ * batch_config[2];
    out_elem_num_per_batch_ = static_cast<int>(out_elem_num_ / batch_config[2]);
    out_prob_d_ = cuda_utils::make_unique<float[]>(out_elem_num_);
    out_prob_h_ = cuda_utils::make_unique_host<float[]>(out_elem_num_, cudaHostAllocPortable);
    int w = input_dims.d[3];
    int h = input_dims.d[2];
    int sum_tensors = (w / 8) * (h / 8) + (w / 16) * (h / 16) + (w / 32) * (h / 32);
    if (sum_tensors == output_dims.d[1]) {
      // 3head (8,16,32)
      m_output_strides = {8, 16, 32};
    } else {
      // 4head (8,16,32.4)
      // last is additional head for high resolution outputs
      m_output_strides = {8, 16, 32, 4};
    }
  } else {
    const auto out_scores_dims = trt_common_->getBindingDimensions(3);
    max_detections_ = out_scores_dims.d[1];
    input_d_ = cuda_utils::make_unique<float[]>(batch_config[2] * input_size);
    out_num_detections_d_ = cuda_utils::make_unique<int32_t[]>(batch_config[2]);
    out_boxes_d_ = cuda_utils::make_unique<float[]>(batch_config[2] * max_detections_ * 4);
    out_scores_d_ = cuda_utils::make_unique<float[]>(batch_config[2] * max_detections_);
    out_classes_d_ = cuda_utils::make_unique<int32_t[]>(batch_config[2] * max_detections_);
  }
  if (cuda) {
    m_cuda = true;
    m_h_img = NULL;
    m_d_img = NULL;
    m_h_roi = NULL;
    m_d_roi = NULL;
  } else {
    m_cuda = false;
  }
  m_scale = 1.0 / scale;
}

TrtYoloX::~TrtYoloX()
{
  if (m_cuda) {
    if (m_h_img) CHECK_CUDA_ERROR(cudaFreeHost(m_h_img));
    if (m_d_img) CHECK_CUDA_ERROR(cudaFree(m_d_img));
    if (m_h_roi) CHECK_CUDA_ERROR(cudaFreeHost(m_h_roi));
    if (m_d_roi) CHECK_CUDA_ERROR(cudaFree(m_d_roi));
  }
}

void TrtYoloX::init_preproces_buffer(int width, int height)
{
  // if size of source input has benn changed...
  if (m_src_width != -1 || m_src_height != -1) {
    if (width != m_src_width || height != m_src_height) {
      // Free cuda memory to reallocate
      if (m_h_img) {
        CHECK_CUDA_ERROR(cudaFreeHost(m_h_img));
        m_h_img = NULL;
      }
      if (m_d_img) {
        CHECK_CUDA_ERROR(cudaFree(m_d_img));
        m_d_img = NULL;
      }
    }
  }
  m_src_width = width;
  m_src_height = height;
  if (m_cuda) {
    auto input_dims = trt_common_->getBindingDimensions(0);
    bool const hasRuntimeDim = std::any_of(
      input_dims.d, input_dims.d + input_dims.nbDims,
      [](int32_t input_dim) { return input_dim == -1; });
    if (hasRuntimeDim) {
      input_dims.d[0] = m_batch_size;
    }
    if (!m_h_img) {
      trt_common_->setBindingDimensions(0, input_dims);
      scales_.clear();
    }
    const float input_height = static_cast<float>(input_dims.d[2]);
    const float input_width = static_cast<float>(input_dims.d[3]);
    if (!m_h_img) {
      const float scale = std::min(input_width / width, input_height / height);
      for (int b = 0; b < m_batch_size; b++) {
        scales_.emplace_back(scale);
      }
      CHECK_CUDA_ERROR(cudaMallocHost(
        reinterpret_cast<void **>(&m_h_img),
        sizeof(unsigned char) * width * height * 3 * m_batch_size));
      CHECK_CUDA_ERROR(cudaMalloc(
        reinterpret_cast<void **>(&m_d_img),
        sizeof(unsigned char) * width * height * 3 * m_batch_size));
    }
  }
}

void TrtYoloX::preprocess_gpu(const std::vector<cv::Mat> & images)
{
  const auto batch_size = images.size();
  auto input_dims = trt_common_->getBindingDimensions(0);

  input_dims.d[0] = batch_size;
  for (const auto & image : images) {
    // if size of source input has benn changed...
    int width = image.cols;
    int height = image.rows;
    if (m_src_width != -1 || m_src_height != -1) {
      if (width != m_src_width || height != m_src_height) {
        // Free cuda memory to reallocate
        if (m_h_img) {
          CHECK_CUDA_ERROR(cudaFreeHost(m_h_img));
          m_h_img = NULL;
        }
        if (m_d_img) {
          CHECK_CUDA_ERROR(cudaFree(m_d_img));
          m_d_img = NULL;
        }
      }
    }
    m_src_width = width;
    m_src_height = height;
  }
  if (!m_h_img) {
    trt_common_->setBindingDimensions(0, input_dims);
    scales_.clear();
  }
  const float input_height = static_cast<float>(input_dims.d[2]);
  const float input_width = static_cast<float>(input_dims.d[3]);
  int b = 0;
  for (const auto & image : images) {
    if (!m_h_img) {
      const float scale = std::min(input_width / image.cols, input_height / image.rows);
      scales_.emplace_back(scale);
      CHECK_CUDA_ERROR(cudaMallocHost(
        reinterpret_cast<void **>(&m_h_img),
        sizeof(unsigned char) * image.cols * image.rows * 3 * batch_size));
      CHECK_CUDA_ERROR(cudaMalloc(
        reinterpret_cast<void **>(&m_d_img),
        sizeof(unsigned char) * image.cols * image.rows * 3 * batch_size));
    }
    int index = b * image.cols * image.rows * 3;
    // Copy into pinned memory
    memcpy(&(m_h_img[index]), &image.data[0], image.cols * image.rows * 3 * sizeof(unsigned char));
    b++;
  }
  // Copy into device memory
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    m_d_img, m_h_img, images[0].cols * images[0].rows * 3 * batch_size * sizeof(unsigned char),
    cudaMemcpyHostToDevice, *stream_));
  resize_bilinear_letterbox_NHWC2NCHW32_batch_gpu(
    input_d_.get(), m_d_img, input_width, input_height, 3, images[0].cols, images[0].rows, 3,
    batch_size, static_cast<float>(m_scale), *stream_);
  // No Need for Sync and used for timer
  // CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
}

void TrtYoloX::preprocess(const std::vector<cv::Mat> & images)
{
  const auto batch_size = images.size();
  auto input_dims = trt_common_->getBindingDimensions(0);
  input_dims.d[0] = batch_size;
  trt_common_->setBindingDimensions(0, input_dims);
  const float input_height = static_cast<float>(input_dims.d[2]);
  const float input_width = static_cast<float>(input_dims.d[3]);
  std::vector<cv::Mat> dst_images;
  scales_.clear();
  bool letterbox = true;
  if (letterbox) {
    for (const auto & image : images) {
      cv::Mat dst_image;
      const float scale = std::min(input_width / image.cols, input_height / image.rows);
      scales_.emplace_back(scale);
      const auto scale_size = cv::Size(image.cols * scale, image.rows * scale);
      cv::resize(image, dst_image, scale_size, 0, 0, cv::INTER_CUBIC);
      const auto bottom = input_height - dst_image.rows;
      const auto right = input_width - dst_image.cols;
      copyMakeBorder(
        dst_image, dst_image, 0, bottom, 0, right, cv::BORDER_CONSTANT, {114, 114, 114});
      dst_images.emplace_back(dst_image);
    }
  } else {
    for (const auto & image : images) {
      cv::Mat dst_image;
      const float scale = -1.0;
      scales_.emplace_back(scale);
      const auto scale_size = cv::Size(input_width, input_height);
      cv::resize(image, dst_image, scale_size, 0, 0, cv::INTER_CUBIC);
      dst_images.emplace_back(dst_image);
    }
  }
  const auto chw_images =
    cv::dnn::blobFromImages(dst_images, m_scale, cv::Size(), cv::Scalar(), false, false, CV_32F);

  const auto data_length = chw_images.total();
  input_h_.reserve(data_length);
  const auto flat = chw_images.reshape(1, data_length);
  input_h_ = chw_images.isContinuous() ? flat : flat.clone();
  CHECK_CUDA_ERROR(cudaMemcpy(
    input_d_.get(), input_h_.data(), input_h_.size() * sizeof(float), cudaMemcpyHostToDevice));
  // No Need for Sync
}

bool TrtYoloX::doInference(const std::vector<cv::Mat> & images, ObjectArrays & objects)
{
  if (!trt_common_->isInitialized()) {
    return false;
  }
  if (m_cuda) {
    preprocess_gpu(images);
  } else {
    preprocess(images);
  }

  if (needs_output_decode_) {
    return feedforwardAndDecode(images, objects);
  } else {
    return feedforward(images, objects);
  }
}

void TrtYoloX::preprocessWithRoi_gpu(
  const std::vector<cv::Mat> & images, const std::vector<cv::Rect> & rois)
{
  const auto batch_size = images.size();
  auto input_dims = trt_common_->getBindingDimensions(0);

  input_dims.d[0] = batch_size;
  for (const auto & image : images) {
    // if size of source input has benn changed...
    int width = image.cols;
    int height = image.rows;
    if (m_src_width != -1 || m_src_height != -1) {
      if (width != m_src_width || height != m_src_height) {
        // Free cuda memory to reallocate
        if (m_h_img) {
          CHECK_CUDA_ERROR(cudaFreeHost(m_h_img));
          m_h_img = NULL;
        }
        if (m_d_img) {
          CHECK_CUDA_ERROR(cudaFree(m_d_img));
          m_d_img = NULL;
        }
      }
    }
    m_src_width = width;
    m_src_height = height;
  }
  if (!m_h_img) {
    trt_common_->setBindingDimensions(0, input_dims);
  }
  const float input_height = static_cast<float>(input_dims.d[2]);
  const float input_width = static_cast<float>(input_dims.d[3]);
  int b = 0;
  scales_.clear();

  if (!m_h_roi) {
    CHECK_CUDA_ERROR(cudaMallocHost(reinterpret_cast<void **>(&m_h_roi), sizeof(Roi) * batch_size));
    CHECK_CUDA_ERROR(cudaMalloc(reinterpret_cast<void **>(&m_d_roi), sizeof(Roi) * batch_size));
  }

  for (const auto & image : images) {
    const float scale = std::min(
      input_width / static_cast<float>(rois[b].width),
      input_height / static_cast<float>(rois[b].height));
    scales_.emplace_back(scale);
    if (!m_h_img) {
      CHECK_CUDA_ERROR(cudaMallocHost(
        reinterpret_cast<void **>(&m_h_img),
        sizeof(unsigned char) * image.cols * image.rows * 3 * batch_size));
      CHECK_CUDA_ERROR(cudaMalloc(
        reinterpret_cast<void **>(&m_d_img),
        sizeof(unsigned char) * image.cols * image.rows * 3 * batch_size));
    }
    int index = b * image.cols * image.rows * 3;
    // Copy into pinned memory
    memcpy(&(m_h_img[index]), &image.data[0], image.cols * image.rows * 3 * sizeof(unsigned char));
    m_h_roi[b].x = rois[b].x;
    m_h_roi[b].y = rois[b].y;
    m_h_roi[b].w = rois[b].width;
    m_h_roi[b].h = rois[b].height;
    b++;
  }
  // Copy into device memory
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    m_d_img, m_h_img, images[0].cols * images[0].rows * 3 * batch_size * sizeof(unsigned char),
    cudaMemcpyHostToDevice, *stream_));
  CHECK_CUDA_ERROR(
    cudaMemcpyAsync(m_d_roi, m_h_roi, batch_size * sizeof(Roi), cudaMemcpyHostToDevice, *stream_));
  crop_resize_bilinear_letterbox_NHWC2NCHW32_batch_gpu(
    input_d_.get(), m_d_img, input_width, input_height, 3, m_d_roi, images[0].cols, images[0].rows,
    3, batch_size, static_cast<float>(m_scale), *stream_);
  // No Need for Sync and used for timer
  // CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
}

void TrtYoloX::preprocessWithRoi(
  const std::vector<cv::Mat> & images, const std::vector<cv::Rect> & rois)
{
  const auto batch_size = images.size();
  auto input_dims = trt_common_->getBindingDimensions(0);
  input_dims.d[0] = batch_size;
  trt_common_->setBindingDimensions(0, input_dims);
  const float input_height = static_cast<float>(input_dims.d[2]);
  const float input_width = static_cast<float>(input_dims.d[3]);
  std::vector<cv::Mat> dst_images;
  scales_.clear();
  bool letterbox = true;
  int b = 0;
  if (letterbox) {
    for (const auto & image : images) {
      cv::Mat dst_image;
      cv::Mat cropped = image(rois[b]);
      const float scale = std::min(
        input_width / static_cast<float>(rois[b].width),
        input_height / static_cast<float>(rois[b].height));
      scales_.emplace_back(scale);
      const auto scale_size = cv::Size(rois[b].width * scale, rois[b].height * scale);
      cv::resize(cropped, dst_image, scale_size, 0, 0, cv::INTER_CUBIC);
      const auto bottom = input_height - dst_image.rows;
      const auto right = input_width - dst_image.cols;
      copyMakeBorder(
        dst_image, dst_image, 0, bottom, 0, right, cv::BORDER_CONSTANT, {114, 114, 114});
      dst_images.emplace_back(dst_image);
      b++;
    }
  } else {
    for (const auto & image : images) {
      cv::Mat dst_image;
      const float scale = -1.0;
      scales_.emplace_back(scale);
      const auto scale_size = cv::Size(input_width, input_height);
      cv::resize(image, dst_image, scale_size, 0, 0, cv::INTER_CUBIC);
      dst_images.emplace_back(dst_image);
    }
  }
  const auto chw_images =
    cv::dnn::blobFromImages(dst_images, m_scale, cv::Size(), cv::Scalar(), false, false, CV_32F);

  const auto data_length = chw_images.total();
  input_h_.reserve(data_length);
  const auto flat = chw_images.reshape(1, data_length);
  input_h_ = chw_images.isContinuous() ? flat : flat.clone();
  CHECK_CUDA_ERROR(cudaMemcpy(
    input_d_.get(), input_h_.data(), input_h_.size() * sizeof(float), cudaMemcpyHostToDevice));
  // No Need for Sync
}

void TrtYoloX::multiScalePreprocess_gpu(const cv::Mat & image, const std::vector<cv::Rect> & rois)
{
  const auto batch_size = rois.size();
  auto input_dims = trt_common_->getBindingDimensions(0);

  input_dims.d[0] = batch_size;

  // if size of source input has benn changed...
  int width = image.cols;
  int height = image.rows;
  if (m_src_width != -1 || m_src_height != -1) {
    if (width != m_src_width || height != m_src_height) {
      // Free cuda memory to reallocate
      if (m_h_img) {
        CHECK_CUDA_ERROR(cudaFreeHost(m_h_img));
        m_h_img = NULL;
      }
      if (m_d_img) {
        CHECK_CUDA_ERROR(cudaFree(m_d_img));
        m_d_img = NULL;
      }
    }
  }
  m_src_width = width;
  m_src_height = height;

  if (!m_h_img) {
    trt_common_->setBindingDimensions(0, input_dims);
  }
  const float input_height = static_cast<float>(input_dims.d[2]);
  const float input_width = static_cast<float>(input_dims.d[3]);

  scales_.clear();

  if (!m_h_roi) {
    CHECK_CUDA_ERROR(cudaMallocHost(reinterpret_cast<void **>(&m_h_roi), sizeof(Roi) * batch_size));
    CHECK_CUDA_ERROR(cudaMalloc(reinterpret_cast<void **>(&m_d_roi), sizeof(Roi) * batch_size));
  }

  for (size_t b = 0; b < rois.size(); b++) {
    const float scale = std::min(
      input_width / static_cast<float>(rois[b].width),
      input_height / static_cast<float>(rois[b].height));
    scales_.emplace_back(scale);
    m_h_roi[b].x = rois[b].x;
    m_h_roi[b].y = rois[b].y;
    m_h_roi[b].w = rois[b].width;
    m_h_roi[b].h = rois[b].height;
  }
  if (!m_h_img) {
    CHECK_CUDA_ERROR(cudaMallocHost(
      reinterpret_cast<void **>(&m_h_img),
      sizeof(unsigned char) * image.cols * image.rows * 3 * 1));
    CHECK_CUDA_ERROR(cudaMalloc(
      reinterpret_cast<void **>(&m_d_img),
      sizeof(unsigned char) * image.cols * image.rows * 3 * 1));
  }
  int index = 0 * image.cols * image.rows * 3;
  // Copy into pinned memory
  memcpy(&(m_h_img[index]), &image.data[0], image.cols * image.rows * 3 * sizeof(unsigned char));

  // Copy into device memory
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    m_d_img, m_h_img, image.cols * image.rows * 3 * 1 * sizeof(unsigned char),
    cudaMemcpyHostToDevice, *stream_));
  CHECK_CUDA_ERROR(
    cudaMemcpyAsync(m_d_roi, m_h_roi, batch_size * sizeof(Roi), cudaMemcpyHostToDevice, *stream_));
  multi_scale_resize_bilinear_letterbox_NHWC2NCHW32_batch_gpu(
    input_d_.get(), m_d_img, input_width, input_height, 3, m_d_roi, image.cols, image.rows, 3,
    batch_size, static_cast<float>(m_scale), *stream_);
  // No Need for Sync and used for timer
  // CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
}

void TrtYoloX::multiScalePreprocess(const cv::Mat & image, const std::vector<cv::Rect> & rois)
{
  const auto batch_size = rois.size();
  auto input_dims = trt_common_->getBindingDimensions(0);
  input_dims.d[0] = batch_size;
  trt_common_->setBindingDimensions(0, input_dims);
  const float input_height = static_cast<float>(input_dims.d[2]);
  const float input_width = static_cast<float>(input_dims.d[3]);
  std::vector<cv::Mat> dst_images;
  scales_.clear();

  for (const auto & roi : rois) {
    cv::Mat dst_image;
    cv::Mat cropped = image(roi);
    const float scale = std::min(
      input_width / static_cast<float>(roi.width), input_height / static_cast<float>(roi.height));
    scales_.emplace_back(scale);
    const auto scale_size = cv::Size(roi.width * scale, roi.height * scale);
    cv::resize(cropped, dst_image, scale_size, 0, 0, cv::INTER_CUBIC);
    const auto bottom = input_height - dst_image.rows;
    const auto right = input_width - dst_image.cols;
    copyMakeBorder(dst_image, dst_image, 0, bottom, 0, right, cv::BORDER_CONSTANT, {114, 114, 114});
    dst_images.emplace_back(dst_image);
  }
  const auto chw_images =
    cv::dnn::blobFromImages(dst_images, m_scale, cv::Size(), cv::Scalar(), false, false, CV_32F);

  const auto data_length = chw_images.total();
  input_h_.reserve(data_length);
  const auto flat = chw_images.reshape(1, data_length);
  input_h_ = chw_images.isContinuous() ? flat : flat.clone();
  CHECK_CUDA_ERROR(cudaMemcpy(
    input_d_.get(), input_h_.data(), input_h_.size() * sizeof(float), cudaMemcpyHostToDevice));
  // No Need for Sync
}

bool TrtYoloX::doInferenceWithRoi(
  const std::vector<cv::Mat> & images, ObjectArrays & objects, const std::vector<cv::Rect> & rois)
{
  if (!trt_common_->isInitialized()) {
    return false;
  }
  if (m_cuda) {
    preprocessWithRoi_gpu(images, rois);
  } else {
    preprocessWithRoi(images, rois);
  }

  if (needs_output_decode_) {
    return feedforwardAndDecode(images, objects);
  } else {
    return feedforward(images, objects);
  }
}

bool TrtYoloX::doMultiScaleInference(
  const cv::Mat & image, ObjectArrays & objects, const std::vector<cv::Rect> & rois)
{
  std::vector<cv::Mat> images;
  if (!trt_common_->isInitialized()) {
    return false;
  }
  if (m_cuda) {
    multiScalePreprocess_gpu(image, rois);
  } else {
    multiScalePreprocess(image, rois);
  }
  if (needs_output_decode_) {
    return multiScaleFeedforwardAndDecode(image, rois.size(), objects);
  } else {
    return multiScaleFeedforward(image, rois.size(), objects);
  }
}

// This method is assumed to be called when specified YOLOX model contains
// EfficientNMS_TRT module.
bool TrtYoloX::feedforward(const std::vector<cv::Mat> & images, ObjectArrays & objects)
{
  std::vector<void *> buffers = {
    input_d_.get(), out_num_detections_d_.get(), out_boxes_d_.get(), out_scores_d_.get(),
    out_classes_d_.get()};

  trt_common_->enqueueV2(buffers.data(), *stream_, nullptr);

  const auto batch_size = images.size();
  auto out_num_detections = std::make_unique<int32_t[]>(batch_size);
  auto out_boxes = std::make_unique<float[]>(4 * batch_size * max_detections_);
  auto out_scores = std::make_unique<float[]>(batch_size * max_detections_);
  auto out_classes = std::make_unique<int32_t[]>(batch_size * max_detections_);

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_num_detections.get(), out_num_detections_d_.get(), sizeof(int32_t) * batch_size,
    cudaMemcpyDeviceToHost, *stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_boxes.get(), out_boxes_d_.get(), sizeof(float) * 4 * batch_size * max_detections_,
    cudaMemcpyDeviceToHost, *stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_scores.get(), out_scores_d_.get(), sizeof(float) * batch_size * max_detections_,
    cudaMemcpyDeviceToHost, *stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_classes.get(), out_classes_d_.get(), sizeof(int32_t) * batch_size * max_detections_,
    cudaMemcpyDeviceToHost, *stream_));
  cudaStreamSynchronize(*stream_);

  objects.clear();
  for (size_t i = 0; i < batch_size; ++i) {
    const size_t num_detection = static_cast<size_t>(out_num_detections[i]);
    ObjectArray object_array(num_detection);
    for (size_t j = 0; j < num_detection; ++j) {
      Object object{};
      const auto x1 = out_boxes[i * max_detections_ * 4 + j * 4] / scales_[i];
      const auto y1 = out_boxes[i * max_detections_ * 4 + j * 4 + 1] / scales_[i];
      const auto x2 = out_boxes[i * max_detections_ * 4 + j * 4 + 2] / scales_[i];
      const auto y2 = out_boxes[i * max_detections_ * 4 + j * 4 + 3] / scales_[i];
      object.x_offset = std::clamp(0, static_cast<int32_t>(x1), images[i].cols);
      object.y_offset = std::clamp(0, static_cast<int32_t>(y1), images[i].rows);
      object.width = static_cast<int32_t>(std::max(0.0F, x2 - x1));
      object.height = static_cast<int32_t>(std::max(0.0F, y2 - y1));
      object.score = out_scores[i * max_detections_ + j];
      object.type = out_classes[i * max_detections_ + j];
      object_array.emplace_back(object);
    }
    objects.emplace_back(object_array);
  }
  return true;
}

bool TrtYoloX::feedforwardAndDecode(const std::vector<cv::Mat> & images, ObjectArrays & objects)
{
  std::vector<void *> buffers = {input_d_.get(), out_prob_d_.get()};
  trt_common_->enqueueV2(buffers.data(), *stream_, nullptr);

  int batch_size = static_cast<int>(images.size());

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_prob_h_.get(), out_prob_d_.get(), sizeof(float) * out_elem_num_, cudaMemcpyDeviceToHost,
    *stream_));
  cudaStreamSynchronize(*stream_);
  objects.clear();

  for (int i = 0; i < batch_size; ++i) {
    auto image_size = images[i].size();
    float * batch_prob = out_prob_h_.get() + (i * out_elem_num_per_batch_);
    ObjectArray object_array;
    decodeOutputs(batch_prob, object_array, scales_[i], image_size);
    objects.emplace_back(object_array);
  }
  return true;
}

// This method is assumed to be called when specified YOLOX model contains
// EfficientNMS_TRT module.
bool TrtYoloX::multiScaleFeedforward(const cv::Mat & image, int batch_size, ObjectArrays & objects)
{
  std::vector<void *> buffers = {
    input_d_.get(), out_num_detections_d_.get(), out_boxes_d_.get(), out_scores_d_.get(),
    out_classes_d_.get()};

  trt_common_->enqueueV2(buffers.data(), *stream_, nullptr);

  auto out_num_detections = std::make_unique<int32_t[]>(batch_size);
  auto out_boxes = std::make_unique<float[]>(4 * batch_size * max_detections_);
  auto out_scores = std::make_unique<float[]>(batch_size * max_detections_);
  auto out_classes = std::make_unique<int32_t[]>(batch_size * max_detections_);

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_num_detections.get(), out_num_detections_d_.get(), sizeof(int32_t) * batch_size,
    cudaMemcpyDeviceToHost, *stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_boxes.get(), out_boxes_d_.get(), sizeof(float) * 4 * batch_size * max_detections_,
    cudaMemcpyDeviceToHost, *stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_scores.get(), out_scores_d_.get(), sizeof(float) * batch_size * max_detections_,
    cudaMemcpyDeviceToHost, *stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_classes.get(), out_classes_d_.get(), sizeof(int32_t) * batch_size * max_detections_,
    cudaMemcpyDeviceToHost, *stream_));
  cudaStreamSynchronize(*stream_);

  objects.clear();
  for (int i = 0; i < batch_size; ++i) {
    const size_t num_detection = static_cast<size_t>(out_num_detections[i]);
    ObjectArray object_array(num_detection);
    for (size_t j = 0; j < num_detection; ++j) {
      Object object{};
      const auto x1 = out_boxes[i * max_detections_ * 4 + j * 4] / scales_[i];
      const auto y1 = out_boxes[i * max_detections_ * 4 + j * 4 + 1] / scales_[i];
      const auto x2 = out_boxes[i * max_detections_ * 4 + j * 4 + 2] / scales_[i];
      const auto y2 = out_boxes[i * max_detections_ * 4 + j * 4 + 3] / scales_[i];
      object.x_offset = std::clamp(0, static_cast<int32_t>(x1), image.cols);
      object.y_offset = std::clamp(0, static_cast<int32_t>(y1), image.rows);
      object.width = static_cast<int32_t>(std::max(0.0F, x2 - x1));
      object.height = static_cast<int32_t>(std::max(0.0F, y2 - y1));
      object.score = out_scores[i * max_detections_ + j];
      object.type = out_classes[i * max_detections_ + j];
      object_array.emplace_back(object);
    }
    objects.emplace_back(object_array);
  }
  return true;
}

bool TrtYoloX::multiScaleFeedforwardAndDecode(
  const cv::Mat & image, int batch_size, ObjectArrays & objects)
{
  std::vector<void *> buffers = {input_d_.get(), out_prob_d_.get()};
  trt_common_->enqueueV2(buffers.data(), *stream_, nullptr);

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_prob_h_.get(), out_prob_d_.get(), sizeof(float) * out_elem_num_, cudaMemcpyDeviceToHost,
    *stream_));
  cudaStreamSynchronize(*stream_);
  objects.clear();

  for (int i = 0; i < batch_size; ++i) {
    auto image_size = image.size();
    float * batch_prob = out_prob_h_.get() + (i * out_elem_num_per_batch_);
    ObjectArray object_array;
    decodeOutputs(batch_prob, object_array, scales_[i], image_size);
    objects.emplace_back(object_array);
  }
  return true;
}

void TrtYoloX::decodeOutputs(
  float * prob, ObjectArray & objects, float scale, cv::Size & img_size) const
{
  ObjectArray proposals;
  auto input_dims = trt_common_->getBindingDimensions(0);
  const float input_height = static_cast<float>(input_dims.d[2]);
  const float input_width = static_cast<float>(input_dims.d[3]);
  std::vector<GridAndStride> grid_strides;
  generateGridsAndStride(input_width, input_height, m_output_strides, grid_strides);
  generateYoloxProposals(grid_strides, prob, score_threshold_, proposals);

  qsortDescentInplace(proposals);

  std::vector<int> picked;
  nmsSortedBboxes(proposals, picked, nms_threshold_);

  int count = static_cast<int>(picked.size());
  objects.resize(count);
  float scale_x = input_width / static_cast<float>(img_size.width);
  float scale_y = input_height / static_cast<float>(img_size.height);
  for (int i = 0; i < count; i++) {
    objects[i] = proposals[picked[i]];
    float x0, y0, x1, y1;
    // adjust offset to original unpadded
    if (scale == -1.0) {
      x0 = (objects[i].x_offset) / scale_x;
      y0 = (objects[i].y_offset) / scale_y;
      x1 = (objects[i].x_offset + objects[i].width) / scale_x;
      y1 = (objects[i].y_offset + objects[i].height) / scale_y;
    } else {
      x0 = (objects[i].x_offset) / scale;
      y0 = (objects[i].y_offset) / scale;
      x1 = (objects[i].x_offset + objects[i].width) / scale;
      y1 = (objects[i].y_offset + objects[i].height) / scale;
    }
    // clip
    x0 = std::clamp(x0, 0.f, static_cast<float>(img_size.width - 1));
    y0 = std::clamp(y0, 0.f, static_cast<float>(img_size.height - 1));
    x1 = std::clamp(x1, 0.f, static_cast<float>(img_size.width - 1));
    y1 = std::clamp(y1, 0.f, static_cast<float>(img_size.height - 1));

    objects[i].x_offset = x0;
    objects[i].y_offset = y0;
    objects[i].width = x1 - x0;
    objects[i].height = y1 - y0;
  }
}

void TrtYoloX::generateGridsAndStride(
  const int target_w, const int target_h, std::vector<int> strides,
  std::vector<GridAndStride> & grid_strides) const
{
  for (auto stride : strides) {
    int num_grid_w = target_w / stride;
    int num_grid_h = target_h / stride;
    for (int g1 = 0; g1 < num_grid_h; g1++) {
      for (int g0 = 0; g0 < num_grid_w; g0++) {
        grid_strides.push_back(GridAndStride{g0, g1, stride});
      }
    }
  }
}

void TrtYoloX::generateYoloxProposals(
  std::vector<GridAndStride> grid_strides, float * feat_blob, float prob_threshold,
  ObjectArray & objects) const
{
  const int num_anchors = grid_strides.size();

  for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++) {
    const int grid0 = grid_strides[anchor_idx].grid0;
    const int grid1 = grid_strides[anchor_idx].grid1;
    const int stride = grid_strides[anchor_idx].stride;

    const int basic_pos = anchor_idx * (num_class_ + 5);

    // yolox/models/yolo_head.py decode logic
    // To apply this logic, YOLOX head must output raw value
    // (i.e., `decode_in_inference` should be False)
    float x_center = (feat_blob[basic_pos + 0] + grid0) * stride;
    float y_center = (feat_blob[basic_pos + 1] + grid1) * stride;
    /*
      //exp is complex for embedded processors
    float w = exp(feat_blob[basic_pos + 2]) * stride;
    float h = exp(feat_blob[basic_pos + 3]) * stride;
    float x0 = x_center - w * 0.5f;
    float y0 = y_center - h * 0.5f;
    */
    float box_objectness = feat_blob[basic_pos + 4];
    for (int class_idx = 0; class_idx < num_class_; class_idx++) {
      float box_cls_score = feat_blob[basic_pos + 5 + class_idx];
      float box_prob = box_objectness * box_cls_score;
      if (box_prob > prob_threshold) {
        Object obj;
        // On-demand applying for exp
        float w = exp(feat_blob[basic_pos + 2]) * stride;
        float h = exp(feat_blob[basic_pos + 3]) * stride;
        float x0 = x_center - w * 0.5f;
        float y0 = y_center - h * 0.5f;
        obj.x_offset = x0;
        obj.y_offset = y0;
        obj.height = h;
        obj.width = w;
        obj.type = class_idx;
        obj.score = box_prob;

        objects.push_back(obj);
      }
    }  // class loop
  }    // point anchor loop
}

void TrtYoloX::qsortDescentInplace(ObjectArray & faceobjects, int left, int right) const
{
  int i = left;
  int j = right;
  float p = faceobjects[(left + right) / 2].score;

  while (i <= j) {
    while (faceobjects[i].score > p) {
      i++;
    }

    while (faceobjects[j].score < p) {
      j--;
    }

    if (i <= j) {
      // swap
      std::swap(faceobjects[i], faceobjects[j]);

      i++;
      j--;
    }
  }

#pragma omp parallel sections
  {
#pragma omp section
    {
      if (left < j) {
        qsortDescentInplace(faceobjects, left, j);
      }
    }
#pragma omp section
    {
      if (i < right) {
        qsortDescentInplace(faceobjects, i, right);
      }
    }
  }
}

void TrtYoloX::nmsSortedBboxes(
  const ObjectArray & faceobjects, std::vector<int> & picked, float nms_threshold) const
{
  picked.clear();
  const int n = faceobjects.size();

  std::vector<float> areas(n);
  for (int i = 0; i < n; i++) {
    cv::Rect rect(
      faceobjects[i].x_offset, faceobjects[i].y_offset, faceobjects[i].width,
      faceobjects[i].height);
    areas[i] = rect.area();
  }

  for (int i = 0; i < n; i++) {
    const Object & a = faceobjects[i];

    int keep = 1;
    for (int j = 0; j < static_cast<int>(picked.size()); j++) {
      const Object & b = faceobjects[picked[j]];

      // intersection over union
      float inter_area = intersectionArea(a, b);
      float union_area = areas[i] + areas[picked[j]] - inter_area;
      // float IoU = inter_area / union_area
      if (inter_area / union_area > nms_threshold) {
        keep = 0;
      }
    }

    if (keep) {
      picked.push_back(i);
    }
  }
}

}  // namespace tensorrt_yolox
