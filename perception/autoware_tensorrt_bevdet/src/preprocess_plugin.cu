/*
 * Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autoware/tensorrt_bevdet/common.hpp"
#include "autoware/tensorrt_bevdet/preprocess_plugin.hpp"

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <iostream>

// kernel for GPU

template <typename T>
__global__ void preprocess_kernel(
  const uint8_t * src_dev, T * dst_dev, int src_row_step, int dst_row_step, int src_img_step,
  int dst_img_step, int src_h, int src_w, float radio_h, float radio_w, float offset_h,
  float offset_w, const float * mean, const float * std, int dst_h, int dst_w, int n)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= dst_h * dst_w * n) return;

  int i = (idx / n) / dst_w;
  int j = (idx / n) % dst_w;
  int k = idx % n;

  int pX = (int)roundf((i / radio_h) + offset_h);
  int pY = (int)roundf((j / radio_w) + offset_w);

  if (pX < src_h && pX >= 0 && pY < src_w && pY >= 0) {
    int s1 = k * src_img_step + 0 * src_img_step / 3 + pX * src_row_step + pY;
    int s2 = k * src_img_step + 1 * src_img_step / 3 + pX * src_row_step + pY;
    int s3 = k * src_img_step + 2 * src_img_step / 3 + pX * src_row_step + pY;

    int d1 = k * dst_img_step + 0 * dst_img_step / 3 + i * dst_row_step + j;
    int d2 = k * dst_img_step + 1 * dst_img_step / 3 + i * dst_row_step + j;
    int d3 = k * dst_img_step + 2 * dst_img_step / 3 + i * dst_row_step + j;

    dst_dev[d1] = (static_cast<T>(src_dev[s1]) - static_cast<T>(mean[0])) / static_cast<T>(std[0]);
    dst_dev[d2] = (static_cast<T>(src_dev[s2]) - static_cast<T>(mean[1])) / static_cast<T>(std[1]);
    dst_dev[d3] = (static_cast<T>(src_dev[s3]) - static_cast<T>(mean[2])) / static_cast<T>(std[2]);
  }
}

namespace nvinfer1
{
// class PreprocessPlugin
PreprocessPlugin::PreprocessPlugin(
  const std::string & name, int crop_h, int crop_w, float resize_radio)
: name_(name)
{
  m_.crop_h = crop_h;
  m_.crop_w = crop_w;
  m_.resize_radio = resize_radio;
}

PreprocessPlugin::PreprocessPlugin(const std::string & name, const void * buffer, size_t length)
: name_(name)
{
  memcpy(&m_, buffer, sizeof(m_));
}

PreprocessPlugin::~PreprocessPlugin()
{
}

IPluginV2DynamicExt * PreprocessPlugin::clone() const noexcept
{
  auto p = new PreprocessPlugin(name_, &m_, sizeof(m_));
  p->setPluginNamespace(namespace_.c_str());
  return p;
}

int32_t PreprocessPlugin::getNbOutputs() const noexcept
{
  return 1;
}

DataType PreprocessPlugin::getOutputDataType(
  int32_t index, DataType const * inputTypes, int32_t nbInputs) const noexcept
{
  // return DataType::kHALF;
  return DataType::kFLOAT;
}

DimsExprs PreprocessPlugin::getOutputDimensions(
  int32_t outputIndex, const DimsExprs * inputs, int32_t nbInputs,
  IExprBuilder & exprBuilder) noexcept
{
  int input_h = inputs[0].d[2]->getConstantValue();
  int input_w = inputs[0].d[3]->getConstantValue() * 4;

  int output_h = input_h * m_.resize_radio - m_.crop_h;
  int output_w = input_w * m_.resize_radio - m_.crop_w;

  DimsExprs ret;
  ret.nbDims = inputs[0].nbDims;
  ret.d[0] = inputs[0].d[0];
  ret.d[1] = inputs[0].d[1];
  ret.d[2] = exprBuilder.constant(output_h);
  ret.d[3] = exprBuilder.constant(output_w);

  return ret;
}

bool PreprocessPlugin::supportsFormatCombination(
  int32_t pos, const PluginTensorDesc * inOut, int32_t nbInputs, int32_t nbOutputs) noexcept
{
  bool res;
  switch (pos) {
    case 0:  // images
      res = (inOut[0].type == DataType::kINT8 || inOut[0].type == DataType::kINT32) &&
            inOut[0].format == TensorFormat::kLINEAR;
      break;
    case 1:  // mean
      res = (inOut[1].type == DataType::kFLOAT) && inOut[1].format == TensorFormat::kLINEAR;
      break;
    case 2:  // std
      res = (inOut[2].type == DataType::kFLOAT) && inOut[2].format == TensorFormat::kLINEAR;
      break;
    case 3:  // output img tensor
      res = (inOut[3].type == DataType::kFLOAT || inOut[3].type == DataType::kHALF) &&
            inOut[3].format == inOut[0].format;

      // res = inOut[3].type == DataType::kHALF && inOut[3].format == inOut[0].format;
      break;
    default:
      res = false;
  }
  return res;
}

void PreprocessPlugin::configurePlugin(
  const DynamicPluginTensorDesc * in, int32_t nbInputs, const DynamicPluginTensorDesc * out,
  int32_t nbOutputs) noexcept
{
  return;
}

size_t PreprocessPlugin::getWorkspaceSize(
  const PluginTensorDesc * inputs, int32_t nbInputs, const PluginTensorDesc * outputs,
  int32_t nbOutputs) const noexcept
{
  return 0;
}

int32_t PreprocessPlugin::enqueue(
  const PluginTensorDesc * inputDesc, const PluginTensorDesc * outputDesc,
  const void * const * inputs, void * const * outputs, void * workspace,
  cudaStream_t stream) noexcept
{
  int n_img = inputDesc[0].dims.d[0];
  int src_img_h = inputDesc[0].dims.d[2];
  int src_img_w = inputDesc[0].dims.d[3] * 4;

  int dst_img_h = outputDesc[0].dims.d[2];
  int dst_img_w = outputDesc[0].dims.d[3];

  int src_row_step = src_img_w;
  int dst_row_step = dst_img_w;

  int src_img_step = src_img_w * src_img_h * 3;
  int dst_img_step = dst_img_w * dst_img_h * 3;

  float offset_h = m_.crop_h / m_.resize_radio;
  float offset_w = m_.crop_w / m_.resize_radio;

  dim3 grid(DIVUP(dst_img_h * dst_img_w * n_img, NUM_THREADS));
  dim3 block(NUM_THREADS);

  switch (int(outputDesc[0].type)) {
    case int(DataType::kFLOAT):
      // float
      preprocess_kernel<<<grid, block, 0, stream>>>(
        reinterpret_cast<const uint8_t *>(inputs[0]), reinterpret_cast<float *>(outputs[0]),
        src_row_step, dst_row_step, src_img_step, dst_img_step, src_img_h, src_img_w,
        m_.resize_radio, m_.resize_radio, offset_h, offset_w,
        reinterpret_cast<const float *>(inputs[1]), reinterpret_cast<const float *>(inputs[2]),
        dst_img_h, dst_img_w, n_img);
      break;
    case int(DataType::kHALF):
      // half
      preprocess_kernel<<<grid, block, 0, stream>>>(
        reinterpret_cast<const uint8_t *>(inputs[0]), reinterpret_cast<__half *>(outputs[0]),
        src_row_step, dst_row_step, src_img_step, dst_img_step, src_img_h, src_img_w,
        m_.resize_radio, m_.resize_radio, offset_h, offset_w,
        reinterpret_cast<const float *>(inputs[1]), reinterpret_cast<const float *>(inputs[2]),
        dst_img_h, dst_img_w, n_img);

      break;
    default:  // should NOT be here
      RCLCPP_ERROR(
        rclcpp::get_logger("PreprocessPlugin"),
        "\tUnsupport datatype!"
      );
  }
  return 0;
}

void PreprocessPlugin::destroy() noexcept
{
  delete this;
  return;
}

int32_t PreprocessPlugin::initialize() noexcept
{
  return 0;
}

void PreprocessPlugin::terminate() noexcept
{
  return;
}

size_t PreprocessPlugin::getSerializationSize() const noexcept
{
  return sizeof(m_);
}

void PreprocessPlugin::serialize(void * buffer) const noexcept
{
  memcpy(buffer, &m_, sizeof(m_));
  return;
}

void PreprocessPlugin::setPluginNamespace(const char * pluginNamespace) noexcept
{
  namespace_ = pluginNamespace;
  return;
}

const char * PreprocessPlugin::getPluginNamespace() const noexcept
{
  return namespace_.c_str();
}

const char * PreprocessPlugin::getPluginType() const noexcept
{
  return PRE_PLUGIN_NAME;
}

const char * PreprocessPlugin::getPluginVersion() const noexcept
{
  return PRE_PLUGIN_VERSION;
}

void PreprocessPlugin::attachToContext(
  cudnnContext * contextCudnn, cublasContext * contextCublas, IGpuAllocator * gpuAllocator) noexcept
{
  return;
}

void PreprocessPlugin::detachFromContext() noexcept
{
  return;
}

// class PreprocessPluginCreator
PluginFieldCollection PreprocessPluginCreator::fc_{};
std::vector<PluginField> PreprocessPluginCreator::attr_;

PreprocessPluginCreator::PreprocessPluginCreator()
{
  attr_.clear();
  attr_.emplace_back(PluginField("crop_h", nullptr, PluginFieldType::kINT32, 1));
  attr_.emplace_back(PluginField("crop_w", nullptr, PluginFieldType::kINT32, 1));
  attr_.emplace_back(PluginField("resize_radio", nullptr, PluginFieldType::kFLOAT32, 1));

  fc_.nbFields = attr_.size();
  fc_.fields = attr_.data();
}

PreprocessPluginCreator::~PreprocessPluginCreator()
{
}

IPluginV2DynamicExt * PreprocessPluginCreator::createPlugin(
  const char * name, const PluginFieldCollection * fc) noexcept
{
  const PluginField * fields = fc->fields;

  int crop_h = -1;
  int crop_w = -1;
  float resize_radio = 0.f;

  for (int i = 0; i < fc->nbFields; ++i) {
    if (std::string(fc->fields[i].name) == std::string("crop_h")) {
      crop_h = *reinterpret_cast<const int *>(fc->fields[i].data);
    } else if (std::string(fc->fields[i].name) == std::string("crop_w")) {
      crop_w = *reinterpret_cast<const int *>(fc->fields[i].data);
    } else if (std::string(fc->fields[i].name) == std::string("resize_radio")) {
      resize_radio = *reinterpret_cast<const float *>(fc->fields[i].data);
    }
  }
  PreprocessPlugin * pObj = new PreprocessPlugin(name, crop_h, crop_w, resize_radio);
  pObj->setPluginNamespace(namespace_.c_str());
  return pObj;
}

IPluginV2DynamicExt * PreprocessPluginCreator::deserializePlugin(
  const char * name, const void * serialData, size_t serialLength) noexcept
{
  PreprocessPlugin * pObj = new PreprocessPlugin(name, serialData, serialLength);
  pObj->setPluginNamespace(namespace_.c_str());
  return pObj;
}

void PreprocessPluginCreator::setPluginNamespace(const char * pluginNamespace) noexcept
{
  namespace_ = pluginNamespace;
  return;
}

const char * PreprocessPluginCreator::getPluginNamespace() const noexcept
{
  return namespace_.c_str();
}

const char * PreprocessPluginCreator::getPluginName() const noexcept
{
  return PRE_PLUGIN_NAME;
}

const char * PreprocessPluginCreator::getPluginVersion() const noexcept
{
  return PRE_PLUGIN_VERSION;
}

const PluginFieldCollection * PreprocessPluginCreator::getFieldNames() noexcept
{
  return &fc_;
}

REGISTER_TENSORRT_PLUGIN(PreprocessPluginCreator);

}  // namespace nvinfer1
