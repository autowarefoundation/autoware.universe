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

#include "autoware/tensorrt_bevdet/alignbev_plugin.hpp"
#include "autoware/tensorrt_bevdet/common.hpp"

#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

static inline __device__ bool within_bounds_2d(int h, int w, int H, int W)
{
  return h >= 0 && h < H && w >= 0 && w < W;
}

template <typename T1, typename T2>
__global__ void align_bev_kernel(
  const int nthreads, const T1 * input, const float * trans, T2 * output, TensorDesc output_desc)
{
  int C = output_desc.shape[1];        // 80
  int out_H = output_desc.shape[2];    // 128
  int out_W = output_desc.shape[3];    // 128
  int out_sN = output_desc.stride[0];  // 80 * 128 * 128
  int out_sC = output_desc.stride[1];  // 128 * 128
  int out_sH = output_desc.stride[2];  // 128
  int out_sW = output_desc.stride[3];  // 1

  CUDA_1D_KERNEL_LOOP(index, nthreads)
  {
    const int w = index % out_W;            //  j
    const int h = (index / out_W) % out_H;  //  i
    const int n = index / (out_H * out_W);  // batch

    float ix =
      trans[n * 6 + 0 * 3 + 0] * w + trans[n * 6 + 0 * 3 + 1] * h + trans[n * 6 + 0 * 3 + 2];
    float iy =
      trans[n * 6 + 1 * 3 + 0] * w + trans[n * 6 + 1 * 3 + 1] * h + trans[n * 6 + 1 * 3 + 2];

    // NE, NW, SE, SW point
    int ix_nw = static_cast<int>(::floor(ix));
    int iy_nw = static_cast<int>(::floor(iy));
    int ix_ne = ix_nw + 1;
    int iy_ne = iy_nw;
    int ix_sw = ix_nw;
    int iy_sw = iy_nw + 1;
    int ix_se = ix_nw + 1;
    int iy_se = iy_nw + 1;

    T2 nw = (ix_se - ix) * (iy_se - iy);
    T2 ne = (ix - ix_sw) * (iy_sw - iy);
    T2 sw = (ix_ne - ix) * (iy - iy_ne);
    T2 se = (ix - ix_nw) * (iy - iy_nw);

    // bilinear
    auto inp_ptr_NC = input + n * out_sN;
    auto out_ptr_NCHW = output + n * out_sN + h * out_sH + w * out_sW;
    for (int c = 0; c < C; ++c, inp_ptr_NC += out_sC, out_ptr_NCHW += out_sC) {
      *out_ptr_NCHW = static_cast<T2>(0);
      if (within_bounds_2d(iy_nw, ix_nw, out_H, out_W)) {
        *out_ptr_NCHW += static_cast<T2>(inp_ptr_NC[iy_nw * out_sH + ix_nw * out_sW]) * nw;
      }
      if (within_bounds_2d(iy_ne, ix_ne, out_H, out_W)) {
        *out_ptr_NCHW += static_cast<T2>(inp_ptr_NC[iy_ne * out_sH + ix_ne * out_sW]) * ne;
      }
      if (within_bounds_2d(iy_sw, ix_sw, out_H, out_W)) {
        *out_ptr_NCHW += static_cast<T2>(inp_ptr_NC[iy_sw * out_sH + ix_sw * out_sW]) * sw;
      }
      if (within_bounds_2d(iy_se, ix_se, out_H, out_W)) {
        *out_ptr_NCHW += static_cast<T2>(inp_ptr_NC[iy_se * out_sH + ix_se * out_sW]) * se;
      }
    }
  }
}

void create_desc(const int * dims, int nb_dims, TensorDesc & desc)
{
  memcpy(&desc.shape[0], dims, sizeof(int) * nb_dims);
  desc.stride[nb_dims - 1] = 1;
  for (int i = nb_dims - 2; i >= 0; --i) {
    desc.stride[i] = desc.stride[i + 1] * desc.shape[i + 1];
  }
}

namespace nvinfer1
{
// class AlignBEVPlugin
AlignBEVPlugin::AlignBEVPlugin(const std::string & name) : name_(name)
{
}

AlignBEVPlugin::AlignBEVPlugin(const std::string & name, const void * buffer, size_t length)
: name_(name)
{
  memcpy(&m_, buffer, sizeof(m_));
}

AlignBEVPlugin::~AlignBEVPlugin()
{
}

IPluginV2DynamicExt * AlignBEVPlugin::clone() const noexcept
{
  auto p = new AlignBEVPlugin(name_, &m_, sizeof(m_));
  p->setPluginNamespace(namespace_.c_str());
  return p;
}

int32_t AlignBEVPlugin::getNbOutputs() const noexcept
{
  return 1;
}

DataType AlignBEVPlugin::getOutputDataType(
  int32_t index, DataType const * inputTypes, int32_t nbInputs) const noexcept
{
  return DataType::kFLOAT;  // FIXME
}

DimsExprs AlignBEVPlugin::getOutputDimensions(
  int32_t outputIndex, const DimsExprs * inputs, int32_t nbInputs,
  IExprBuilder & exprBuilder) noexcept
{
  return inputs[0];
}

bool AlignBEVPlugin::supportsFormatCombination(
  int32_t pos, const PluginTensorDesc * inOut, int32_t nbInputs, int32_t nbOutputs) noexcept
{
  // adj_feat
  if (pos == 0) {
    return (inOut[pos].type == DataType::kFLOAT || inOut[pos].type == DataType::kHALF) &&
           inOut[pos].format == TensorFormat::kLINEAR;
  } else if (pos == 2) {  // out
    return (inOut[pos].type == DataType::kFLOAT || inOut[pos].type == DataType::kHALF) &&
           inOut[pos].format == TensorFormat::kLINEAR;
  } else if (pos == 1) {  // transform
    return inOut[pos].type == DataType::kFLOAT && inOut[pos].format == TensorFormat::kLINEAR;
  }
  return false;
}

size_t AlignBEVPlugin::getWorkspaceSize(
  const PluginTensorDesc * inputs, int32_t nbInputs, const PluginTensorDesc * outputs,
  int32_t nbOutputs) const noexcept
{
  return 0;
}

int32_t AlignBEVPlugin::enqueue(
  const PluginTensorDesc * inputDesc, const PluginTensorDesc * outputDesc,
  const void * const * inputs, void * const * outputs, void * workspace,
  cudaStream_t stream) noexcept
{
  // TODO
  // inputs[0] == adj_feat  b x 8 x 80 x 128 x 128
  // inputs[1] == transform b x 8 x 6

  int bev_channel = inputDesc[0].dims.d[2];
  int bev_h = inputDesc[0].dims.d[3];
  int bev_w = inputDesc[0].dims.d[4];
  int adj_num = inputDesc[0].dims.d[1];

  int output_dim[4] = {8, bev_channel, bev_h, bev_w};

  TensorDesc output_desc;
  create_desc(output_dim, 4, output_desc);

  int count = 1;
  for (int i = 0; i < 4; ++i) {
    if (i == 1) {
      continue;
    }
    count *= output_desc.shape[i];
  }

  switch (int(outputDesc[0].type)) {
    case int(DataType::kFLOAT):
      if (inputDesc[0].type == DataType::kFLOAT) {
        // align : fp32, fp32;
        align_bev_kernel<float, float><<<GET_BLOCKS(count), NUM_THREADS, 0, stream>>>(
          count, reinterpret_cast<const float *>(inputs[0]),
          reinterpret_cast<const float *>(inputs[1]), reinterpret_cast<float *>(outputs[0]),
          output_desc);
      } else {
        // align : fp16, fp32
        align_bev_kernel<__half, float><<<GET_BLOCKS(count), NUM_THREADS, 0, stream>>>(
          count, reinterpret_cast<const __half *>(inputs[0]),
          reinterpret_cast<const float *>(inputs[1]), reinterpret_cast<float *>(outputs[0]),
          output_desc);
      }
      break;
    case int(DataType::kHALF):
      if (inputDesc[0].type == DataType::kFLOAT) {
        // align : fp32, fp16
        align_bev_kernel<float, __half><<<GET_BLOCKS(count), NUM_THREADS, 0, stream>>>(
          count, reinterpret_cast<const float *>(inputs[0]),
          reinterpret_cast<const float *>(inputs[1]), reinterpret_cast<__half *>(outputs[0]),
          output_desc);
      } else {
        // align : fp16, fp16
        align_bev_kernel<__half, __half><<<GET_BLOCKS(count), NUM_THREADS, 0, stream>>>(
          count, reinterpret_cast<const __half *>(inputs[0]),
          reinterpret_cast<const float *>(inputs[1]), reinterpret_cast<__half *>(outputs[0]),
          output_desc);
      }
      break;
    default:  // should NOT be here
      RCLCPP_ERROR(
        rclcpp::get_logger("AlignBEVPlugin"), 
        "\tUnsupported datatype!"
      );
  }

  return 0;
}

void AlignBEVPlugin::destroy() noexcept
{
  delete this;
  return;
}

void AlignBEVPlugin::configurePlugin(
  const DynamicPluginTensorDesc * in, int32_t nbInputs, const DynamicPluginTensorDesc * out,
  int32_t nbOutputs) noexcept
{
  return;
}

int32_t AlignBEVPlugin::initialize() noexcept
{
  return 0;
}

void AlignBEVPlugin::terminate() noexcept
{
  return;
}

size_t AlignBEVPlugin::getSerializationSize() const noexcept
{
  return sizeof(m_);
}

void AlignBEVPlugin::serialize(void * buffer) const noexcept
{
  memcpy(buffer, &m_, sizeof(m_));
  return;
}

void AlignBEVPlugin::setPluginNamespace(const char * pluginNamespace) noexcept
{
  namespace_ = pluginNamespace;
  return;
}

const char * AlignBEVPlugin::getPluginNamespace() const noexcept
{
  return namespace_.c_str();
}

const char * AlignBEVPlugin::getPluginType() const noexcept
{
  return ALIGN_PLUGIN_NAME;
}

const char * AlignBEVPlugin::getPluginVersion() const noexcept
{
  return ALIGN_PLUGIN_VERSION;
}

void AlignBEVPlugin::attachToContext(
  cudnnContext * contextCudnn, cublasContext * contextCublas, IGpuAllocator * gpuAllocator) noexcept
{
  return;
}

void AlignBEVPlugin::detachFromContext() noexcept
{
  return;
}

// class AlignBEVPluginCreator
PluginFieldCollection AlignBEVPluginCreator::fc_{};
std::vector<PluginField> AlignBEVPluginCreator::attr_;

AlignBEVPluginCreator::AlignBEVPluginCreator()
{
  attr_.clear();
  fc_.nbFields = attr_.size();
  fc_.fields = attr_.data();
}

AlignBEVPluginCreator::~AlignBEVPluginCreator()
{
}

IPluginV2DynamicExt * AlignBEVPluginCreator::createPlugin(
  const char * name, const PluginFieldCollection * fc) noexcept
{
  // const PluginField *fields = fc->fields;
  AlignBEVPlugin * pObj = new AlignBEVPlugin(name);
  pObj->setPluginNamespace(namespace_.c_str());
  return pObj;
}

IPluginV2DynamicExt * AlignBEVPluginCreator::deserializePlugin(
  const char * name, const void * serialData, size_t serialLength) noexcept
{
  AlignBEVPlugin * pObj = new AlignBEVPlugin(name, serialData, serialLength);
  pObj->setPluginNamespace(namespace_.c_str());
  return pObj;
}

void AlignBEVPluginCreator::setPluginNamespace(const char * pluginNamespace) noexcept
{
  namespace_ = pluginNamespace;
  return;
}

const char * AlignBEVPluginCreator::getPluginNamespace() const noexcept
{
  return namespace_.c_str();
}

const char * AlignBEVPluginCreator::getPluginName() const noexcept
{
  return ALIGN_PLUGIN_NAME;
}

const char * AlignBEVPluginCreator::getPluginVersion() const noexcept
{
  return ALIGN_PLUGIN_VERSION;
}

const PluginFieldCollection * AlignBEVPluginCreator::getFieldNames() noexcept
{
  return &fc_;
}

REGISTER_TENSORRT_PLUGIN(AlignBEVPluginCreator);

}  // namespace nvinfer1
