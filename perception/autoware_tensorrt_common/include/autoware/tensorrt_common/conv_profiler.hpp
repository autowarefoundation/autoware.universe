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

#ifndef AUTOWARE__TENSORRT_COMMON__CONV_PROFILER_HPP_
#define AUTOWARE__TENSORRT_COMMON__CONV_PROFILER_HPP_

#include <autoware/tensorrt_common/profiler.hpp>

#include <NvInfer.h>

#include <map>
#include <string>
#include <vector>

namespace autoware
{
namespace tensorrt_common
{
/**
 * @struct LayerInfo
 * @brief Information of a layer.
 */
struct LayerInfo
{
  //! @brief Input channel.
  int in_c;
  //! @brief Output channel.
  int out_c;
  //! @brief Width.
  int w;
  //! @brief Height.
  int h;
  //! @brief Kernel size.
  int k;
  //! @brief Stride.
  int stride;
  //! @brief Number of groups.
  int groups;
  //! @brief Layer type.
  nvinfer1::LayerType type;
};

/**
 * @class ConvProfiler
 * @brief Collect per-layer profile information, assuming times are reported in the same order.
 */
class ConvProfiler : public tensorrt_common::Profiler
{
public:
  /**
   * @brief Construct Profiler for convolutional layers.
   *
   * @param[in] src_profilers Source profilers to merge.
   */
  explicit ConvProfiler(
    const std::vector<Profiler> & src_profilers = std::vector<tensorrt_common::Profiler>())
  : Profiler(src_profilers)
  {
  }

  /**
   * @brief Set per-layer profile information for model.
   *
   * @param[in] layer Layer to set profile information.
   */
  void setProfDict(nvinfer1::ILayer * const layer) noexcept final
  {
    std::string name = layer->getName();
    layer_dict_[name];
    layer_dict_[name].type = layer->getType();
    if (layer->getType() == nvinfer1::LayerType::kCONVOLUTION) {
      auto conv = dynamic_cast<nvinfer1::IConvolutionLayer *>(layer);
      nvinfer1::ITensor * in = layer->getInput(0);
      nvinfer1::Dims dim_in = in->getDimensions();
      nvinfer1::ITensor * out = layer->getOutput(0);
      nvinfer1::Dims dim_out = out->getDimensions();
      nvinfer1::Dims k_dims = conv->getKernelSizeNd();
      nvinfer1::Dims s_dims = conv->getStrideNd();
      int groups = conv->getNbGroups();
      int stride = s_dims.d[0];
      int kernel = k_dims.d[0];
      layer_dict_[name].in_c = dim_in.d[1];
      layer_dict_[name].out_c = dim_out.d[1];
      layer_dict_[name].w = dim_in.d[3];
      layer_dict_[name].h = dim_in.d[2];
      layer_dict_[name].k = kernel;
      layer_dict_[name].stride = stride;
      layer_dict_[name].groups = groups;
    }
  }

private:
  //! @brief Per-layer information.
  std::map<std::string, LayerInfo> layer_dict_;
};
}  // namespace tensorrt_common
}  // namespace autoware
#endif  // AUTOWARE__TENSORRT_COMMON__CONV_PROFILER_HPP_
