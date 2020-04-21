/*
 * MIT License
 * 
 * Copyright (c) 2018 lewes6369
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/
#ifndef _UPSAMPLE_LAYER_H
#define _UPSAMPLE_LAYER_H

#include <assert.h>
#include <cublas_v2.h>
#include <cudnn.h>
#include <string.h>
#include <cmath>
#include <iostream>
#include "NvInfer.h"
#include "Utils.h"

namespace nvinfer1
{
class UpsampleLayerPlugin : public IPluginExt
{
public:
  explicit UpsampleLayerPlugin(const float scale, const int cudaThread = 512);
  // create the plugin at runtime from a byte stream
  UpsampleLayerPlugin(const void * data, size_t length);

  ~UpsampleLayerPlugin();

  int getNbOutputs() const override { return 1; }

  Dims getOutputDimensions(int index, const Dims * inputs, int nbInputDims) override;

  bool supportsFormat(DataType type, PluginFormat format) const override
  {
    // std::cout << "supportsFormat=== type:"  << int(type) << "format" << int(format) << std::endl;
    return (type == DataType::kFLOAT || type == DataType::kHALF || type == DataType::kINT8) &&
           format == PluginFormat::kNCHW;
  }

  void configureWithFormat(
    const Dims * inputDims, int nbInputs, const Dims * outputDims, int nbOutputs, DataType type,
    PluginFormat format, int maxBatchSize) override;

  int initialize() override;

  virtual void terminate() override{};

  virtual size_t getWorkspaceSize(int maxBatchSize) const override { return 0; }

  virtual int enqueue(
    int batchSize, const void * const * inputs, void ** outputs, void * workspace,
    cudaStream_t stream) override;

  virtual size_t getSerializationSize() override
  {
    return sizeof(nvinfer1::Dims) + sizeof(mDataType) + sizeof(mScale) + sizeof(mOutputWidth) +
           sizeof(mOutputHeight) + sizeof(mThreadCount);
  }

  virtual void serialize(void * buffer) override;

  template <typename Dtype>
  void forwardGpu(const Dtype * input, Dtype * outputint, int N, int C, int H, int W);

private:
  nvinfer1::Dims mCHW;
  DataType mDataType{DataType::kFLOAT};
  float mScale;
  int mOutputWidth;
  int mOutputHeight;
  int mThreadCount;

  void * mInputBuffer{nullptr};
  void * mOutputBuffer{nullptr};
};
};  // namespace nvinfer1

#endif
