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
#ifndef __TRT_NET_H_
#define __TRT_NET_H_

#include <algorithm>
#include <fstream>
#include <numeric>
#include <string>
#include <vector>
#include "NvCaffeParser.h"
#include "NvInferPlugin.h"
#include "PluginFactory.h"
#include "Utils.h"

namespace Tn
{
enum class RUN_MODE { FLOAT32 = 0, FLOAT16 = 1, INT8 = 2 };

class trtNet
{
public:
  // Load from caffe model
  trtNet(
    const std::string & prototxt, const std::string & caffeModel,
    const std::vector<std::string> & outputNodesName,
    const std::vector<std::vector<float>> & calibratorData, RUN_MODE mode = RUN_MODE::FLOAT32);

  // Load from engine file
  explicit trtNet(const std::string & engineFile);

  ~trtNet()
  {
    // Release the stream and the buffers
    cudaStreamSynchronize(mTrtCudaStream);
    cudaStreamDestroy(mTrtCudaStream);
    for (auto & item : mTrtCudaBuffer) cudaFree(item);

    mTrtPluginFactory.destroyPlugin();

    if (!mTrtRunTime) mTrtRunTime->destroy();
    if (!mTrtContext) mTrtContext->destroy();
    if (!mTrtEngine) mTrtEngine->destroy();
  };

  void saveEngine(std::string fileName)
  {
    if (mTrtEngine) {
      nvinfer1::IHostMemory * data = mTrtEngine->serialize();
      std::ofstream file;
      file.open(fileName, std::ios::binary | std::ios::out);
      if (!file.is_open()) {
        std::cout << "read create engine file" << fileName << " failed" << std::endl;
        return;
      }

      file.write((const char *)data->data(), data->size());
      file.close();
    }
  };

  void doInference(const void * inputData, void * outputData);

  inline size_t getInputSize()
  {
    return std::accumulate(
      mTrtBindBufferSize.begin(), mTrtBindBufferSize.begin() + mTrtInputCount, 0);
  };

  inline size_t getOutputSize()
  {
    return std::accumulate(
      mTrtBindBufferSize.begin() + mTrtInputCount, mTrtBindBufferSize.end(), 0);
  };

  void printTime() { mTrtProfiler.printLayerTimes(mTrtIterationTime); }

private:
  nvinfer1::ICudaEngine * loadModelAndCreateEngine(
    const char * deployFile, const char * modelFile, int maxBatchSize,
    nvcaffeparser1::ICaffeParser * parser, nvcaffeparser1::IPluginFactory * pluginFactory,
    nvinfer1::IInt8Calibrator * calibrator, nvinfer1::IHostMemory *& trtModelStream,
    const std::vector<std::string> & outputNodesName);

  void InitEngine();

  nvinfer1::IExecutionContext * mTrtContext;
  nvinfer1::ICudaEngine * mTrtEngine;
  nvinfer1::IRuntime * mTrtRunTime;
  PluginFactory mTrtPluginFactory;
  cudaStream_t mTrtCudaStream;
  Profiler mTrtProfiler;
  RUN_MODE mTrtRunMode;

  std::vector<void *> mTrtCudaBuffer;
  std::vector<int64_t> mTrtBindBufferSize;
  int mTrtInputCount;
  int mTrtIterationTime;
};
}  // namespace Tn

#endif  //__TRT_NET_H_
