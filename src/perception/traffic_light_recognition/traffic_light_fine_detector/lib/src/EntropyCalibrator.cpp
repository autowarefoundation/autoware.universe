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
#include "EntropyCalibrator.h"
#include <string.h>
#include <algorithm>
#include <cassert>
#include <fstream>
#include <iterator>

namespace nvinfer1
{
Int8EntropyCalibrator::Int8EntropyCalibrator(
  int BatchSize, const std::vector<std::vector<float>> & data, const std::string & CalibDataName,
  bool readCache)
: mCalibDataName(CalibDataName), mBatchSize(BatchSize), mReadCache(readCache)
{
  mDatas.reserve(data.size());
  mDatas = data;
  mInputCount = BatchSize * data[0].size();
  mCurBatchData = new float[mInputCount];
  mCurBatchIdx = 0;
  CUDA_CHECK(cudaMalloc(&mDeviceInput, mInputCount * sizeof(float)));
}

Int8EntropyCalibrator::~Int8EntropyCalibrator()
{
  CUDA_CHECK(cudaFree(mDeviceInput));
  if (mCurBatchData) delete[] mCurBatchData;
}

bool Int8EntropyCalibrator::getBatch(void * bindings[], const char * names[], int nbBindings)
{
  if (mCurBatchIdx + mBatchSize > int(mDatas.size())) return false;

  float * ptr = mCurBatchData;
  size_t imgSize = mInputCount / mBatchSize;
  auto iter = mDatas.begin() + mCurBatchIdx;

  std::for_each(iter, iter + mBatchSize, [=, &ptr](std::vector<float> & val) {
    assert(imgSize == val.size());
    memcpy(ptr, val.data(), imgSize * sizeof(float));

    ptr += imgSize;
  });

  CUDA_CHECK(
    cudaMemcpy(mDeviceInput, mCurBatchData, mInputCount * sizeof(float), cudaMemcpyHostToDevice));
  bindings[0] = mDeviceInput;

  std::cout << "load batch " << mCurBatchIdx << " to " << mCurBatchIdx + mBatchSize - 1
            << std::endl;
  mCurBatchIdx += mBatchSize;
  return true;
}

const void * Int8EntropyCalibrator::readCalibrationCache(size_t & length)
{
  mCalibrationCache.clear();
  std::ifstream input(mCalibDataName + ".calib", std::ios::binary);
  input >> std::noskipws;
  if (mReadCache && input.good())
    std::copy(
      std::istream_iterator<char>(input), std::istream_iterator<char>(),
      std::back_inserter(mCalibrationCache));

  length = mCalibrationCache.size();
  return length ? &mCalibrationCache[0] : nullptr;
}

void Int8EntropyCalibrator::writeCalibrationCache(const void * cache, size_t length)
{
  std::ofstream output(mCalibDataName + ".calib", std::ios::binary);
  output.write(reinterpret_cast<const char *>(cache), length);
}
}  // namespace nvinfer1