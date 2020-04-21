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
#ifndef _ENTROPY_CALIBRATOR_H
#define _ENTROPY_CALIBRATOR_H

#include <cudnn.h>
#include <string>
#include <vector>
#include "NvInfer.h"
#include "Utils.h"

namespace nvinfer1
{
class Int8EntropyCalibrator : public IInt8EntropyCalibrator2
{
public:
  Int8EntropyCalibrator(
    int BatchSize, const std::vector<std::vector<float>> & data, const std::string & CalibDataName,
    bool readCache);

  virtual ~Int8EntropyCalibrator();

  int getBatchSize() const override { return mBatchSize; }

  bool getBatch(void * bindings[], const char * names[], int nbBindings) override;

  const void * readCalibrationCache(size_t & length) override;

  void writeCalibrationCache(const void * cache, size_t length) override;

private:
  std::string mCalibDataName;
  std::vector<std::vector<float>> mDatas;
  int mBatchSize;

  int mCurBatchIdx;
  float * mCurBatchData{nullptr};

  size_t mInputCount;
  bool mReadCache;
  void * mDeviceInput{nullptr};

  std::vector<char> mCalibrationCache;
};

}  // namespace nvinfer1

#endif  //_ENTROPY_CALIBRATOR_H
