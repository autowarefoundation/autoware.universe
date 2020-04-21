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
#include "UpsampleLayer.h"

namespace nvinfer1
{
  __device__ int translate_idx(int ii, int d1, int d2, int d3, int scale_factor) {
    int x, y, z, w;
    w = ii % d3;
    ii = ii/d3;
    z = ii % d2;
    ii = ii/d2;
    y = ii % d1;
    ii = ii/d1;
    x = ii;
    w = w/scale_factor;
    z = z/scale_factor;
    d2 /= scale_factor;
    d3 /= scale_factor;
    return (((x*d1+y)*d2)+z)*d3+w;
  }

  template <typename Dtype>
  __global__ void upscale(const Dtype *input, Dtype *output,
          int no_elements, int scale_factor, int d1, int d2, int d3) {
    int ii = threadIdx.x + blockDim.x * blockIdx.x;
    if (ii >= no_elements) return;
    int ipidx = translate_idx(ii, d1, d2, d3, scale_factor);
    output[ii]=input[ipidx];
  }

  template <typename Dtype>
  void UpsampleLayerPlugin::forwardGpu(const Dtype* input,Dtype * output,
      int N,int C,int H ,int W) {

    int numElem = N*C*H*W;
    upscale<<<(numElem + mThreadCount - 1) / mThreadCount, mThreadCount>>>(input,output, numElem, mScale, C, H, W);
  }
  
  size_t type2size(DataType dataType) { 
    size_t _size = 0;
    switch (dataType)
    {
        case DataType::kFLOAT: _size = sizeof(float);break;
        case DataType::kHALF: _size = sizeof(__half);break;
        case DataType::kINT8: _size = sizeof(u_int8_t);break;
        default:std::cerr << "error data type" << std::endl;
    }
    return _size;
  }

  int UpsampleLayerPlugin::enqueue(int batchSize, const void*const * inputs, void** outputs, void* workspace, cudaStream_t stream)
  {
      assert(batchSize == 1);
      const int channels = mCHW.d[0];
      const int64_t in_height = mCHW.d[1];
      const int64_t in_width = mCHW.d[2];
      const int64_t out_height = mOutputHeight;
      const int64_t out_width = mOutputWidth;
      int totalElems = batchSize * in_height * in_width * channels;
      
      // Handle no-op resizes efficiently.
      if (out_height == in_height && out_width == in_width) {
          CUDA_CHECK(cudaMemcpyAsync(outputs[0], inputs[0], totalElems * type2size(mDataType), cudaMemcpyDeviceToDevice, stream));
          CUDA_CHECK(cudaStreamSynchronize(stream));
          return 0;
      }
      //CUDA_CHECK(cudaStreamSynchronize(stream));
      
       switch (mDataType)
       {
           case DataType::kFLOAT :
              forwardGpu<float>((const float *)inputs[0],(float *)outputs[0],batchSize,mCHW.d[0],mOutputHeight,mOutputWidth);
               break;
           case DataType::kHALF:
               forwardGpu<__half>((const __half *)inputs[0],(__half *)outputs[0],batchSize,mCHW.d[0],mOutputHeight,mOutputWidth);
               break;
           case DataType::kINT8:
               forwardGpu<u_int8_t>((const u_int8_t *)inputs[0],(u_int8_t *)outputs[0],batchSize,mCHW.d[0],mOutputHeight,mOutputWidth);
              break;
           default:
               std::cerr << "error data type" << std::endl;
       }
      return 0;    
  };
}