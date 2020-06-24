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
#include "YoloConfigs.h"
#include "YoloLayer.h"

using namespace Yolo;

namespace nvinfer1
{
    __device__ float Logist(float data){ return 1./(1. + exp(-data)); };

    __global__ void CalDetection(const float *input, float *output,int noElements, 
            int yoloWidth,int yoloHeight,const float anchors[CHECK_COUNT*2],int classes) {
 
        int idx = threadIdx.x + blockDim.x * blockIdx.x;
        if (idx >= noElements) return;

        int stride = yoloWidth*yoloHeight;

        for (int k = 0;k < CHECK_COUNT; ++k )
        {
            int beginIdx = (LOCATIONS + 1 + classes)* stride *k + idx;
            int objIndex = beginIdx + LOCATIONS*stride;
            
            //check objectness
            float objProb = Logist(input[objIndex]);   
            if(objProb <= IGNORE_THRESH)
                continue;

            int row = idx / yoloWidth;
            int cols = idx % yoloWidth;
            
            //classes
            int classId = -1;
            float maxProb = IGNORE_THRESH;
            for (int c = 0;c<classes;++c){
                float cProb =  Logist(input[beginIdx + (5 + c) * stride]) * objProb;
                if(cProb > maxProb){
                    maxProb = cProb;
                    classId = c;
                }
            }

            if(classId >= 0) {
                int resCount = (int)atomicAdd(output,1);
                char* data = (char * )output + sizeof(float) + resCount*sizeof(Detection);
                Detection* det =  (Detection*)(data);

                //Location
                det->bbox[0] = (cols + Logist(input[beginIdx]))/ yoloWidth;
                det->bbox[1] = (row + Logist(input[beginIdx+stride]))/ yoloHeight;
                det->bbox[2] = exp(input[beginIdx+2*stride]) * anchors[2*k];
                det->bbox[3] = exp(input[beginIdx+3*stride]) * anchors[2*k + 1];
                det->classId = classId;
                det->prob = maxProb;
            }
        }
    }
   
    void YoloLayerPlugin::forwardGpu(const float *const * inputs,float * output,cudaStream_t stream) {
        int numElem;
        void* devAnchor;
        size_t AnchorLen = sizeof(float)* CHECK_COUNT*2;
        CUDA_CHECK(cudaMalloc(&devAnchor,AnchorLen));

        //first detect count init 0
        CUDA_CHECK(cudaMemset(output, 0, sizeof(float)));
        for (unsigned int i = 0;i< mYoloKernel.size();++i)
        {
            const auto& yolo = mYoloKernel[i];
            numElem = yolo.width*yolo.height;

            //copy anchor to device
	        CUDA_CHECK(cudaMemcpy(devAnchor,yolo.anchors,AnchorLen,cudaMemcpyHostToDevice));

            CalDetection<<< (yolo.width*yolo.height + mThreadCount - 1) / mThreadCount, mThreadCount>>>
                    (inputs[i],output, numElem, yolo.width, yolo.height, (float *)devAnchor, mClassCount);
        }
        CUDA_CHECK(cudaFree(devAnchor));
    }


    int YoloLayerPlugin::enqueue(int batchSize, const void*const * inputs, void** outputs, void* workspace, cudaStream_t stream)
    {
        assert(batchSize == 1);
        
        //GPU
        forwardGpu((const float *const *)inputs,(float *)outputs[0],stream);

        //CPU
        //forwardCpu((const float *const *)inputs,(float *)outputs[0],stream);
        return 0;
    };
}
