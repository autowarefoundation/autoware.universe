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
    YoloLayerPlugin::YoloLayerPlugin(const int cudaThread /*= 512*/):mThreadCount(cudaThread)
    {
        mClassCount = CLASS_NUM;
        mYoloKernel.clear();
        mYoloKernel.push_back(yolo1);
        mYoloKernel.push_back(yolo2);
        mYoloKernel.push_back(yolo3);

        mKernelCount = mYoloKernel.size();
    }
    
    YoloLayerPlugin::~YoloLayerPlugin()
    {
        if(mInputBuffer)
            CUDA_CHECK(cudaFreeHost(mInputBuffer));

        if(mOutputBuffer)
            CUDA_CHECK(cudaFreeHost(mOutputBuffer));
    }
    
    // create the plugin at runtime from a byte stream
    YoloLayerPlugin::YoloLayerPlugin(const void* data, size_t length)
    {
        using namespace Tn;
        const char *d = reinterpret_cast<const char *>(data), *a = d;
        read(d, mClassCount);
        read(d, mThreadCount);
        read(d, mKernelCount);
        mYoloKernel.resize(mKernelCount);
        auto kernelSize = mKernelCount*sizeof(YoloKernel);
        memcpy(mYoloKernel.data(),d,kernelSize);
        d += kernelSize;

        assert(d == a + length);
    }

    void YoloLayerPlugin::serialize(void* buffer)
    {
        using namespace Tn;
        char* d = static_cast<char*>(buffer), *a = d;
        write(d, mClassCount);
        write(d, mThreadCount);
        write(d, mKernelCount);
        auto kernelSize = mKernelCount*sizeof(YoloKernel);
        memcpy(d,mYoloKernel.data(),kernelSize);
        d += kernelSize;

        assert(d == a + getSerializationSize());
    }
    
    size_t YoloLayerPlugin::getSerializationSize()
    {  
        return sizeof(mClassCount) + sizeof(mThreadCount) + sizeof(mKernelCount) + sizeof(Yolo::YoloKernel) * mYoloKernel.size();
    }

    int YoloLayerPlugin::initialize()
    { 
            int totalCount = 0;
            for(const auto& yolo : mYoloKernel)
                totalCount += (LOCATIONS + 1 + mClassCount) * yolo.width*yolo.height * CHECK_COUNT;
            CUDA_CHECK(cudaHostAlloc(&mInputBuffer, totalCount * sizeof(float), cudaHostAllocDefault));

            totalCount = 0;//detection count
            for(const auto& yolo : mYoloKernel)
                totalCount += yolo.width*yolo.height * CHECK_COUNT;
            CUDA_CHECK(cudaHostAlloc(&mOutputBuffer, sizeof(float) + totalCount * sizeof(Detection), cudaHostAllocDefault));
            return 0;
    }
    
    Dims YoloLayerPlugin::getOutputDimensions(int index, const Dims* inputs, int nbInputDims)
    {
            //output the result to channel
            int totalCount = 0;
            for(const auto& yolo : mYoloKernel)
                totalCount += yolo.width*yolo.height * CHECK_COUNT * sizeof(Detection) / sizeof(float);

            return Dims3(totalCount + 1, 1, 1);
    }

    void YoloLayerPlugin::forwardCpu(const float*const * inputs, float* outputs, cudaStream_t stream)
    {
            auto Logist = [=](float data){
                return 1./(1. + exp(-data));
            };

            CUDA_CHECK(cudaStreamSynchronize(stream));
            int i = 0;
            float* inputData = (float *)mInputBuffer; 
            for(const auto& yolo : mYoloKernel)
            {
                int size = (LOCATIONS + 1 + mClassCount) * yolo.width*yolo.height * CHECK_COUNT;
                CUDA_CHECK(cudaMemcpyAsync(inputData, inputs[i], size * sizeof(float), cudaMemcpyDeviceToHost, stream));
                inputData += size;
                ++ i;
            }

            inputData = (float *)mInputBuffer;
            std::vector <Detection> result;
            for (const auto& yolo : mYoloKernel)
            {
                int stride = yolo.width*yolo.height;
                for (int j = 0;j < stride ;++j)
                {
                    for (int k = 0;k < CHECK_COUNT; ++k )
                    {
                        int beginIdx = (LOCATIONS + 1 + mClassCount)* stride *k + j;
                        int objIndex = beginIdx + LOCATIONS*stride;
                        
                        //check obj
                        float objProb = Logist(inputData[objIndex]);   
                        if(objProb <= IGNORE_THRESH)
                            continue;

                        //classes
                        int classId = -1;
                        float maxProb = IGNORE_THRESH;
                        for (int c = 0;c< mClassCount;++c){
                            float cProb =  Logist(inputData[beginIdx + (5 + c) * stride]) * objProb;
                            if(cProb > maxProb){
                                maxProb = cProb;
                                classId = c;
                            }
                        }
            
                        if(classId >= 0) {
                            Detection det;
                            int row = j / yolo.width;
                            int cols = j % yolo.width;
    
                            //Location
                            det.bbox[0] = (cols + Logist(inputData[beginIdx]))/ yolo.width;
                            det.bbox[1] = (row + Logist(inputData[beginIdx+stride]))/ yolo.height;
                            det.bbox[2] = exp(inputData[beginIdx+2*stride]) * yolo.anchors[2*k];
                            det.bbox[3] = exp(inputData[beginIdx+3*stride]) * yolo.anchors[2*k + 1];
                            det.classId = classId;
                            det.prob = maxProb;
                            //det.objectness = objProb;

                            result.emplace_back(det);
                        }
                    }
                }

                inputData += (LOCATIONS + 1 + mClassCount) * stride * CHECK_COUNT;
            }

            
            int detCount =result.size();
            auto data = (float *)mOutputBuffer;
            //copy count;
            data[0] = (float)detCount;
            //std::cout << "detCount"<< detCount << std::endl;
            data++;
            //copy result
            memcpy(data,result.data(),result.size()*sizeof(Detection));

            //(count + det result)
            CUDA_CHECK(cudaMemcpyAsync(outputs, mOutputBuffer, sizeof(float) + result.size()*sizeof(Detection), cudaMemcpyHostToDevice, stream));
    };

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
