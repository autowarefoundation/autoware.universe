// Copyright 2024 AutoCore, Inc.
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
#ifdef __HAVE_NVJPEG__

#include "autoware/tensorrt_bevdet/nvjpegdecoder.hpp"

#include <chrono>

using std::chrono::duration;
using std::chrono::high_resolution_clock;

int dev_malloc(void ** p, size_t s)
{
  return static_cast<int>(cudaMalloc(p, s));
}

int dev_free(void * p)
{
  return static_cast<int>(cudaFree(p));
}

int host_malloc(void ** p, size_t s, unsigned int f)
{
  return static_cast<int>(cudaHostAlloc(p, s, f));
}

int host_free(void * p)
{
  return static_cast<int>(cudaFreeHost(p));
}

int prepare_buffers(
  const std::vector<std::vector<char>> & file_data, std::vector<int> & img_width,
  std::vector<int> & img_height, std::vector<nvjpegImage_t> & ibuf,
  std::vector<nvjpegImage_t> & isz, share_params & share_param)
{
  int widths[NVJPEG_MAX_COMPONENT];
  int heights[NVJPEG_MAX_COMPONENT];
  int channels;
  nvjpegChromaSubsampling_t subsampling;

  for (size_t i = 0; i < file_data.size(); i++) {
    CHECK_NVJPEG(nvjpegGetImageInfo(
      share_param.nvjpeg_handle, reinterpret_cast<uchar *>(file_data[i].data()),
      file_data[i].size(), &channels, &subsampling, widths, heights));

    int mul = 1;
    // in the case of interleaved RGB output, write only to single channel, but
    // 3 samples at once
    if (share_param.fmt == NVJPEG_OUTPUT_RGBI || share_param.fmt == NVJPEG_OUTPUT_BGRI) {
      channels = 1;
      mul = 3;
    } else if (share_param.fmt == NVJPEG_OUTPUT_RGB || share_param.fmt == NVJPEG_OUTPUT_BGR) {
      // in the case of rgb create 3 buffers with sizes of original image
      channels = 3;
      widths[1] = widths[2] = widths[0];
      heights[1] = heights[2] = heights[0];
    }

    if (img_width[i] != widths[0] || img_height[i] != heights[0]) {
      img_width[i] = widths[0];
      img_height[i] = heights[0];
      // realloc output buffer if required
      for (int c = 0; c < channels; c++) {
        int aw = mul * widths[c];
        int ah = heights[c];
        size_t sz = aw * ah;
        ibuf[i].pitch[c] = aw;
        if (sz > isz[i].pitch[c]) {
          if (ibuf[i].channel[c]) {
            CHECK_CUDA(cudaFree(ibuf[i].channel[c]));
          }
          CHECK_CUDA(cudaMalloc(reinterpret_cast<void **>(&ibuf[i].channel[c]), sz));
          isz[i].pitch[c] = sz;
        }
      }
    }
  }
  return EXIT_SUCCESS;
}

void create_decoupled_api_handles(std::vector<decode_params_t> & params, share_params & share_param)
{
  for (size_t i = 0; i < params.size(); i++) {
    CHECK_NVJPEG(nvjpegDecoderCreate(
      share_param.nvjpeg_handle, NVJPEG_BACKEND_DEFAULT, &params[i].nvjpeg_decoder));
    CHECK_NVJPEG(nvjpegDecoderStateCreate(
      share_param.nvjpeg_handle, params[i].nvjpeg_decoder, &params[i].nvjpeg_decoupled_state));

    CHECK_NVJPEG(
      nvjpegBufferPinnedCreate(share_param.nvjpeg_handle, NULL, &params[i].pinned_buffers[0]));
    CHECK_NVJPEG(
      nvjpegBufferPinnedCreate(share_param.nvjpeg_handle, NULL, &params[i].pinned_buffers[1]));
    CHECK_NVJPEG(
      nvjpegBufferDeviceCreate(share_param.nvjpeg_handle, NULL, &params[i].device_buffer));

    CHECK_NVJPEG(nvjpegJpegStreamCreate(share_param.nvjpeg_handle, &params[i].jpeg_streams[0]));
    CHECK_NVJPEG(nvjpegJpegStreamCreate(share_param.nvjpeg_handle, &params[i].jpeg_streams[1]));

    CHECK_NVJPEG(
      nvjpegDecodeParamsCreate(share_param.nvjpeg_handle, &params[i].nvjpeg_decode_params));
  }
}

void destroy_decoupled_api_handles(
  std::vector<decode_params_t> & params, share_params & share_param)
{
  for (size_t i = 0; i < params.size(); i++) {
    CHECK_NVJPEG(nvjpegDecodeParamsDestroy(params[i].nvjpeg_decode_params));

    CHECK_NVJPEG(nvjpegJpegStreamDestroy(params[i].jpeg_streams[0]));
    CHECK_NVJPEG(nvjpegJpegStreamDestroy(params[i].jpeg_streams[1]));
    CHECK_NVJPEG(nvjpegBufferPinnedDestroy(params[i].pinned_buffers[0]));
    CHECK_NVJPEG(nvjpegBufferPinnedDestroy(params[i].pinned_buffers[1]));
    CHECK_NVJPEG(nvjpegBufferDeviceDestroy(params[i].device_buffer));

    CHECK_NVJPEG(nvjpegJpegStateDestroy(params[i].nvjpeg_decoupled_state));
    CHECK_NVJPEG(nvjpegDecoderDestroy(params[i].nvjpeg_decoder));
  }
}

void release_buffers(std::vector<nvjpegImage_t> & ibuf)
{
  for (size_t i = 0; i < ibuf.size(); i++) {
    for (size_t c = 0; c < NVJPEG_MAX_COMPONENT; c++)
      if (ibuf[i].channel[c]) CHECK_CUDA(cudaFree(ibuf[i].channel[c]));
  }
}

int get_img(
  const uchar * d_chanR, int pitchR, const uchar * d_chanG, int pitchG, const uchar * d_chanB,
  int pitchB, size_t width, size_t height, uchar * img)
{
  size_t step = width * height;

  CHECK_CUDA(cudaMemcpy2D(
    img + 0 * step, static_cast<size_t>(width), d_chanR, static_cast<size_t>(pitchR), width, height,
    cudaMemcpyDeviceToDevice));
  CHECK_CUDA(cudaMemcpy2D(
    img + 1 * step, static_cast<size_t>(width), d_chanG, static_cast<size_t>(pitchG), width, height,
    cudaMemcpyDeviceToDevice));
  CHECK_CUDA(cudaMemcpy2D(
    img + 2 * step, static_cast<size_t>(width), d_chanB, static_cast<size_t>(pitchB), width, height,
    cudaMemcpyDeviceToDevice));

  return EXIT_SUCCESS;
}

void decode_single_image(
  const std::vector<char> & img_data, nvjpegImage_t & out, decode_params_t & params,
  share_params & share_param, double & time)
{
  CHECK_CUDA(cudaStreamSynchronize(params.stream));
  cudaEvent_t startEvent = NULL, stopEvent = NULL;
  float loopTime = 0;
  // cudaEventBlockingSync
  CHECK_CUDA(cudaEventCreate(&startEvent));
  CHECK_CUDA(cudaEventCreate(&stopEvent));

  CHECK_CUDA(cudaEventRecord(startEvent, params.stream));

  CHECK_NVJPEG(nvjpegStateAttachDeviceBuffer(params.nvjpeg_decoupled_state, params.device_buffer));
  int buffer_index = 0;
  CHECK_NVJPEG(nvjpegDecodeParamsSetOutputFormat(params.nvjpeg_decode_params, share_param.fmt));

  CHECK_NVJPEG(nvjpegJpegStreamParse(
    share_param.nvjpeg_handle, reinterpret_cast<const uchar *>(img_data.data()), img_data.size(), 0,
    0, params.jpeg_streams[buffer_index]));

  CHECK_NVJPEG(nvjpegStateAttachPinnedBuffer(
    params.nvjpeg_decoupled_state, params.pinned_buffers[buffer_index]));

  CHECK_NVJPEG(nvjpegDecodeJpegHost(
    share_param.nvjpeg_handle, params.nvjpeg_decoder, params.nvjpeg_decoupled_state,
    params.nvjpeg_decode_params, params.jpeg_streams[buffer_index]));

  CHECK_CUDA(cudaStreamSynchronize(params.stream));

  CHECK_NVJPEG(nvjpegDecodeJpegTransferToDevice(
    share_param.nvjpeg_handle, params.nvjpeg_decoder, params.nvjpeg_decoupled_state,
    params.jpeg_streams[buffer_index], params.stream));

  buffer_index = 1 - buffer_index;  // switch pinned buffer in pipeline mode to avoid an extra sync

  CHECK_NVJPEG(nvjpegDecodeJpegDevice(
    share_param.nvjpeg_handle, params.nvjpeg_decoder, params.nvjpeg_decoupled_state, &out,
    params.stream));

  // CHECK_CUDA(cudaStreamSynchronize(params.stream)); //TODO new add

  CHECK_CUDA(cudaEventRecord(stopEvent, params.stream));

  CHECK_CUDA(cudaEventSynchronize(stopEvent));
  CHECK_CUDA(cudaEventElapsedTime(&loopTime, startEvent, stopEvent));
  time = 0.001 * static_cast<double>(loopTime);  // cudaEventElapsedTime returns milliseconds
                                                 // time = 1.0;
}

nvjpegDecoder::nvjpegDecoder(size_t n, size_t _fmt)
: N_img(n), iout(n), isz(n), widths(n), heights(n), params(n), share_param(_fmt)
{
  init();
}

int nvjpegDecoder::decode(const std::vector<std::vector<char>> & files_data, uchar * out_imgs)
{
  for (size_t i = 0; i < params.size(); i++) {
    CHECK_CUDA(cudaStreamCreate(&params[i].stream));
  }
  if (prepare_buffers(files_data, widths, heights, iout, isz, share_param)) return EXIT_FAILURE;

  double total_time = 0;
  double times[6];

  auto decode_start = high_resolution_clock::now();
  std::vector<std::thread> threads(files_data.size());
  for (size_t i = 0; i < files_data.size(); i++) {
    threads[i] = std::thread(
      decode_single_image, std::ref(files_data[i]), std::ref(iout[i]), std::ref(params[i]),
      std::ref(share_param), std::ref(times[i]));
  }
  for (size_t i = 0; i < files_data.size(); i++) {
    threads[i].join();
    total_time += times[i];
  }
  auto decode_end = high_resolution_clock::now();
  duration<double> decode_time = decode_end - decode_start;

  RCLCPP_INFO(
    rclcpp::get_logger("nvjpegDecoder"), "Decode total time : %.4lf ms",
    decode_time.count() * 1000);

  for (size_t i = 0; i < files_data.size(); i++) {
    get_img(
      iout[i].channel[0], iout[i].pitch[0], iout[i].channel[1], iout[i].pitch[1],
      iout[i].channel[2], iout[i].pitch[2], widths[i], heights[i],
      out_imgs + i * 3 * widths[i] * heights[i]);
  }

  for (size_t i = 0; i < params.size(); i++) {
    CHECK_CUDA(cudaStreamDestroy(params[i].stream));
  }
  return EXIT_SUCCESS;
}

int nvjpegDecoder::init()
{
  nvjpegDevAllocator_t dev_allocator = {&dev_malloc, &dev_free};
  nvjpegPinnedAllocator_t pinned_allocator = {&host_malloc, &host_free};

  nvjpegStatus_t status = nvjpegCreateEx(
    NVJPEG_BACKEND_HARDWARE, &dev_allocator, &pinned_allocator, NVJPEG_FLAGS_DEFAULT,
    &share_param.nvjpeg_handle);
  share_param.hw_decode_available = true;
  if (status == NVJPEG_STATUS_ARCH_MISMATCH) {
    RCLCPP_WARN(
      rclcpp::get_logger("nvjpegDecoder"),
      "Hardware Decoder not supported. Falling back to default backend");
    CHECK_NVJPEG(nvjpegCreateEx(
      NVJPEG_BACKEND_DEFAULT, &dev_allocator, &pinned_allocator, NVJPEG_FLAGS_DEFAULT,
      &share_param.nvjpeg_handle));
    share_param.hw_decode_available = false;
  } else {
    CHECK_NVJPEG(status);
  }

  CHECK_NVJPEG(nvjpegJpegStateCreate(share_param.nvjpeg_handle, &share_param.nvjpeg_state));

  create_decoupled_api_handles(params, share_param);

  for (size_t i = 0; i < iout.size(); i++) {
    for (size_t c = 0; c < NVJPEG_MAX_COMPONENT; c++) {
      iout[i].channel[c] = NULL;
      iout[i].pitch[c] = 0;
      isz[i].pitch[c] = 0;
    }
  }
  return EXIT_SUCCESS;
}

nvjpegDecoder::~nvjpegDecoder()
{
  release_buffers(iout);
  destroy_decoupled_api_handles(params, share_param);
  CHECK_NVJPEG(nvjpegJpegStateDestroy(share_param.nvjpeg_state));
  CHECK_NVJPEG(nvjpegDestroy(share_param.nvjpeg_handle));
}

#endif
