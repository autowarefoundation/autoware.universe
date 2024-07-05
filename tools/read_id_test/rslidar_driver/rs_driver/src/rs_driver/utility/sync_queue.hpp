/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>

namespace robosense
{
namespace lidar
{
template <typename T>
class SyncQueue
{
public:
  inline size_t push(const T& value)
  {
#ifndef ENABLE_WAIT_IF_QUEUE_EMPTY
     bool empty = false;
#endif
     size_t size = 0;

    {
      std::lock_guard<std::mutex> lg(mtx_);
#ifndef ENABLE_WAIT_IF_QUEUE_EMPTY
      empty = queue_.empty();
#endif
      queue_.push(value);
      size = queue_.size();
    }

#ifndef ENABLE_WAIT_IF_QUEUE_EMPTY
    if (empty)
      cv_.notify_one();
#endif

    return size;
  }

  inline T pop()
  {
    T value;

    std::lock_guard<std::mutex> lg(mtx_);
    if (!queue_.empty())
    {
      value = queue_.front();
      queue_.pop();
    }

    return value;
  }

  inline T popWait(unsigned int usec = 1000000)
  {
    //
    // Low latency, or low CPU usage, that is the question. 
    //                                            - Hamlet

#ifdef ENABLE_WAIT_IF_QUEUE_EMPTY
    T value;

    {
      std::lock_guard<std::mutex> lg(mtx_);
      if (!queue_.empty())
      {
        value = queue_.front();
        queue_.pop();
        return value;
      }
    }

    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    return value;
#else

    T value;

    std::unique_lock<std::mutex> ul(mtx_);
    cv_.wait_for(ul, std::chrono::microseconds(usec), [this] { return (!queue_.empty()); });

    if (!queue_.empty())
    {
      value = queue_.front();
      queue_.pop();
    }

    return value;
#endif
  }

  inline void clear()
  {
    std::queue<T> empty;
    std::lock_guard<std::mutex> lg(mtx_);
    swap(empty, queue_);
  }

private:
  std::queue<T> queue_;
  std::mutex mtx_;
#ifndef ENABLE_WAIT_IF_QUEUE_EMPTY
  std::condition_variable cv_;
#endif
};
}  // namespace lidar
}  // namespace robosense
