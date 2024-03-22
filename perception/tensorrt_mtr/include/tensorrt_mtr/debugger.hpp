// Copyright 2024 TIER IV, Inc.
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

#ifndef TENSORRT_MTR__DEBUGGER_HPP_
#define TENSORRT_MTR__DEBUGGER_HPP_

#include <chrono>
#include <iostream>
#include <string>

namespace trt_mtr
{
/**
 * @brief A class to debug the operation time.
 */
class Debugger
{
public:
  /**
   * @brief Create a event to measure the processing time.
   */
  void createEvent()
  {
    start_ = std::chrono::system_clock::now();
    has_event_ = true;
  }

  /**
   * @brief Display elapsed processing time from the event was created.
   *
   * @param prefix The message prefix. Defaults to `""`.
   */
  void printElapsedTime(const std::string & prefix = "")
  {
    if (!has_event_) {
      std::cerr << "There is no event." << std::endl;
    } else {
      end_ = std::chrono::system_clock::now();
      const auto elapsed_time = std::chrono::duration<double, std::milli>(end_ - start_).count();
      std::cout << prefix << elapsed_time << " ms" << std::endl;
      has_event_ = false;
    }
  };

private:
  std::chrono::system_clock::time_point start_, end_;
  bool has_event_{false};
};
}  // namespace trt_mtr
#endif  // TENSORRT_MTR__DEBUGGER_HPP_
