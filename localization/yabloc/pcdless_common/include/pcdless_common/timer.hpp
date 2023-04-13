// Copyright 2023 TIER IV, Inc.
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

#pragma once
#include <chrono>
#include <ostream>
namespace pcdless::common
{
class Timer
{
public:
  Timer() { reset(); }

  void reset() { start = std::chrono::system_clock::now(); }

  long milli_seconds() const
  {
    auto dur = std::chrono::system_clock::now() - start;
    return std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
  }

  long micro_seconds() const
  {
    auto dur = std::chrono::system_clock::now() - start;
    return std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
  }

  friend std::ostream & operator<<(std::ostream & os, Timer & t)
  {
    os << t.micro_seconds() / 1000.f << "[ms] ";
    return os;
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start;
};

}  // namespace pcdless::common