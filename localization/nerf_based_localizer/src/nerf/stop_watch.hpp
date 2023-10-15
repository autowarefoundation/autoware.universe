// Copyright 2023 Autoware Foundation
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
//
// Created by ppwang on 2022/5/18.
//

#ifndef NERF__STOP_WATCH_HPP_
#define NERF__STOP_WATCH_HPP_

#include <chrono>
#include <string>

class ScopeWatch
{
public:
  ScopeWatch(const std::string & scope_name);
  ~ScopeWatch();

private:
  std::chrono::steady_clock::time_point t_point_;
  std::string scope_name_;
};

class Timer
{
public:
  void start() { start_time_ = std::chrono::steady_clock::now(); }
  int64_t elapsed_milli_seconds() const
  {
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    return std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
  }
  double elapsed_seconds() const { return elapsed_milli_seconds() / 1000.0; }

private:
  std::chrono::steady_clock::time_point start_time_;
};

#endif  // NERF__STOP_WATCH_HPP_
