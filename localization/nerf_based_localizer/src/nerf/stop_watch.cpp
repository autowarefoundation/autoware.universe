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
// This file is derived from the following file.
// https://github.com/Totoro97/f2-nerf/blob/main/src/Utils/StopWatch.cpp
//
// Created by ppwang on 2022/5/18.
//

#include "stop_watch.hpp"

#include <torch/torch.h>

#include <iostream>

ScopeWatch::ScopeWatch(const std::string & scope_name) : scope_name_(scope_name)
{
  torch::cuda::synchronize();
  t_point_ = std::chrono::steady_clock::now();
  std::cout << "[" << scope_name_ << "] begin" << std::endl;
}

ScopeWatch::~ScopeWatch()
{
  torch::cuda::synchronize();
  std::chrono::steady_clock::time_point new_point = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_span =
    std::chrono::duration_cast<std::chrono::duration<double>>(new_point - t_point_);
  std::cout << "[" << scope_name_ << "] end in " << time_span.count() << " seconds" << std::endl;
}
