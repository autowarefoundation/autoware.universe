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
#ifndef MAIN_FUNCTIONS_HPP_
#define MAIN_FUNCTIONS_HPP_

#include <string>

void walk(const std::string & train_result_dir);
void test(const std::string & train_result_dir, const std::string & dataset_dir);
void infer(const std::string & train_result_dir, const std::string & dataset_dir);

#endif  // MAIN_FUNCTIONS_HPP_
