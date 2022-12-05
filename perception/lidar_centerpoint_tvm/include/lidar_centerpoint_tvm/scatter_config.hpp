// Copyright 2021 Arm Limited and Contributors.
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

#include "tvm_utility/pipeline.hpp"


namespace scatter_config
{

static const tvm_utility::pipeline::InferenceEngineTVMConfig config {
  {
    2,
    1,
    0
  },  // modelzoo_version

  "scatter",  // network_name
  "llvm",  // network_backend

  "./",  //network_module_path
  "./",  // network_graph_path
  "./",  // network_params_path

  kDLFloat,  // tvm_dtype_code
  32,  // tvm_dtype_bits
  1,  // tvm_dtype_lanes

  kDLCPU,  // tvm_device_type
  0,  // tvm_device_id

  {
    {"pillar_features", {40000, 1, 32}},
    {"coords", {40000, 3}},
    {"spatial_features", {1, 32, 560, 560}}
  },  // network_inputs

  {
    {"spatial_features", {1, 32, 560, 560}}
  }   // network_outputs
};

}  // namespace model_zoo
