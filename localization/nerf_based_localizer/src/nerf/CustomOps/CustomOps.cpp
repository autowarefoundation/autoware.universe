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
// Created by ppwang on 2022/10/5.
//

#include "CustomOps.hpp"

namespace torch::autograd
{

variable_list TruncExp::forward(AutogradContext * ctx, Tensor input)
{
  ctx->save_for_backward({input});
  return {torch::exp(input)};
}

variable_list TruncExp::backward(AutogradContext * ctx, variable_list grad_output)
{
  Tensor x = ctx->get_saved_variables()[0];
  return {grad_output[0] * torch::exp(x.clamp(-100.f, 5.f))};
}

}  // namespace torch::autograd
