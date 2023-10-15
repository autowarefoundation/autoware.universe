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
// Created by ppwang on 2022/10/8.
//

#include "common.hpp"
#include "common_cuda.hpp"
#include "sh_shader.hpp"

using Tensor = torch::Tensor;

__global__ void SHKernel(
  const uint32_t num_elements, const uint32_t degree, float * data_in, float * data_out)
{
  const uint32_t i = threadIdx.x + blockIdx.x * blockDim.x;
  if (i >= num_elements) return;

  data_out = data_out + (degree * degree) * i;

  float x = data_in[i * 3];
  float y = data_in[i * 3 + 1];
  float z = data_in[i * 3 + 2];

  // Let compiler figure out how to sequence/reorder these calculations w.r.t. branches
  float xy = x * y, xz = x * z, yz = y * z, x2 = x * x, y2 = y * y, z2 = z * z;
  float x4 = x2 * x2, y4 = y2 * y2, z4 = z2 * z2;
  float x6 = x4 * x2, y6 = y4 * y2, z6 = z4 * z2;

  // SH polynomials generated using scripts/gen_sh.py based on the recurrence relations in appendix
  // A1 of https://www.ppsloan.org/publications/StupidSH36.pdf

  data_out[0] = 0.28209479177387814f;  // 1/(2*sqrt(pi))
  if (degree <= 1) {
    return;
  }
  data_out[1] = -0.48860251190291987f * y;  // -sqrt(3)*y/(2*sqrt(pi))
  data_out[2] = 0.48860251190291987f * z;   // sqrt(3)*z/(2*sqrt(pi))
  data_out[3] = -0.48860251190291987f * x;  // -sqrt(3)*x/(2*sqrt(pi))
  if (degree <= 2) {
    return;
  }
  data_out[4] = 1.0925484305920792f * xy;   // sqrt(15)*xy/(2*sqrt(pi))
  data_out[5] = -1.0925484305920792f * yz;  // -sqrt(15)*yz/(2*sqrt(pi))
  data_out[6] =
    0.94617469575755997f * z2 - 0.31539156525251999f;  // sqrt(5)*(3*z2 - 1)/(4*sqrt(pi))
  data_out[7] = -1.0925484305920792f * xz;             // -sqrt(15)*xz/(2*sqrt(pi))
  data_out[8] =
    0.54627421529603959f * x2 - 0.54627421529603959f * y2;  // sqrt(15)*(x2 - y2)/(4*sqrt(pi))
  if (degree <= 3) {
    return;
  }
  data_out[9] =
    0.59004358992664352f * y * (-3.0f * x2 + y2);  // sqrt(70)*y*(-3*x2 + y2)/(8*sqrt(pi))
  data_out[10] = 2.8906114426405538f * xy * z;     // sqrt(105)*xy*z/(2*sqrt(pi))
  data_out[11] =
    0.45704579946446572f * y * (1.0f - 5.0f * z2);  // sqrt(42)*y*(1 - 5*z2)/(8*sqrt(pi))
  data_out[12] = 0.3731763325901154f * z * (5.0f * z2 - 3.0f);  // sqrt(7)*z*(5*z2 - 3)/(4*sqrt(pi))
  data_out[13] =
    0.45704579946446572f * x * (1.0f - 5.0f * z2);     // sqrt(42)*x*(1 - 5*z2)/(8*sqrt(pi))
  data_out[14] = 1.4453057213202769f * z * (x2 - y2);  // sqrt(105)*z*(x2 - y2)/(4*sqrt(pi))
  data_out[15] =
    0.59004358992664352f * x * (-x2 + 3.0f * y2);  // sqrt(70)*x*(-x2 + 3*y2)/(8*sqrt(pi))
  if (degree <= 4) {
    return;
  }
  data_out[16] = 2.5033429417967046f * xy * (x2 - y2);  // 3*sqrt(35)*xy*(x2 - y2)/(4*sqrt(pi))
  data_out[17] =
    1.7701307697799304f * yz * (-3.0f * x2 + y2);  // 3*sqrt(70)*yz*(-3*x2 + y2)/(8*sqrt(pi))
  data_out[18] =
    0.94617469575756008f * xy * (7.0f * z2 - 1.0f);  // 3*sqrt(5)*xy*(7*z2 - 1)/(4*sqrt(pi))
  data_out[19] =
    0.66904654355728921f * yz * (3.0f - 7.0f * z2);  // 3*sqrt(10)*yz*(3 - 7*z2)/(8*sqrt(pi))
  data_out[20] = -3.1735664074561294f * z2 + 3.7024941420321507f * z4 +
                 0.31735664074561293f;  // 3*(-30*z2 + 35*z4 + 3)/(16*sqrt(pi))
  data_out[21] =
    0.66904654355728921f * xz * (3.0f - 7.0f * z2);  // 3*sqrt(10)*xz*(3 - 7*z2)/(8*sqrt(pi))
  data_out[22] = 0.47308734787878004f * (x2 - y2) *
                 (7.0f * z2 - 1.0f);  // 3*sqrt(5)*(x2 - y2)*(7*z2 - 1)/(8*sqrt(pi))
  data_out[23] =
    1.7701307697799304f * xz * (-x2 + 3.0f * y2);  // 3*sqrt(70)*xz*(-x2 + 3*y2)/(8*sqrt(pi))
  data_out[24] = -3.7550144126950569f * x2 * y2 + 0.62583573544917614f * x4 +
                 0.62583573544917614f * y4;  // 3*sqrt(35)*(-6*x2*y2 + x4 + y4)/(16*sqrt(pi))
  if (degree <= 5) {
    return;
  }
  data_out[25] =
    0.65638205684017015f * y *
    (10.0f * x2 * y2 - 5.0f * x4 - y4);  // 3*sqrt(154)*y*(10*x2*y2 - 5*x4 - y4)/(32*sqrt(pi))
  data_out[26] =
    8.3026492595241645f * xy * z * (x2 - y2);  // 3*sqrt(385)*xy*z*(x2 - y2)/(4*sqrt(pi))
  data_out[27] = -0.48923829943525038f * y * (3.0f * x2 - y2) *
                 (9.0f * z2 - 1.0f);  // -sqrt(770)*y*(3*x2 - y2)*(9*z2 - 1)/(32*sqrt(pi))
  data_out[28] =
    4.7935367849733241f * xy * z * (3.0f * z2 - 1.0f);  // sqrt(1155)*xy*z*(3*z2 - 1)/(4*sqrt(pi))
  data_out[29] = 0.45294665119569694f * y *
                 (14.0f * z2 - 21.0f * z4 - 1.0f);  // sqrt(165)*y*(14*z2 - 21*z4 - 1)/(16*sqrt(pi))
  data_out[30] =
    0.1169503224534236f * z *
    (-70.0f * z2 + 63.0f * z4 + 15.0f);  // sqrt(11)*z*(-70*z2 + 63*z4 + 15)/(16*sqrt(pi))
  data_out[31] = 0.45294665119569694f * x *
                 (14.0f * z2 - 21.0f * z4 - 1.0f);  // sqrt(165)*x*(14*z2 - 21*z4 - 1)/(16*sqrt(pi))
  data_out[32] = 2.3967683924866621f * z * (x2 - y2) *
                 (3.0f * z2 - 1.0f);  // sqrt(1155)*z*(x2 - y2)*(3*z2 - 1)/(8*sqrt(pi))
  data_out[33] = -0.48923829943525038f * x * (x2 - 3.0f * y2) *
                 (9.0f * z2 - 1.0f);  // -sqrt(770)*x*(x2 - 3*y2)*(9*z2 - 1)/(32*sqrt(pi))
  data_out[34] = 2.0756623148810411f * z *
                 (-6.0f * x2 * y2 + x4 + y4);  // 3*sqrt(385)*z*(-6*x2*y2 + x4 + y4)/(16*sqrt(pi))
  data_out[35] =
    0.65638205684017015f * x *
    (10.0f * x2 * y2 - x4 - 5.0f * y4);  // 3*sqrt(154)*x*(10*x2*y2 - x4 - 5*y4)/(32*sqrt(pi))
  if (degree <= 6) {
    return;
  }
  data_out[36] = 1.3663682103838286f * xy *
                 (-10.0f * x2 * y2 + 3.0f * x4 +
                  3.0f * y4);  // sqrt(6006)*xy*(-10*x2*y2 + 3*x4 + 3*y4)/(32*sqrt(pi))
  data_out[37] =
    2.3666191622317521f * yz *
    (10.0f * x2 * y2 - 5.0f * x4 - y4);  // 3*sqrt(2002)*yz*(10*x2*y2 - 5*x4 - y4)/(32*sqrt(pi))
  data_out[38] = 2.0182596029148963f * xy * (x2 - y2) *
                 (11.0f * z2 - 1.0f);  // 3*sqrt(91)*xy*(x2 - y2)*(11*z2 - 1)/(8*sqrt(pi))
  data_out[39] = -0.92120525951492349f * yz * (3.0f * x2 - y2) *
                 (11.0f * z2 - 3.0f);  // -sqrt(2730)*yz*(3*x2 - y2)*(11*z2 - 3)/(32*sqrt(pi))
  data_out[40] =
    0.92120525951492349f * xy *
    (-18.0f * z2 + 33.0f * z4 + 1.0f);  // sqrt(2730)*xy*(-18*z2 + 33*z4 + 1)/(32*sqrt(pi))
  data_out[41] =
    0.58262136251873131f * yz *
    (30.0f * z2 - 33.0f * z4 - 5.0f);  // sqrt(273)*yz*(30*z2 - 33*z4 - 5)/(16*sqrt(pi))
  data_out[42] = 6.6747662381009842f * z2 - 20.024298714302954f * z4 + 14.684485723822165f * z6 -
                 0.31784601133814211f;  // sqrt(13)*(105*z2 - 315*z4 + 231*z6 - 5)/(32*sqrt(pi))
  data_out[43] =
    0.58262136251873131f * xz *
    (30.0f * z2 - 33.0f * z4 - 5.0f);  // sqrt(273)*xz*(30*z2 - 33*z4 - 5)/(16*sqrt(pi))
  data_out[44] = 0.46060262975746175f * (x2 - y2) *
                 (11.0f * z2 * (3.0f * z2 - 1.0f) - 7.0f * z2 +
                  1.0f);  // sqrt(2730)*(x2 - y2)*(11*z2*(3*z2 - 1) - 7*z2 + 1)/(64*sqrt(pi))
  data_out[45] = -0.92120525951492349f * xz * (x2 - 3.0f * y2) *
                 (11.0f * z2 - 3.0f);  // -sqrt(2730)*xz*(x2 - 3*y2)*(11*z2 - 3)/(32*sqrt(pi))
  data_out[46] =
    0.50456490072872406f * (11.0f * z2 - 1.0f) *
    (-6.0f * x2 * y2 + x4 + y4);  // 3*sqrt(91)*(11*z2 - 1)*(-6*x2*y2 + x4 + y4)/(32*sqrt(pi))
  data_out[47] =
    2.3666191622317521f * xz *
    (10.0f * x2 * y2 - x4 - 5.0f * y4);  // 3*sqrt(2002)*xz*(10*x2*y2 - x4 - 5*y4)/(32*sqrt(pi))
  data_out[48] =
    10.247761577878714f * x2 * y4 - 10.247761577878714f * x4 * y2 + 0.6831841051919143f * x6 -
    0.6831841051919143f * y6;  // sqrt(6006)*(15*x2*y4 - 15*x4*y2 + x6 - y6)/(64*sqrt(pi))
  if (degree <= 7) {
    return;
  }
  data_out[49] = 0.70716273252459627f * y *
                 (-21.0f * x2 * y4 + 35.0f * x4 * y2 - 7.0f * x6 +
                  y6);  // 3*sqrt(715)*y*(-21*x2*y4 + 35*x4*y2 - 7*x6 + y6)/(64*sqrt(pi))
  data_out[50] = 5.2919213236038001f * xy * z *
                 (-10.0f * x2 * y2 + 3.0f * x4 +
                  3.0f * y4);  // 3*sqrt(10010)*xy*z*(-10*x2*y2 + 3*x4 + 3*y4)/(32*sqrt(pi))
  data_out[51] = -0.51891557872026028f * y * (13.0f * z2 - 1.0f) *
                 (-10.0f * x2 * y2 + 5.0f * x4 +
                  y4);  // -3*sqrt(385)*y*(13*z2 - 1)*(-10*x2*y2 + 5*x4 + y4)/(64*sqrt(pi))
  data_out[52] = 4.1513246297620823f * xy * z * (x2 - y2) *
                 (13.0f * z2 - 3.0f);  // 3*sqrt(385)*xy*z*(x2 - y2)*(13*z2 - 3)/(8*sqrt(pi))
  data_out[53] = -0.15645893386229404f * y * (3.0f * x2 - y2) *
                 (13.0f * z2 * (11.0f * z2 - 3.0f) - 27.0f * z2 +
                  3.0f);  // -3*sqrt(35)*y*(3*x2 - y2)*(13*z2*(11*z2 - 3) - 27*z2 + 3)/(64*sqrt(pi))
  data_out[54] =
    0.44253269244498261f * xy * z *
    (-110.0f * z2 + 143.0f * z4 + 15.0f);  // 3*sqrt(70)*xy*z*(-110*z2 + 143*z4 + 15)/(32*sqrt(pi))
  data_out[55] = 0.090331607582517306f * y *
                 (-135.0f * z2 + 495.0f * z4 - 429.0f * z6 +
                  5.0f);  // sqrt(105)*y*(-135*z2 + 495*z4 - 429*z6 + 5)/(64*sqrt(pi))
  data_out[56] = 0.068284276912004949f * z *
                 (315.0f * z2 - 693.0f * z4 + 429.0f * z6 -
                  35.0f);  // sqrt(15)*z*(315*z2 - 693*z4 + 429*z6 - 35)/(32*sqrt(pi))
  data_out[57] = 0.090331607582517306f * x *
                 (-135.0f * z2 + 495.0f * z4 - 429.0f * z6 +
                  5.0f);  // sqrt(105)*x*(-135*z2 + 495*z4 - 429*z6 + 5)/(64*sqrt(pi))
  data_out[58] = 0.07375544874083044f * z * (x2 - y2) *
                 (143.0f * z2 * (3.0f * z2 - 1.0f) - 187.0f * z2 +
                  45.0f);  // sqrt(70)*z*(x2 - y2)*(143*z2*(3*z2 - 1) - 187*z2 + 45)/(64*sqrt(pi))
  data_out[59] = -0.15645893386229404f * x * (x2 - 3.0f * y2) *
                 (13.0f * z2 * (11.0f * z2 - 3.0f) - 27.0f * z2 +
                  3.0f);  // -3*sqrt(35)*x*(x2 - 3*y2)*(13*z2*(11*z2 - 3) - 27*z2 + 3)/(64*sqrt(pi))
  data_out[60] =
    1.0378311574405206f * z * (13.0f * z2 - 3.0f) *
    (-6.0f * x2 * y2 + x4 + y4);  // 3*sqrt(385)*z*(13*z2 - 3)*(-6*x2*y2 + x4 + y4)/(32*sqrt(pi))
  data_out[61] = -0.51891557872026028f * x * (13.0f * z2 - 1.0f) *
                 (-10.0f * x2 * y2 + x4 +
                  5.0f * y4);  // -3*sqrt(385)*x*(13*z2 - 1)*(-10*x2*y2 + x4 + 5*y4)/(64*sqrt(pi))
  data_out[62] = 2.6459606618019f * z *
                 (15.0f * x2 * y4 - 15.0f * x4 * y2 + x6 -
                  y6);  // 3*sqrt(10010)*z*(15*x2*y4 - 15*x4*y2 + x6 - y6)/(64*sqrt(pi))
  data_out[63] = 0.70716273252459627f * x *
                 (-35.0f * x2 * y4 + 21.0f * x4 * y2 - x6 +
                  7.0f * y6);  // 3*sqrt(715)*x*(-35*x2*y4 + 21*x4*y2 - x6 + 7*y6)/(64*sqrt(pi))
}

Tensor SHShader::encode(const Tensor & dirs)
{
  CHECK(dirs.is_contiguous());
  int n_pts = dirs.size(0);
  Tensor out = torch::empty({n_pts, DEGREE * DEGREE}, CUDAFloat);
  dim3 grid_dim = LIN_GRID_DIM(n_pts);
  dim3 block_dim = LIN_BLOCK_DIM;

  SHKernel<<<grid_dim, block_dim>>>(n_pts, DEGREE, dirs.data_ptr<float>(), out.data_ptr<float>());

  return out;
}
