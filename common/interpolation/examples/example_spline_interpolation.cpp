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

#include "interpolation/spline_interpolation.hpp"

#include <matplotlibcpp17/pyplot.h>

#include <vector>

int main()
{
  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();
  {
    std::vector<double> base_keys = {3.8344151882577773, 4.236547993389047,  4.375872112626925,
                                     5.448831829968968,  5.4881350392732475, 6.027633760716439,
                                     6.458941130666561,  7.151893663724195,  8.917730007820797,
                                     9.636627605010293};
    std::vector<double> base_values = {7.917250380826646,  5.288949197529044,  5.680445610939323,
                                       9.25596638292661,   0.7103605819788694, 0.8712929970154071,
                                       0.2021839744032572, 8.32619845547938,   7.781567509498505,
                                       8.700121482468191};
    std::vector<double> query_keys = {
      3.83441519, 3.89302339, 3.9516316,  4.01023981, 4.06884801, 4.12745622, 4.18606443,
      4.24467263, 4.30328084, 4.36188904, 4.42049725, 4.47910546, 4.53771366, 4.59632187,
      4.65493008, 4.71353828, 4.77214649, 4.83075469, 4.8893629,  4.94797111, 5.00657931,
      5.06518752, 5.12379573, 5.18240393, 5.24101214, 5.29962034, 5.35822855, 5.41683676,
      5.47544496, 5.53405317, 5.59266138, 5.65126958, 5.70987779, 5.76848599, 5.8270942,
      5.88570241, 5.94431061, 6.00291882, 6.06152702, 6.12013523, 6.17874344, 6.23735164,
      6.29595985, 6.35456806, 6.41317626, 6.47178447, 6.53039267, 6.58900088, 6.64760909,
      6.70621729, 6.7648255,  6.82343371, 6.88204191, 6.94065012, 6.99925832, 7.05786653,
      7.11647474, 7.17508294, 7.23369115, 7.29229936, 7.35090756, 7.40951577, 7.46812397,
      7.52673218, 7.58534039, 7.64394859, 7.7025568,  7.76116501, 7.81977321, 7.87838142,
      7.93698962, 7.99559783, 8.05420604, 8.11281424, 8.17142245, 8.23003066, 8.28863886,
      8.34724707, 8.40585527, 8.46446348, 8.52307169, 8.58167989, 8.6402881,  8.69889631,
      8.75750451, 8.81611272, 8.87472092, 8.93332913, 8.99193734, 9.05054554, 9.10915375,
      9.16776196, 9.22637016, 9.28497837, 9.34358657, 9.40219478, 9.46080299, 9.51941119,
      9.5780194,  9.63662761};

    const auto query_values = interpolation::spline(base_keys, base_values, query_keys);

    plt.scatter(Args(base_keys, base_values), Kwargs{"color"_a = "red"});
    plt.plot(Args(query_keys, query_values), Kwargs{"color"_a = "red"});
    plt.show();
  }

  {
    std::vector<double> base_keys = {0.0, 1.0};
    std::vector<double> base_values = {0.0, 1.0};
    std::vector<double> query_keys = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};

    const auto query_values = interpolation::spline(base_keys, base_values, query_keys);

    plt.scatter(Args(base_keys, base_values), Kwargs{"color"_a = "red"});
    plt.plot(Args(query_keys, query_values), Kwargs{"color"_a = "red"});
    plt.show();
  }

  {
    std::vector<double> base_keys = {0.0, 1.0, 2.0};
    std::vector<double> base_values = {0.0, -1.0, 1.0};
    std::vector<double> query_keys = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0,
                                      1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0};

    const auto query_values = interpolation::spline(base_keys, base_values, query_keys);

    plt.scatter(Args(base_keys, base_values), Kwargs{"color"_a = "red"});
    plt.plot(Args(query_keys, query_values), Kwargs{"color"_a = "red"});
    plt.show();
  }
}
