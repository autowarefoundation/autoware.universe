#include "camera_ekf_corrector/logit.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace pcdless
{
namespace
{
struct ProbToLogitTable
{
  ProbToLogitTable()
  {
    for (int i = 0; i < 100; ++i) {
      float p = i / 100.0f;
      table_.at(i) = std::log(p / std::max(1 - p, 1e-6f));
    }
  }
  float operator()(float prob) const
  {
    int index = std::clamp(int(prob * 100), 0, 99);
    return table_.at(index);
  }

  std::array<float, 100> table_;
} prob_to_logit_table;

}  // namespace

float logit_to_prob(float logit, float gain) { return 1.f / (1 + std::exp(-gain * logit)); }

float prob_to_logit(float prob) { return prob_to_logit_table(prob); }

}  // namespace pcdless