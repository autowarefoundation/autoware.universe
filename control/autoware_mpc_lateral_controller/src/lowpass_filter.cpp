// Copyright 2018-2021 The Autoware Foundation
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

#include "autoware/mpc_lateral_controller/lowpass_filter.hpp"

#include <algorithm>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{
Butterworth2dFilter::Butterworth2dFilter(double dt, double f_cutoff_hz)
{
  initialize(dt, f_cutoff_hz);
}

Butterworth2dFilter::~Butterworth2dFilter()
{
}

void Butterworth2dFilter::initialize(const double & dt, const double & f_cutoff_hz)
{
  m_y1 = 0.0;
  m_y2 = 0.0;
  m_u2 = 0.0;
  m_u1 = 0.0;

  // 2d Butterworth low pass filter with bi-linear transformation
  const double f_sampling_hz = 1.0 / dt;
  const double xi = 1.0 / std::tan(M_PI * f_cutoff_hz / f_sampling_hz);
  const double xi_2 = xi * xi;
  const double q = std::sqrt(2);
  m_b0 = 1.0 / (1.0 + q * xi + xi_2);
  m_b1 = 2.0 * m_b0;
  m_b2 = m_b0;
  m_a1 = 2.0 * (xi_2 - 1.0) * m_b0;
  m_a2 = -(1.0 - q * xi + xi_2) * m_b0;
}

double Butterworth2dFilter::filter(const double & u0)
{
  double y0 = m_b2 * m_u2 + m_b1 * m_u1 + m_b0 * u0 + m_a2 * m_y2 + m_a1 * m_y1;
  m_y2 = m_y1;
  m_y1 = y0;
  m_u2 = m_u1;
  m_u1 = u0;
  return y0;
}

void Butterworth2dFilter::filt_vector(const std::vector<double> & t, std::vector<double> & u) const
{
  u.resize(t.size());
  double y1 = t.at(0);
  double y2 = t.at(0);
  double u2 = t.at(0);
  double u1 = t.at(0);
  for (size_t i = 0; i < t.size(); ++i) {
    const double u0 = t.at(i);
    const double y0 = m_b2 * u2 + m_b1 * u1 + m_b0 * u0 + m_a2 * y2 + m_a1 * y1;
    y2 = y1;
    y1 = y0;
    u2 = u1;
    u1 = u0;
    u.at(i) = y0;
  }
}

// filtering forward and backward direction
void Butterworth2dFilter::filtfilt_vector(
  const std::vector<double> & t, std::vector<double> & u) const
{
  std::vector<double> t_fwd(t);
  std::vector<double> t_rev(t);

  // forward filtering
  filt_vector(t, t_fwd);

  // backward filtering
  std::reverse(t_rev.begin(), t_rev.end());
  filt_vector(t, t_rev);
  std::reverse(t_rev.begin(), t_rev.end());

  // merge
  u.clear();
  for (size_t i = 0; i < t.size(); ++i) {
    u.push_back((t_fwd[i] + t_rev[i]) * 0.5);
  }
}

void Butterworth2dFilter::getCoefficients(std::vector<double> & coeffs) const
{
  coeffs.clear();
  coeffs.push_back(m_a1);
  coeffs.push_back(m_a2);
  coeffs.push_back(m_b0);
  coeffs.push_back(m_b1);
  coeffs.push_back(m_b2);
}

namespace MoveAverageFilter
{
bool filt_vector(const int num, std::vector<double> & u)
{
  if (static_cast<int>(u.size()) < num) {
    return false;
  }
  std::vector<double> filtered_u(u);
  for (int i = 0; i < static_cast<int>(u.size()); ++i) {
    double tmp = 0.0;
    int num_tmp = 0;
    double count = 0;
    if ((i - num < 0) || (i + num > static_cast<int>(u.size()) - 1)) {
      num_tmp = std::min(i, static_cast<int>(u.size()) - i - 1);
    } else {
      num_tmp = num;
    }

    for (int j = -num_tmp; j <= num_tmp; ++j) {
      int index = i + j;
      if (index >= 0 && index < static_cast<int>(u.size())) {
        tmp += u[static_cast<size_t>(index)];
        ++count;
      }
    }
    filtered_u[static_cast<size_t>(i)] = tmp / count;
  }
  u = filtered_u;
  return true;
}
}  // namespace MoveAverageFilter
}  // namespace autoware::motion::control::mpc_lateral_controller
