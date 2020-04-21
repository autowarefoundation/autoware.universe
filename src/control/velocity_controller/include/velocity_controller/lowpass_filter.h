/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file lowpass_filter.h
 * @brief vehicle model interface class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

#ifndef VELOCITY_CONTROLLER_LOWPASS_FILTER
#define VELOCITY_CONTROLLER_LOWPASS_FILTER

/**
 * @class 1D low-pass filter
 * @brief filtering values
 */
class Lpf1d
{
private:
  double x_;     //!< @brief current filtered value
  double gain_;  //!< @brief gain value of 1d filter

public:
  Lpf1d(double gain) : gain_(gain){};
  Lpf1d(){};
  ~Lpf1d() = default;

  void init(double gain)
  {
    gain_ = gain;
    x_ = 0.0;
  }

  void reset() { x_ = 0.0; }

  double filter(double u)
  {
    double ret = gain_ * x_ + (1.0 - gain_) * u;
    x_ = ret;
    return ret;
  }
};

#endif
