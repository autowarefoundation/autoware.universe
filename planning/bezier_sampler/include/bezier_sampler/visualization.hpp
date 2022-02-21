/*
 * Copyright 2021 Tier IV, Inc. All rights reserved.
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

#pragma once

#include "bezier_sampler/bezier.hpp"
#include "bezier_sampler/matplotlibcpp.h"

namespace motion_planning
{
namespace bezier_sampler
{

//@brief Plot a bezier curve and its control points
void plot(Bezier b, int nb_points = 100);
//@brief Plot a bezier curve and its curvature
void plot_curvature(Bezier b, int nb_points = 100);
//@brief Plot a set of bezier curves
void plot(std::vector<Bezier> bs, int nb_points = 100);

}  // namespace bezier_sampler
}  // namespace motion_planning
