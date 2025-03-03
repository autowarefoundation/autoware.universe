// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__OBJECT_RECOGNITION_UTILS__MATCHING_HPP_
#define AUTOWARE__OBJECT_RECOGNITION_UTILS__MATCHING_HPP_

#include "autoware/object_recognition_utils/geometry.hpp"
#include "autoware_utils/geometry/boost_geometry.hpp"
#include "autoware_utils/geometry/boost_polygon_utils.hpp"

#include <boost/geometry.hpp>

#include <algorithm>
#include <numeric>
#include <utility>
#include <vector>

namespace autoware::object_recognition_utils
{
using autoware_utils::Polygon2d;
// minimum area to avoid division by zero
static const double MIN_AREA = 1e-6;

inline double getConvexShapeArea(const Polygon2d & source_polygon, const Polygon2d & target_polygon)
{
  boost::geometry::model::multi_polygon<Polygon2d> union_polygons;
  boost::geometry::union_(source_polygon, target_polygon, union_polygons);

  autoware_utils::Polygon2d hull;
  boost::geometry::convex_hull(union_polygons, hull);
  return boost::geometry::area(hull);
}

inline double getSumArea(const std::vector<Polygon2d> & polygons)
{
  return std::accumulate(polygons.begin(), polygons.end(), 0.0, [](double acc, Polygon2d p) {
    return acc + boost::geometry::area(p);
  });
}

inline double getIntersectionArea(
  const Polygon2d & source_polygon, const Polygon2d & target_polygon)
{
  std::vector<Polygon2d> intersection_polygons;
  boost::geometry::intersection(source_polygon, target_polygon, intersection_polygons);
  return getSumArea(intersection_polygons);
}

inline double getUnionArea(const Polygon2d & source_polygon, const Polygon2d & target_polygon)
{
  std::vector<Polygon2d> union_polygons;
  boost::geometry::union_(source_polygon, target_polygon, union_polygons);
  return getSumArea(union_polygons);
}

template <class T1, class T2>
double get2dIoU(const T1 source_object, const T2 target_object, const double min_union_area = 0.01)
{
  const auto source_polygon = autoware_utils::to_polygon2d(source_object);
  if (boost::geometry::area(source_polygon) < MIN_AREA) return 0.0;
  const auto target_polygon = autoware_utils::to_polygon2d(target_object);
  if (boost::geometry::area(target_polygon) < MIN_AREA) return 0.0;

  const double intersection_area = getIntersectionArea(source_polygon, target_polygon);
  if (intersection_area < MIN_AREA) return 0.0;
  const double union_area = getUnionArea(source_polygon, target_polygon);

  const double iou =
    union_area < min_union_area ? 0.0 : std::min(1.0, intersection_area / union_area);
  return iou;
}

template <class T1, class T2>
double get2dGeneralizedIoU(const T1 & source_object, const T2 & target_object)
{
  const auto source_polygon = autoware_utils::to_polygon2d(source_object);
  if (boost::geometry::area(source_polygon) < MIN_AREA) return 0.0;
  const auto target_polygon = autoware_utils::to_polygon2d(target_object);
  if (boost::geometry::area(target_polygon) < MIN_AREA) return 0.0;

  const double intersection_area = getIntersectionArea(source_polygon, target_polygon);
  const double union_area = getUnionArea(source_polygon, target_polygon);
  const double convex_shape_area = getConvexShapeArea(source_polygon, target_polygon);

  const double iou = union_area < 0.01 ? 0.0 : std::min(1.0, intersection_area / union_area);
  return iou - (convex_shape_area - union_area) / convex_shape_area;
}

template <class T1, class T2>
double get2dPrecision(const T1 source_object, const T2 target_object)
{
  const auto source_polygon = autoware_utils::to_polygon2d(source_object);
  const double source_area = boost::geometry::area(source_polygon);
  if (source_area < MIN_AREA) return 0.0;
  const auto target_polygon = autoware_utils::to_polygon2d(target_object);
  if (boost::geometry::area(target_polygon) < MIN_AREA) return 0.0;

  const double intersection_area = getIntersectionArea(source_polygon, target_polygon);
  if (intersection_area < MIN_AREA) return 0.0;

  return std::min(1.0, intersection_area / source_area);
}

template <class T1, class T2>
double get2dRecall(const T1 source_object, const T2 target_object)
{
  const auto source_polygon = autoware_utils::to_polygon2d(source_object);
  if (boost::geometry::area(source_polygon) < MIN_AREA) return 0.0;
  const auto target_polygon = autoware_utils::to_polygon2d(target_object);
  const double target_area = boost::geometry::area(target_polygon);
  if (target_area < MIN_AREA) return 0.0;

  const double intersection_area = getIntersectionArea(source_polygon, target_polygon);
  if (intersection_area < MIN_AREA) return 0.0;

  return std::min(1.0, intersection_area / target_area);
}
}  // namespace autoware::object_recognition_utils

#endif  // AUTOWARE__OBJECT_RECOGNITION_UTILS__MATCHING_HPP_
