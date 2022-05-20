// Copyright 2022 Tier IV, Inc.
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

#ifndef GRID_MAP_UTILS__POLYGON_ITERATOR_HPP_
#define GRID_MAP_UTILS__POLYGON_ITERATOR_HPP_

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/Polygon.hpp>

#include <vector>

namespace grid_map_utils
{

/// @brief A polygon iterator for grid_map::GridMap based on the scan line algorithm.
/** @details This iterator allows to iterate over all cells whose center is inside a polygon. \
             This reproduces the behavior of the original grid_map::PolygonIterator which uses\
             a "point in polygon" check for each cell of the gridmap, making it very expensive\
             to run on large maps. In comparison, this implementation uses
             that is much more scalable.
*/
class PolygonIterator
{
public:
  /// @brief Constructor.
  /// @details Calculate the indexes of the gridmap that are inside the polygon using the scan line
  /// algorithm.
  /// @param grid_map the grid map to iterate on.
  /// @param polygon the polygonal area to iterate on.
  PolygonIterator(const grid_map::GridMap & grid_map, const grid_map::Polygon & polygon);
  /// @brief Compare to another iterator.
  /// @param other other iterator.
  /// @return whether the current iterator points to a different address than the other one.
  bool operator!=(const PolygonIterator & other) const;
  /// @brief Dereference the iterator with const.
  /// @return the value to which the iterator is pointing.
  const grid_map::Index & operator*() const;
  /// @brief Increase the iterator to the next element.
  /// @return a reference to the updated iterator.
  PolygonIterator & operator++();
  /// @brief Indicates if iterator is past end.
  /// @return true if iterator is out of scope, false if end has not been reached.
  [[nodiscard]] bool isPastEnd() const;

private:
  /// calculated indexes of the gridmap that are inside the polygon
  std::vector<grid_map::Index> polygon_indexes_;
  /// current index of the iterator
  size_t current_index = 0;
};
}  // namespace grid_map_utils

#endif  // GRID_MAP_UTILS__POLYGON_ITERATOR_HPP_
