// Copyright 2024 Tier IV, Inc.
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

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::universe_utils
{

class Earclipping
{
public:
  std::vector<std::size_t> indices;
  std::size_t vertices = 0;
  using Polygon2d = autoware::universe_utils::Polygon2d;
  using Point2d = autoware::universe_utils::Point2d;
  using LinearRing2d = autoware::universe_utils::LinearRing2d;
  void operator()(const Polygon2d & points);

private:
  struct Point
  {
    Point(std::size_t index, const Point2d & point) : i(index), pt(point) {}

    Point(const Point &) = delete;
    Point & operator=(const Point &) = delete;
    Point(Point &&) = delete;
    Point & operator=(Point &&) = delete;

    const std::size_t i;  // Index of the point in the original polygon
    const Point2d pt;     // The Point2d object representing the coordinates

    // Previous and next vertices (Points) in the polygon ring
    Point * prev = nullptr;
    Point * next = nullptr;
    bool steiner = false;

    double x() const { return pt.x(); }
    double y() const { return pt.y(); }
  };

  Point * linkedList(const LinearRing2d & points, bool clockwise);
  Point * filterPoints(Point * start, Point * end = nullptr);
  Point * cureLocalIntersections(Point * start);
  Point * getLeftmost(Point * start);
  Point * splitPolygon(Point * a, Point * b);
  Point * insertPoint(std::size_t i, const Point2d & p, Point * last);
  Point * eliminateHoles(const std::vector<LinearRing2d> & points, Point * outerPoint);
  Point * eliminateHole(Point * hole, Point * outerPoint);
  Point * findHoleBridge(Point * hole, Point * outerPoint);
  void earclippingLinked(Point * ear, int pass = 0);
  void splitearclipping(Point * start);
  void indexCurve(Point * start);
  void removePoint(Point * p);
  bool isEar(Point * ear);
  bool sectorContainsSector(const Point * m, const Point * p);
  bool pointInTriangle(
    double ax, double ay, double bx, double by, double cx, double cy, double px, double py) const;
  bool isValidDiagonal(Point * a, Point * b);
  bool equals(const Point * p1, const Point * p2);
  bool intersects(const Point * p1, const Point * q1, const Point * p2, const Point * q2);
  bool onSegment(const Point * p, const Point * q, const Point * r);
  bool intersectsPolygon(const Point * a, const Point * b);
  bool locallyInside(const Point * a, const Point * b);
  bool middleInside(const Point * a, const Point * b);
  int sign(double val);
  double area(const Point * p, const Point * q, const Point * r) const;
  double minX, maxX;
  double minY, maxY;
  double inv_size = 0;

  class ObjectPool
  {
  public:
    ObjectPool() = default;
    ObjectPool(std::size_t blockSize_) { reset(blockSize_); }
    ~ObjectPool() { clear(); }

    // Construct a new Point object in the pool
    template <typename... Args>
    Point * construct(Args &&... args)
    {
      if (currentIndex >= blockSize) {
        currentBlock = alloc_traits::allocate(alloc, blockSize);
        allocations.emplace_back(currentBlock);
        currentIndex = 0;
      }
      Point * object = &currentBlock[currentIndex++];
      alloc_traits::construct(alloc, object, std::forward<Args>(args)...);
      return object;
    }

    // Reset the pool with a new block size
    void reset(std::size_t newBlockSize)
    {
      for (auto allocation : allocations) {
        alloc_traits::deallocate(alloc, allocation, blockSize);
      }
      allocations.clear();
      blockSize = std::max<std::size_t>(1, newBlockSize);
      currentBlock = nullptr;
      currentIndex = blockSize;
    }

    // Clear the pool and reset
    void clear() { reset(blockSize); }

  private:
    Point * currentBlock = nullptr;    // Pointer to the current block of memory
    std::size_t currentIndex = 1;      // Current index in the block
    std::size_t blockSize = 1;         // Size of each block
    std::vector<Point *> allocations;  // Track allocated blocks
    std::allocator<Point> alloc;       // Allocator for Point
    typedef std::allocator_traits<std::allocator<Point>> alloc_traits;  // Traits for allocator
  };
  ObjectPool Points;
};

/// @brief Triangulate based on earclipping algorithm
/// @param polyogn concave/convex polygon with/without holes
/// @details algorithm based on https://github.com/mapbox/earclipping with modification
std::vector<autoware::universe_utils::Polygon2d> triangulate(
  const autoware::universe_utils::Polygon2d & poly);

}  // namespace autoware::universe_utils
