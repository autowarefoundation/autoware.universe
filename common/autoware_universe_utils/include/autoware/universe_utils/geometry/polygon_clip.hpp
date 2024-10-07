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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__POLYGON_CLIP_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__POLYGON_CLIP_HPP_

#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace autoware::universe_utils
{

struct Vertex
{
  double x;
  double y;
  std::optional<std::size_t> next;
  std::optional<std::size_t> prev;
  std::optional<std::size_t> corresponding;
  double distance;
  bool isEntry;
  bool isIntersection;
  bool visited;

  // Default constructor
  Vertex()
  : x(0.0),
    y(0.0),
    next(std::nullopt),
    prev(std::nullopt),
    corresponding(std::nullopt),
    distance(0.0),
    isEntry(true),
    isIntersection(false),
    visited(false)
  {
  }

  // Parameterized constructor
  Vertex(
    double xCoord, double yCoord, std::optional<std::size_t> nextIndex = std::nullopt,
    std::optional<std::size_t> prevIndex = std::nullopt,
    std::optional<std::size_t> correspondingIndex = std::nullopt, double dist = 0.0,
    bool entry = true, bool intersection = false, bool visitedState = false)
  : x(xCoord),
    y(yCoord),
    next(nextIndex),
    prev(prevIndex),
    corresponding(correspondingIndex),
    distance(dist),
    isEntry(entry),
    isIntersection(intersection),
    visited(visitedState)
  {
  }
};

struct Intersection
{
  double x, y, toSource, toClip;
};

struct Polygon
{
  std::size_t first;
  std::vector<Vertex> vertices;
  std::optional<std::size_t> lastUnprocessed;
  bool arrayVertices;
  std::optional<std::size_t> firstIntersect;

  // Default constructor
  Polygon()
  : first(0),
    vertices(0),
    lastUnprocessed(std::nullopt),
    arrayVertices(true),
    firstIntersect(std::nullopt)
  {
  }

  // Constructor
  explicit Polygon(bool arrayVertices)
  : first(0),
    vertices(0),
    lastUnprocessed(std::nullopt),
    arrayVertices(arrayVertices),
    firstIntersect(std::nullopt)
  {
  }
};

// Vertex methods
void visit(std::vector<Vertex> & vertices, std::vector<Vertex> & vertices_2, std::size_t index);
bool equals(const Vertex & v1, const Vertex & v2);
bool isInside(const Vertex & v, const Polygon & poly, const std::vector<Vertex> & vertices);

// Intersection methods
Intersection createIntersection(
  const std::vector<Vertex> & source_vertices, std::size_t s1Index, std::size_t s2Index,
  const std::vector<Vertex> & clip_vertices, std::size_t c1Index, std::size_t c2Index);
bool valid(const Intersection & intersection);

// Polygon methods
Polygon createPolygon(const std::vector<std::vector<double>> & p, bool arrayVertices = true);
std::size_t addVertex(
  Polygon & polygon, const Vertex & newVertex, const std::size_t last_index = 0);
std::size_t insertVertex(
  std::vector<Vertex> & vertices, const Vertex & vertex, std::size_t start, std::size_t end);
std::size_t getNext(std::size_t vIndex, const std::vector<Vertex> & vertices);
std::size_t getFirstIntersect(Polygon & polygon, const std::vector<Vertex> & vertices);
bool hasUnprocessed(Polygon & polygon);
std::vector<std::vector<double>> getPoints(const Polygon & polygon);
std::vector<std::vector<std::vector<double>>> clip(
  Polygon & source, Polygon & clip, bool sourceForwards, bool clipForwards);

// Function declarations
std::vector<std::vector<std::vector<double>>> difference(
  const std::vector<std::vector<double>> & polygonA,
  const std::vector<std::vector<double>> & polygonB);
std::vector<std::vector<std::vector<double>>> unionPolygons(
  const std::vector<std::vector<double>> & polygonA,
  const std::vector<std::vector<double>> & polygonB);

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__POLYGON_CLIP_HPP_
