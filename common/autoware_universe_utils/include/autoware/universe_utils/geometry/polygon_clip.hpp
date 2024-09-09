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
#include <vector>

namespace autoware::universe_utils
{

struct Vertex
{
  double x, y;
  Vertex * next;
  Vertex * prev;
  Vertex * corresponding;
  double distance;
  bool isEntry;
  bool isIntersection;
  bool visited;

  // Constructor with all parameters
  Vertex(
    double xCoord, double yCoord, Vertex * next = nullptr, Vertex * prev = nullptr,
    Vertex * corresponding = nullptr, double distance = 0.0, bool isEntry = true,
    bool isIntersection = false, bool visited = false)
  : x(xCoord),
    y(yCoord),
    next(next),
    prev(prev),
    corresponding(corresponding),
    distance(distance),
    isEntry(isEntry),
    isIntersection(isIntersection),
    visited(visited)
  {
  }
};

struct Intersection
{
  double x, y, toSource, toClip;
};

struct Polygon
{
  Vertex * first;
  int vertices;
  Vertex * lastUnprocessed;
  bool arrayVertices;
  Vertex * firstIntersect;
  // Default constructor
  Polygon()
  : first(nullptr),
    vertices(0),
    lastUnprocessed(nullptr),
    arrayVertices(true),
    firstIntersect(nullptr)
  {
  }

  // Constructor
  explicit Polygon(bool arrayVertices)
  : first(nullptr),
    vertices(0),
    lastUnprocessed(nullptr),
    arrayVertices(arrayVertices),
    firstIntersect(nullptr)
  {
  }
  {
  }
};

// Vertex methods
Vertex * createIntersection(double x, double y, double distance);
void visit(Vertex * vertex);
bool equals(const Vertex * v1, const Vertex * v2);
bool isInside(const Vertex * v, const Polygon & poly);

// Intersection methods
Intersection createIntersection(
  const Vertex * s1, const Vertex * s2, const Vertex * c1, const Vertex * c2);
bool valid(const Intersection & intersection);

// Polygon methods
Polygon createPolygon(const std::vector<std::vector<double>> & p, bool arrayVertices = true);
void addVertex(Polygon & polygon, Vertex * vertex);
void insertVertex(Polygon & polygon, Vertex * vertex, Vertex * start, Vertex * end);
Vertex * getNext(Vertex * v);
Vertex * getFirstIntersect(Polygon & polygon);
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
