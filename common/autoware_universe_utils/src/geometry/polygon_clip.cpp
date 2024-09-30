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

#include "autoware/universe_utils/geometry/polygon_clip.hpp"

#include <iostream>

namespace autoware::universe_utils
{

// Vertex methods
void visit(std::vector<Vertex> & vertices, std::vector<Vertex> & vertices_2, std::size_t index)
{
  Vertex & vertex = vertices[index];
  vertex.visited = true;

  if (vertex.corresponding.has_value()) {
    std::size_t correspondingIndex = *vertex.corresponding;

    if (!vertices_2[correspondingIndex].visited) {
      visit(vertices_2, vertices, correspondingIndex);
    }
  }
}

bool equals(const Vertex & v1, const Vertex & v2)
{
  return v1.x == v2.x && v1.y == v2.y;
}

bool isInside(const Vertex & v, const Polygon & poly)
{
  bool contains = false;
  int32_t winding_num = 0;
  constexpr double tolerance = 1e-9;

  std::size_t vertexIndex = poly.first;
  std::size_t nextIndex = poly.vertices[vertexIndex].next.value_or(poly.first);

  do {
    const Vertex & vertex = poly.vertices[vertexIndex];
    const Vertex & next = poly.vertices[nextIndex];

    bool y_intersects = ((next.y < v.y) != (vertex.y < v.y)) &&
                        (v.x < (vertex.x - next.x) * (v.y - next.y) / (vertex.y - next.y) + next.x);

    if (y_intersects) {
      contains = !contains;

      if (std::abs(vertex.x - next.x) < tolerance && std::abs(vertex.y - next.y) < tolerance) {
        vertexIndex = nextIndex;
        nextIndex = poly.vertices[vertexIndex].next.value_or(poly.first);
        continue;
      }

      if (vertex.x < next.x - tolerance) {
        winding_num += 1;
      } else if (vertex.x > next.x + tolerance) {
        winding_num -= 1;
      }
    }

    vertexIndex = nextIndex;
    nextIndex = poly.vertices[vertexIndex].next.value_or(poly.first);
  } while (vertexIndex != poly.first);

  return contains;
}

Intersection createIntersection(
  const std::vector<Vertex> & source_vertices, std::size_t s1Index, std::size_t s2Index,
  const std::vector<Vertex> & clip_vertices, std::size_t c1Index, std::size_t c2Index)
{
  Intersection intersection;

  const Vertex & s1 = source_vertices[s1Index];
  const Vertex & s2 = source_vertices[s2Index];
  const Vertex & c1 = clip_vertices[c1Index];
  const Vertex & c2 = clip_vertices[c2Index];

  double d = (c2.y - c1.y) * (s2.x - s1.x) - (c2.x - c1.x) * (s2.y - s1.y);

  if (std::abs(d) > 1e-9) {
    double t1 = ((c2.x - c1.x) * (s1.y - c1.y) - (c2.y - c1.y) * (s1.x - c1.x)) / d;
    double t2 = ((s2.x - s1.x) * (s1.y - c1.y) - (s2.y - s1.y) * (s1.x - c1.x)) / d;

    if (t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1) {
      intersection.x = s1.x + t1 * (s2.x - s1.x);
      intersection.y = s1.y + t1 * (s2.y - s1.y);
      intersection.toSource = t1;
      intersection.toClip = t2;
      return intersection;
    }
  }

  intersection.x = std::numeric_limits<double>::quiet_NaN();
  intersection.y = std::numeric_limits<double>::quiet_NaN();
  intersection.toSource = 0;
  intersection.toClip = 0;

  return intersection;
}

bool valid(const Intersection & intersection)
{
  return (intersection.toSource > 0 && intersection.toSource < 1) &&
         (intersection.toClip > 0 && intersection.toClip < 1);
}

std::size_t addVertex(Polygon & polygon, const Vertex & newVertex, const std::size_t last_index)
{
  std::size_t p_idx = polygon.vertices.size();
  polygon.vertices.push_back(newVertex);
  if (!polygon.vertices.empty()) {
    std::size_t last = last_index;
    std::size_t next = polygon.vertices[last].next.value();
    polygon.vertices[p_idx].prev = last;
    polygon.vertices[p_idx].next = next;
    polygon.vertices[last].next = p_idx;
    if (next != p_idx) {
      polygon.vertices[next].prev = p_idx;
    }
  }
  return p_idx;
}

Polygon createPolygon(const std::vector<std::vector<double>> & p, bool arrayVertices)
{
  Polygon polygon;
  polygon.arrayVertices = arrayVertices;

  if (p.empty()) return polygon;

  polygon.vertices.resize(p.size());

  for (std::size_t i = 0; i < p.size(); ++i) {
    Vertex vertex{p[i][0], p[i][1]};

    vertex.prev = (i == 0) ? p.size() - 1 : i - 1;

    vertex.next = (i + 1) % p.size();

    polygon.vertices[i] = vertex;
  }

  return polygon;
}

Polygon createPolygon(const Vertex & vertex, bool arrayVertices)
{
  Polygon polygon;
  polygon.arrayVertices = arrayVertices;

  polygon.vertices.push_back(vertex);

  polygon.vertices.back().prev = 0;
  polygon.vertices.back().next = 0;

  return polygon;
}

void insertVertex(
  std::vector<Vertex> & vertices, Vertex & vertex, std::size_t startIndex, std::size_t endIndex)
{
  std::size_t currIndex = startIndex;

  while (currIndex != endIndex && vertices[currIndex].distance < vertex.distance) {
    currIndex = vertices[currIndex].next.value();
  }

  vertex.next = currIndex;
  vertex.prev = vertices[currIndex].prev.value();
  std::size_t prevIndex = vertex.prev.value();

  if (prevIndex != currIndex) {
    vertices[prevIndex].next = &vertex - &vertices[0];
  }
  vertices[currIndex].prev = &vertex - &vertices[0];

  if (currIndex == startIndex) {
    vertex.prev = startIndex;
    vertex.next = startIndex;
  }
}

std::size_t getNext(std::size_t index, const std::vector<Vertex> & vertices)
{
  std::size_t currIndex = index;
  while (vertices[currIndex].isIntersection) {
    currIndex = vertices[currIndex].next.value();
  }
  return currIndex;
}

std::size_t getFirstIntersect(Polygon & polygon)
{
  std::size_t v =
    polygon.firstIntersect.has_value() ? polygon.firstIntersect.value() : polygon.first;

  do {
    if (polygon.vertices[v].isIntersection && !polygon.vertices[v].visited) break;
    v = polygon.vertices[v].next.value();
  } while (v != polygon.first);

  polygon.firstIntersect = v;
  return v;
}

bool hasUnprocessed(Polygon & polygon)
{
  std::size_t v =
    polygon.lastUnprocessed.has_value() ? polygon.lastUnprocessed.value() : polygon.first;

  do {
    if (polygon.vertices[v].isIntersection && !polygon.vertices[v].visited) {
      polygon.lastUnprocessed = v;
      return true;
    }
    v = polygon.vertices[v].next.value();
  } while (v != polygon.first);
  polygon.lastUnprocessed = std::nullopt;
  return false;
}

std::vector<std::vector<double>> getPoints(const Polygon & polygon)
{
  std::vector<std::vector<double>> points;

  std::size_t vIndex = polygon.first;
  std::size_t startIndex = vIndex;

  do {
    const auto & vertex = polygon.vertices[vIndex];
    points.push_back({vertex.x, vertex.y});

    vIndex = vertex.next.value();
  } while (vIndex != startIndex);

  return points;
}

std::vector<std::vector<std::vector<double>>> clip(
  Polygon & source, Polygon & clip, bool sourceForwards, bool clipForwards)
{
  std::size_t sourceVertexIndex = source.first;
  std::size_t clipVertexIndex = clip.first;
  bool sourceInClip, clipInSource;

  bool isUnion = !sourceForwards && !clipForwards;
  bool isIntersection = sourceForwards && clipForwards;

  do {
    if (!source.vertices[sourceVertexIndex].isIntersection) {
      do {
        if (!clip.vertices[clipVertexIndex].isIntersection) {
          Intersection i = createIntersection(
            source.vertices, sourceVertexIndex,
            getNext(source.vertices[sourceVertexIndex].next.value(), source.vertices),
            clip.vertices, clipVertexIndex,
            getNext(clip.vertices[clipVertexIndex].next.value(), clip.vertices));

          if (valid(i)) {
            Vertex intersectionVertex1{i.x,        i.y,   std::nullopt, std::nullopt, std::nullopt,
                                       i.toSource, false, true,         false};
            Vertex intersectionVertex2{i.x,      i.y,   std::nullopt, std::nullopt, std::nullopt,
                                       i.toClip, false, true,         false};

            source.vertices.push_back(intersectionVertex1);
            clip.vertices.push_back(intersectionVertex2);

            std::size_t index1 = source.vertices.size() - 1;
            std::size_t index2 = clip.vertices.size() - 1;

            source.vertices[index1].corresponding = index2;
            clip.vertices[index2].corresponding = index1;

            insertVertex(
              source.vertices, source.vertices[index1], sourceVertexIndex,
              getNext(source.vertices[sourceVertexIndex].next.value(), source.vertices));
            insertVertex(
              clip.vertices, clip.vertices[index2], clipVertexIndex,
              getNext(clip.vertices[clipVertexIndex].next.value(), clip.vertices));
          }
        }

        clipVertexIndex = clip.vertices[clipVertexIndex].next.value();
      } while (clipVertexIndex != clip.first);
    }

    sourceVertexIndex = source.vertices[sourceVertexIndex].next.value();
  } while (sourceVertexIndex != source.first);

  sourceVertexIndex = source.first;
  clipVertexIndex = clip.first;

  sourceInClip = isInside(source.vertices[sourceVertexIndex], clip);
  clipInSource = isInside(clip.vertices[clipVertexIndex], source);

  sourceForwards ^= sourceInClip;
  clipForwards ^= clipInSource;

  do {
    if (source.vertices[sourceVertexIndex].isIntersection) {
      source.vertices[sourceVertexIndex].isEntry = sourceForwards;
      sourceForwards = !sourceForwards;
    }
    sourceVertexIndex = source.vertices[sourceVertexIndex].next.value();
  } while (sourceVertexIndex != source.first);

  do {
    if (clip.vertices[clipVertexIndex].isIntersection) {
      clip.vertices[clipVertexIndex].isEntry = clipForwards;
      clipForwards = !clipForwards;
    }
    clipVertexIndex = clip.vertices[clipVertexIndex].next.value();
  } while (clipVertexIndex != clip.first);

  std::vector<std::vector<std::vector<double>>> list;

  while (hasUnprocessed(source)) {
    std::size_t currentIndex = getFirstIntersect(source);
    Polygon clipped = createPolygon(source.vertices[currentIndex], source.arrayVertices);
    std::size_t last_idx = 0;
    bool usingSource = true;

    do {
      visit(source.vertices, clip.vertices, currentIndex);

      if (usingSource) {
        if (source.vertices[currentIndex].isEntry) {
          do {
            currentIndex = source.vertices[currentIndex].next.value();
            last_idx = addVertex(clipped, source.vertices[currentIndex], last_idx);
          } while (!source.vertices[currentIndex].isIntersection);
        } else {
          do {
            currentIndex = source.vertices[currentIndex].prev.value();
            last_idx = addVertex(clipped, source.vertices[currentIndex], last_idx);
          } while (!source.vertices[currentIndex].isIntersection);
        }
      } else {
        if (clip.vertices[currentIndex].isEntry) {
          do {
            currentIndex = clip.vertices[currentIndex].next.value();
            last_idx = addVertex(clipped, clip.vertices[currentIndex], last_idx);
          } while (!clip.vertices[currentIndex].isIntersection);
        } else {
          do {
            currentIndex = clip.vertices[currentIndex].prev.value();
            last_idx = addVertex(clipped, clip.vertices[currentIndex], last_idx);
          } while (!clip.vertices[currentIndex].isIntersection);
        }
      }
      currentIndex = (usingSource ? source.vertices[currentIndex] : clip.vertices[currentIndex])
                       .corresponding.value();
      usingSource = !usingSource;
    } while (
      !((usingSource ? source.vertices[currentIndex] : clip.vertices[currentIndex]).visited));

    auto points = getPoints(clipped);
    list.push_back(points);
  }

  if (list.empty()) {
    if (isUnion) {
      if (sourceInClip) {
        list.push_back(getPoints(clip));
      } else if (clipInSource) {
        list.push_back(getPoints(source));
      } else {
        list.push_back(getPoints(source));
        list.push_back(getPoints(clip));
      }
    } else if (isIntersection) {
      if (sourceInClip) {
        list.push_back(getPoints(source));
      } else if (clipInSource) {
        list.push_back(getPoints(clip));
      }
    } else {  // Difference
      if (sourceInClip) {
        list.push_back(getPoints(clip));
        list.push_back(getPoints(source));
      } else if (clipInSource) {
        list.push_back(getPoints(source));
        list.push_back(getPoints(clip));
      } else {
        list.push_back(getPoints(source));
      }
    }
  }
  return list;
}

// Difference function
std::vector<std::vector<std::vector<double>>> difference(
  const std::vector<std::vector<double>> & polygonA,
  const std::vector<std::vector<double>> & polygonB)
{
  Polygon polyA = createPolygon(polygonA);
  Polygon polyB = createPolygon(polygonB);
  return clip(polyA, polyB, false, true);
}

// Union function
std::vector<std::vector<std::vector<double>>> unionPolygons(
  const std::vector<std::vector<double>> & polygonA,
  const std::vector<std::vector<double>> & polygonB)
{
  Polygon polyA = createPolygon(polygonA);
  Polygon polyB = createPolygon(polygonB);
  return clip(polyA, polyB, false, false);
}

}  // namespace autoware::universe_utils
