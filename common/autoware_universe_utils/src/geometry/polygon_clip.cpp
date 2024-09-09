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

namespace autoware::universe_utils
{

// Vertex methods
Vertex * createIntersection(double x, double y, double distance)
{
  Vertex * vertex = new Vertex(x, y);  // Provide x and y to the constructor
  vertex->distance = distance;
  vertex->isIntersection = true;
  vertex->isEntry = false;
  return vertex;
}

void visit(Vertex * vertex)
{
  vertex->visited = true;
  if (vertex->corresponding && !vertex->corresponding->visited) {
    visit(vertex->corresponding);
  }
}

bool equals(const Vertex * v1, const Vertex * v2)
{
  return v1->x == v2->x && v1->y == v2->y;
}

bool isInside(const Vertex * v, const Polygon & poly)
{
  bool contains = false;
  int32_t winding_num = 0;
  constexpr double tolerance = 1e-9;

  Vertex * vertex = poly.first;
  Vertex * next = vertex->next;

  do {
    bool y_intersects =
      ((next->y < v->y) != (vertex->y < v->y)) &&
      (v->x < (vertex->x - next->x) * (v->y - next->y) / (vertex->y - next->y) + next->x);

    if (y_intersects) {
      contains = !contains;

      if (std::abs(vertex->x - next->x) < tolerance && std::abs(vertex->y - next->y) < tolerance) {
        // Skip zero-length edges
        vertex = next;
        next = vertex->next ? vertex->next : poly.first;
        continue;
      }

      if (vertex->x < next->x - tolerance) {
        winding_num += 1;
      } else if (vertex->x > next->x + tolerance) {
        winding_num -= 1;
      }
    }

    vertex = next;
    next = vertex->next ? vertex->next : poly.first;
  } while (vertex != poly.first);

  // Determine if the point is inside based on winding number
  return contains;
}

Intersection createIntersection(
  const Vertex * s1, const Vertex * s2, const Vertex * c1, const Vertex * c2)
{
  Intersection intersection;

  // Calculate the determinant
  double d = (c2->y - c1->y) * (s2->x - s1->x) - (c2->x - c1->x) * (s2->y - s1->y);

  if (std::abs(d) > 1e-9) {  // Use a tolerance to handle precision issues
    double t1 = ((c2->x - c1->x) * (s1->y - c1->y) - (c2->y - c1->y) * (s1->x - c1->x)) / d;
    double t2 = ((s2->x - s1->x) * (s1->y - c1->y) - (s2->y - s1->y) * (s1->x - c1->x)) / d;

    if (t1 >= 0 && t1 <= 1 && t2 >= 0 && t2 <= 1) {
      intersection.x = s1->x + t1 * (s2->x - s1->x);
      intersection.y = s1->y + t1 * (s2->y - s1->y);
      intersection.toSource = t1;
      intersection.toClip = t2;

      // Debug output
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

Polygon createPolygon(const std::vector<std::vector<double>> & p, bool arrayVertices)
{
  Polygon polygon;
  polygon.arrayVertices = arrayVertices;
  for (const auto & point : p) {
    addVertex(
      polygon, new Vertex{point[0], point[1], nullptr, nullptr, nullptr, 0.0, true, false, false});
  }
  return polygon;
}

void addVertex(Polygon & polygon, Vertex * vertex)
{
  if (polygon.first == nullptr) {
    polygon.first = vertex;
    polygon.first->next = vertex;
    polygon.first->prev = vertex;
  } else {
    Vertex * next = polygon.first;
    Vertex * prev = next->prev;

    next->prev = vertex;
    vertex->next = next;
    vertex->prev = prev;
    prev->next = vertex;
  }
  polygon.vertices++;
}

void insertVertex(Polygon & polygon, Vertex * vertex, Vertex * start, Vertex * end)
{
  Vertex * curr = start;
  while (!equals(curr, end) && curr->distance < vertex->distance) {
    curr = curr->next;
  }

  vertex->next = curr;
  Vertex * prev = curr->prev;

  vertex->prev = prev;
  prev->next = vertex;
  curr->prev = vertex;

  polygon.vertices++;
}

Vertex * getNext(Vertex * v)
{
  Vertex * c = v;
  while (c->isIntersection) c = c->next;
  return c;
}

Vertex * getFirstIntersect(Polygon & polygon)
{
  Vertex * v = polygon.firstIntersect ? polygon.firstIntersect : polygon.first;

  do {
    if (v->isIntersection && !v->visited) break;
    v = v->next;
  } while (!equals(v, polygon.first));

  polygon.firstIntersect = v;
  return v;
}

bool hasUnprocessed(Polygon & polygon)
{
  Vertex * v = polygon.lastUnprocessed ? polygon.lastUnprocessed : polygon.first;
  do {
    if (v->isIntersection && !v->visited) {
      polygon.lastUnprocessed = v;
      return true;
    }
    v = v->next;
  } while (!equals(v, polygon.first));

  polygon.lastUnprocessed = nullptr;
  return false;
}

std::vector<std::vector<double>> getPoints(const Polygon & polygon)
{
  Vertex * v = polygon.first;
  std::vector<std::vector<double>> points;
  do {
    points.push_back({v->x, v->y});
    v = v->next;
  } while (v != polygon.first);

  return points;
}

std::vector<std::vector<std::vector<double>>> clip(
  Polygon & source, Polygon & clip, bool sourceForwards, bool clipForwards)
{
  Vertex * sourceVertex = source.first;
  Vertex * clipVertex = clip.first;
  bool sourceInClip, clipInSource;

  bool isUnion = !sourceForwards && !clipForwards;
  bool isIntersection = sourceForwards && clipForwards;
  do {
    if (!sourceVertex->isIntersection) {
      do {
        if (!clipVertex->isIntersection) {
          Intersection i = createIntersection(
            sourceVertex, getNext(sourceVertex->next), clipVertex, getNext(clipVertex->next));

          if (valid(i)) {
            Vertex * sourceIntersection = createIntersection(i.x, i.y, i.toSource);
            Vertex * clipIntersection = createIntersection(i.x, i.y, i.toClip);

            sourceIntersection->corresponding = clipIntersection;
            clipIntersection->corresponding = sourceIntersection;

            insertVertex(source, sourceIntersection, sourceVertex, getNext(sourceVertex->next));
            insertVertex(clip, clipIntersection, clipVertex, getNext(clipVertex->next));
          }
        }
        clipVertex = clipVertex->next;
      } while (!equals(clipVertex, clip.first));
    }

    sourceVertex = sourceVertex->next;
  } while (!equals(sourceVertex, source.first));

  sourceVertex = source.first;
  clipVertex = clip.first;

  sourceInClip = isInside(sourceVertex, clip);
  clipInSource = isInside(clipVertex, source);

  sourceForwards ^= sourceInClip;
  clipForwards ^= clipInSource;

  do {
    if (sourceVertex->isIntersection) {
      sourceVertex->isEntry = sourceForwards;
      sourceForwards = !sourceForwards;
    }
    sourceVertex = sourceVertex->next;
  } while (!equals(sourceVertex, source.first));

  do {
    if (clipVertex->isIntersection) {
      clipVertex->isEntry = clipForwards;
      clipForwards = !clipForwards;
    }
    clipVertex = clipVertex->next;
  } while (!equals(clipVertex, clip.first));

  std::vector<std::vector<std::vector<double>>> list;

  while (hasUnprocessed(source)) {
    Vertex * current = getFirstIntersect(source);
    Polygon clipped({}, source.arrayVertices);

    addVertex(clipped, new Vertex{current->x, current->y});
    do {
      visit(current);
      if (current->isEntry) {
        do {
          current = current->next;
          addVertex(clipped, new Vertex{current->x, current->y});
        } while (!current->isIntersection);
      } else {
        do {
          current = current->prev;
          addVertex(clipped, new Vertex{current->x, current->y});
        } while (!current->isIntersection);
      }
      current = current->corresponding;
    } while (!current->visited);

    list.push_back(getPoints(clipped));
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
