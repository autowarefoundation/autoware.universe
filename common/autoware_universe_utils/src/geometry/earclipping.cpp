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

#include "autoware/universe_utils/geometry/earclipping.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

#ifndef EAR_CUT_IMPL_HPP
#define EAR_CUT_IMPL_HPP

namespace autoware::universe_utils::earclipping {

void Earclipping::operator()(const Polygon2d& polygon) {
    indices.clear();
    vertices = 0;

    if (polygon.outer().size() == 0) return;

    std::size_t len;

    auto outer_ring = polygon.outer();
    len = outer_ring.size();
    Points.reset(len * 3 / 2);
    indices.reserve(len + outer_ring.size());

    Earclipping::Point* outerPoint = linkedList(outer_ring, true);
    if (!outerPoint || outerPoint->prev == outerPoint->next) return;
    if (polygon.inners().size() > 0) outerPoint = eliminateHoles(polygon.inners(), outerPoint);
    earclippingLinked(outerPoint);
    Points.clear();
}

/// @brief create a circular doubly linked list from polygon points in the specified winding order
Earclipping::Point* Earclipping::linkedList(const LinearRing2d& points, bool clockwise) {
    double sum = 0;
    const std::size_t len = points.size();
    std::size_t i, j;
    Earclipping::Point* last = nullptr;

    for (i = 0, j = len > 0 ? len - 1 : 0; i < len; j = i++) {
        const auto& p1 = points[i];
        const auto& p2 = points[j];
        const double p10 = p1.x();
        const double p11 = p1.y();
        const double p20 = p2.x();
        const double p21 = p2.y();
        sum += (p20 - p10) * (p11 + p21);
    }

    if (clockwise == (sum > 0)) {
        for (i = 0; i < len; i++) {
            last = insertPoint(vertices + i, points[i], last);
        }
    } else {
        for (i = len; i-- > 0;) {
            last = insertPoint(vertices + i, points[i], last);
        }
    }

    if (last && equals(last, last->next)) {
        removePoint(last);
        last = last->next;
    }

    vertices += len;

    return last;
}

/// @brief eliminate colinear or duplicate points
Earclipping::Point* Earclipping::filterPoints(Earclipping::Point* start, Earclipping::Point* end) {
    if (!end) end = start;

    Earclipping::Point* p = start;
    bool again;
    do {
        again = false;

        if (!p->steiner && (equals(p, p->next) || area(p->prev, p, p->next) == 0)) {
            removePoint(p);
            p = end = p->prev;

            if (p == p->next) break;
            again = true;

        } else {
            p = p->next;
        }
    } while (again || p != end);

    return end;
}

/// @brief find a bridge between vertices that connects hole with an outer ring and and link it
Earclipping::Point* Earclipping::eliminateHole(Earclipping::Point* hole, Earclipping::Point* outerNode) {
    Earclipping::Point* bridge = findHoleBridge(hole, outerNode);
    if (!bridge) {
        return outerNode;
    }
    Earclipping::Point* bridgeReverse = splitPolygon(bridge, hole);

    // filter collinear points around the cuts
    filterPoints(bridgeReverse, bridgeReverse->next);

    // Check if input node was removed by the filtering
    return filterPoints(bridge, bridge->next);
}

Earclipping::Point* Earclipping::eliminateHoles(const std::vector<LinearRing2d>& inners, Earclipping::Point* outerPoint) {
    const size_t len = inners.size();

    std::vector<Point*> queue;
    for (size_t i = 0; i < len; i++) {
        Point* list = linkedList(inners[i], false);
        if (list) {
            if (list == list->next) list->steiner = true;
            queue.push_back(getLeftmost(list));
        }
    }
    std::sort(queue.begin(), queue.end(), [](const Point* a, const Point* b) {
        return a->x() < b->x();
    });
    for (size_t i = 0; i < queue.size(); i++) {
        outerPoint = eliminateHole(queue[i], outerPoint);
    }

    return outerPoint;
}

/// @brief David Eberly's algorithm for finding a bridge between hole and outer polygon
Earclipping::Point*  Earclipping::findHoleBridge(Point* hole, Point* outerPoint) {
    Point* p = outerPoint;
    double hx = hole->x();
    double hy = hole->y();
    double qx = -std::numeric_limits<double>::infinity();
    Point* m = nullptr;
    do {
        if (hy <= p->y() && hy >= p->next->y() && p->next->y() != p->y()) {
          double x = p->x() + (hy - p->y()) * (p->next->x() - p->x())/ (p->next->y() - p->y());
          if (x <= hx && x > qx) {
            qx = x;
            m = p->x() < p->next->x() ? p : p->next;
            if (x == hx) return m; 
          }
        }
        p = p->next;
    } while (p != outerPoint);

    if (!m) return 0;

    const Point* stop = m;
    double tanMin = std::numeric_limits<double>::infinity();
    double tanCur = 0;

    p = m;
    double mx = m->x();
    double my = m->y();

    do {
        if (hx >= p->x() && p->x() >= mx && hx != p->x() &&
            pointInTriangle(hy < my ? hx : qx, hy, mx, my, hy < my ? qx : hx, hy, p->x(), p->y())) {

            tanCur = std::abs(hy - p->y()) / (hx - p->x()); 

            if (locallyInside(p, hole) &&
                (tanCur < tanMin || (tanCur == tanMin && (p->x() > m->x() || sectorContainsSector(m, p))))) {
                m = p;
                tanMin = tanCur;
            }
        }

        p = p->next;
    } while (p != stop);

    return m;
}

/// @brief main ear slicing loop which triangulates a polygon using linked list
void Earclipping::earclippingLinked(Earclipping::Point* ear, int pass) {
    if (!ear) return;

    Earclipping::Point* stop = ear;
    Earclipping::Point* next;

    // Iterate through ears, slicing them one by one
    while (ear->prev != ear->next) {
        next = ear->next;

        if (isEar(ear)) {
            // Cut off the triangle
            indices.emplace_back(ear->prev->i);
            indices.emplace_back(ear->i);
            indices.emplace_back(next->i);

            removePoint(ear);
            ear = next->next;
            stop = next->next;

            continue;
        }

        ear = next;
        if (ear == stop) {
            if (!pass) earclippingLinked(filterPoints(ear), 1);

            else if (pass == 1) {
                ear = cureLocalIntersections(filterPoints(ear));
                earclippingLinked(ear, 2);

            } else if (pass == 2) splitearclipping(ear);

            break;
        }
    }
}

/// @brief check whether ear is valid
bool Earclipping::isEar(Earclipping::Point* ear) {
    const Earclipping::Point* a = ear->prev;
    const Earclipping::Point* b = ear;
    const Earclipping::Point* c = ear->next;

    if (area(a, b, c) >= 0) return false; // Reflex, can't be an ear

    Earclipping::Point* p = ear->next->next;

    while (p != ear->prev) {
        if (pointInTriangle(a->x(), a->y(), b->x(), b->y(), c->x(), c->y(), p->x(), p->y()) &&
            area(p->prev, p, p->next) >= 0) return false;
        p = p->next;
    }

    return true;
}

/// @brief go through all polygon Points and cure small local self-intersections
Earclipping::Point* Earclipping::cureLocalIntersections(Earclipping::Point* start) {
    Earclipping::Point* p = start;
    do {
        Earclipping::Point* a = p->prev;
        Earclipping::Point* b = p->next->next;

        if (!equals(a, b) && intersects(a, p, p->next, b) && locallyInside(a, b) && locallyInside(b, a)) {
            indices.emplace_back(a->i);
            indices.emplace_back(p->i);
            indices.emplace_back(b->i);
            removePoint(p);
            removePoint(p->next);

            p = start = b;
        }
        p = p->next;
    } while (p != start);

    return filterPoints(p);
}

/// @brief splitting polygon into two and triangulate them independently
void Earclipping::splitearclipping(Earclipping::Point* start) {
    Earclipping::Point* a = start;
    do {
        Earclipping::Point* b = a->next->next;
        while (b != a->prev) {
            if (a->i != b->i && isValidDiagonal(a, b)) {
                Earclipping::Point* c = splitPolygon(a, b);

                a = filterPoints(a, a->next);
                c = filterPoints(c, c->next);

                earclippingLinked(a);
                earclippingLinked(c);
                return;
            }
            b = b->next;
        }
        a = a->next;
    } while (a != start);
}

/// @brief check whether sector in vertex m contains sector in vertex p in the same coordinates
bool Earclipping::sectorContainsSector(const Earclipping::Point* m, const Earclipping::Point* p) {
    return area(m->prev, m, p->prev) < 0 && area(p->next, m, m->next) < 0;
}

/// @brief find the leftmost Point of a polygon ring
Earclipping::Point* Earclipping::getLeftmost(Earclipping::Point* start) {
    Earclipping::Point* p = start;
    Earclipping::Point* leftmost = start;
    do {
        if (p->x() < leftmost->x() || (p->x() == leftmost->x() && p->y() < leftmost->y()))
            leftmost = p;
        p = p->next;
    } while (p != start);

    return leftmost;
}

/// @brief check if a point lies within a convex triangle
bool Earclipping::pointInTriangle(double ax, double ay, double bx, double by, double cx, double cy, double px, double py) const {
    return (cx - px) * (ay - py) >= (ax - px) * (cy - py) &&
           (ax - px) * (by - py) >= (bx - px) * (ay - py) &&
           (bx - px) * (cy - py) >= (cx - px) * (by - py);
}

/// @brief check if a diagonal between two polygon Points is valid
bool Earclipping::isValidDiagonal(Earclipping::Point* a, Earclipping::Point* b) {
    return a->next->i != b->i && a->prev->i != b->i && !intersectsPolygon(a, b) &&
           ((locallyInside(a, b) && locallyInside(b, a) && middleInside(a, b) &&
            (area(a->prev, a, b->prev) != 0.0 || area(a, b->prev, b) != 0.0)) ||
            (equals(a, b) && area(a->prev, a, a->next) > 0 && area(b->prev, b, b->next) > 0));
}

/// @brief signed area of a triangle
double Earclipping::area(const Earclipping::Point* p, const Earclipping::Point* q, const Earclipping::Point* r) const {
    return (q->y() - p->y()) * (r->x() - q->x()) - (q->x() - p->x()) * (r->y() - q->y());
}

/// @brief check if two points are equal
bool Earclipping::equals(const Earclipping::Point* p1, const Earclipping::Point* p2) {
    return p1->x() == p2->x() && p1->y() == p2->y();
}

/// @brief check if two segments intersect
bool Earclipping::intersects(const Earclipping::Point* p1, const Earclipping::Point* q1, const Earclipping::Point* p2, const Earclipping::Point* q2) {
    int o1 = sign(area(p1, q1, p2));
    int o2 = sign(area(p1, q1, q2));
    int o3 = sign(area(p2, q2, p1));
    int o4 = sign(area(p2, q2, q1));

    if (o1 != o2 && o3 != o4) return true; 

    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1)) return true; 
    if (o3 == 0 && onSegment(p2, p1, q2)) return true; 
    if (o4 == 0 && onSegment(p2, q1, q2)) return true; 

    return false;
}

/// @brief for collinear points p, q, r, check if point q lies on segment pr
bool Earclipping::onSegment(const Earclipping::Point* p, const Earclipping::Point* q, const Earclipping::Point* r) {
    return q->x() <= std::max<double>(p->x(), r->x()) &&
        q->x() >= std::min<double>(p->x(), r->x()) &&
        q->y() <= std::max<double>(p->y(), r->y()) &&
        q->y() >= std::min<double>(p->y(), r->y());
}

/// @brief Sign function for area calculation
int Earclipping::sign(double val) {
    return (0.0 < val) - (val < 0.0);
}

/// @brief Check if a polygon diagonal intersects any polygon segments
bool Earclipping::intersectsPolygon(const Earclipping::Point* a, const Earclipping::Point* b) {
    const Earclipping::Point* p = a;
    do {
        if (p->i != a->i && p->next->i != a->i && p->i != b->i && p->next->i != b->i &&
                intersects(p, p->next, a, b)) return true;
        p = p->next;
    } while (p != a);

    return false;
}

/// @brief Check if a polygon diagonal is locally inside the polygon
bool Earclipping::locallyInside(const Earclipping::Point* a, const Earclipping::Point* b) {
    return area(a->prev, a, a->next) < 0 ?
        area(a, b, a->next) >= 0 && area(a, a->prev, b) >= 0 :
        area(a, b, a->prev) < 0 || area(a, a->next, b) < 0;
}

/// @brief Check if the middle vertex of a polygon diagonal is inside the polygon
bool Earclipping::middleInside(const Earclipping::Point* a, const Earclipping::Point* b) {
    const Earclipping::Point* p = a;
    bool inside = false;
    double px = (a->x() + b->x()) / 2;
    double py = (a->y() + b->y()) / 2;
    do {
        if (((p->y() > py) != (p->next->y() > py)) && p->next->y() != p->y() &&
                (px < (p->next->x() - p->x()) * (py - p->y()) / (p->next->y() - p->y()) + p->x()))
            inside = !inside;
        p = p->next;
    } while (p != a);

    return inside;
}

/// @brief Link two polygon vertices with a bridge
Earclipping::Point* Earclipping::splitPolygon(Earclipping::Point* a, Earclipping::Point* b) {
    // Create Point2d objects from the x and y coordinates of the Points
    Point2d aPoint(a->x(), a->y());
    Point2d bPoint(b->x(), b->y());

    // Construct new Points with the index and Point2d objects
    Earclipping::Point* a2 = Points.construct(a->i, aPoint);
    Earclipping::Point* b2 = Points.construct(b->i, bPoint);

    Earclipping::Point* an = a->next;
    Earclipping::Point* bp = b->prev;

    // Update the linked list connections
    a->next = b;
    b->prev = a;

    a2->next = an;
    an->prev = a2;

    b2->next = a2;
    a2->prev = b2;

    bp->next = b2;
    b2->prev = bp;

    return b2;
}

/// @brief create a Point and optionally link it with the previous one (in a circular doubly linked list)
Earclipping::Point* Earclipping::insertPoint(std::size_t i, const Point2d& pt, Earclipping::Point* last) {
    Earclipping::Point* p = Points.construct(static_cast<std::size_t>(i), pt);

    if (!last) {
        p->prev = p;
        p->next = p;
    } else {
        assert(last);
        p->next = last->next;
        p->prev = last;
        last->next->prev = p;
        last->next = p;
    }
    return p;
}

/// @brief remove a Point from the linked list
void Earclipping::removePoint(Earclipping::Point* p) {
    p->next->prev = p->prev;
    p->prev->next = p->next;
}

/// @brief main triangulate function
std::vector<Polygon2d> triangulate(const Polygon2d& poly) {
    autoware::universe_utils::earclipping::Earclipping triangulate;
    triangulate(poly);

    std::vector<Polygon2d> triangles;

    const auto& indices = triangulate.indices;
    const std::size_t num_indices = indices.size();

    if (num_indices % 3 != 0) {
        throw std::runtime_error("Indices size should be a multiple of 3");
    }

    // Gather all vertices from outer and inner rings
    std::vector<Point2d> all_vertices;
    const auto& outer_ring = poly.outer();
    all_vertices.insert(all_vertices.end(), outer_ring.begin(), outer_ring.end());

    for (const auto& inner_ring : poly.inners()) {
        all_vertices.insert(all_vertices.end(), inner_ring.begin(), inner_ring.end());
    }

    const std::size_t total_vertices = all_vertices.size();

    for (std::size_t i = 0; i < num_indices; i += 3) {
        if (indices[i] >= total_vertices || indices[i + 1] >= total_vertices || indices[i + 2] >= total_vertices) {
            throw std::runtime_error("Index out of range");
        }

        Polygon2d triangle;
        triangle.outer().push_back(all_vertices[indices[i]]);
        triangle.outer().push_back(all_vertices[indices[i + 1]]);
        triangle.outer().push_back(all_vertices[indices[i + 2]]);
        triangle.outer().push_back(all_vertices[indices[i]]); // Close the triangle

        triangles.push_back(triangle);
    }

    return triangles;
}

}// autoware::universe_utils::earclipping


#endif // EAR_CUT_IMPL_HPP
