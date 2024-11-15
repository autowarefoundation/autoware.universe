// // Copyright 2024 TIER IV, Inc.
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

#include "autoware/universe_utils/geometry/buffer.hpp"

namespace autoware::universe_utils
{

namespace 
{
autoware::universe_utils::Polygon2d create_arc(autoware::universe_utils::Polygon2d& vertices, 
                                              const autoware::universe_utils::Point2d& center,
                                              double radius, 
                                              const autoware::universe_utils::Point2d& start_vertex, 
                                              const autoware::universe_utils::Point2d& end_vertex, 
                                              const autoware::universe_utils::Point2d& start_vertex_next, 
                                              double segments,
                                              bool skip) {
    using namespace autoware::universe_utils;

    const double PI2 = M_PI * 2;
    double startAngle = atan2(start_vertex_next.y() - center.y(), start_vertex_next.x() - center.x());
    double endAngle = atan2(end_vertex.y() - center.y(), end_vertex.x() - center.x());

    if (skip) {
        vertices.outer().push_back(start_vertex);
        skip = false;
    }

    if (startAngle < 0) startAngle += PI2;
    if (endAngle < 0) endAngle += PI2;

    double angleDiff = ((startAngle > endAngle) ? (startAngle - endAngle) : (startAngle + PI2 - endAngle));
    if (angleDiff < 0) angleDiff += PI2; 

    double segmentAngle = angleDiff / segments;

    for (int i = 0; i <= segments; ++i) {
        double angle = endAngle + i * segmentAngle;
        double x = center.x() + radius * cos(angle);
        double y = center.y() + radius * sin(angle);

        vertices.outer().push_back(Point2d(x, y));
    }
    return vertices;
}

autoware::universe_utils::Polygon2d _offset_segment(const autoware::universe_utils::Point2d& v1,
                                                   const autoware::universe_utils::Point2d& v2, 
                                                   const autoware::universe_utils::Point2d& next_vertex, 
                                                   double dist,
                                                   double segments,
                                                   bool skip) {
    using namespace autoware::universe_utils;

    Polygon2d vertices;
    
    double dx = v2.x() - v1.x();
    double dy = v2.y() - v1.y();

    double length = std::sqrt(dx * dx + dy * dy); 
    double normal_x = -dy / length;  
    double normal_y = dx / length;

    Point2d offset_v1(v1.x() - normal_x * dist, v1.y() - normal_y * dist); 
    Point2d offset_v2(v2.x() - normal_x * dist, v2.y() - normal_y * dist);

    double next_dx = next_vertex.x() - v2.x();
    double next_dy = next_vertex.y() - v2.y();
    double length_next = std::sqrt(next_dx * next_dx + next_dy * next_dy); 
    double normal_x_next = -next_dy / length_next; 
    double normal_y_next = next_dx / length_next;
    Point2d offset_v1next(v2.x() - normal_x_next * dist, v2.y() - normal_y_next * dist); 
    Point2d offset_v2next(next_vertex.x() - normal_x_next * dist, next_vertex.y() - normal_y_next * dist);




    double currAngle = atan2(dy, dx);
    double nextAngle = atan2(next_dy, next_dx);

    double angle_current_next = nextAngle - currAngle;
    if (angle_current_next < 0) angle_current_next += M_PI * 2;
    std::cout << "angleBetween" << angle_current_next << "\n";

    if (2* M_PI - angle_current_next > M_PI) {
        create_arc(vertices, v2, dist, offset_v1, offset_v2, offset_v1next, segments, skip);
    }

    else if (2* M_PI - angle_current_next < M_PI) {
        auto intersection_point = autoware::universe_utils::intersection(offset_v1, offset_v2, offset_v1next, offset_v2next);
        vertices.outer().push_back(offset_v1);
        vertices.outer().push_back(intersection_point[0]);
        vertices.outer().push_back(offset_v2next);
        skip = true;
    }

    else {
        vertices.outer().push_back(offset_v1);
        vertices.outer().push_back(offset_v2);
    }

    return vertices;
}

}

autoware::universe_utils::Polygon2d buffer(const autoware::universe_utils::Polygon2d& input_polygon, double dist, double segments) {
    using namespace autoware::universe_utils;

    Polygon2d offset_polygon;
    size_t verticesCount = input_polygon.outer().size();

    if (verticesCount < 2) {
        std::cerr << "Polygon needs at least 2 vertices!" << std::endl;
        return offset_polygon;
    }

    bool skip = false;

    for (size_t i = 0; i < verticesCount; ++i) {
        const auto& v1 = input_polygon.outer()[i];
        const auto& v2 = input_polygon.outer()[(i + 1) % verticesCount];

        const auto& next_vertex = input_polygon.outer()[(i + 2) % verticesCount];

        Polygon2d offset_segment = _offset_segment(v1, v2, next_vertex, dist, segments, skip);
        
        for (const auto& point : offset_segment.outer()) {
            offset_polygon.outer().push_back(point);
        }
    }

    if (!offset_polygon.outer().empty()) {
        offset_polygon.outer().push_back(offset_polygon.outer().front());
    }
    boost::geometry::correct(offset_polygon);

    return offset_polygon;

} // namespace

autoware::universe_utils::Polygon2d buffer(const autoware::universe_utils::Point2d& point, double distance, double segments) {
    using namespace autoware::universe_utils;

    Polygon2d offsetPolygon;

    for (int i = 0; i < segments; ++i) {
        double angle = 2 * M_PI * i / segments;
        Point2d buffer_point(point.x() + distance * cos(angle), point.y() + distance * sin(angle));
        offsetPolygon.outer().push_back(buffer_point);
    }
    boost::geometry::correct(offsetPolygon);
    return offsetPolygon;
}

autoware::universe_utils::Polygon2d buffer(const autoware::universe_utils::MultiPoint2d& multi_point, double distance, double segments) {
    using namespace autoware::universe_utils;

    autoware::universe_utils::Polygon2d buffer_union;
    bool first = true;  
    for (const auto& point : multi_point) {
        autoware::universe_utils::Polygon2d buffer_point = buffer(point, distance, segments);
        if (!first) {
            buffer_union = autoware::universe_utils::union_(buffer_point, buffer_union)[0];
        } else {
            buffer_union = buffer_point;
            first = false;
        }
    }
    return buffer_union;
}

} // namespace autoware::universe_utils
