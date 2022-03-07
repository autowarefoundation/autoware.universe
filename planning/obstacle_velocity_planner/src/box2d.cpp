#include "obstacle_velocity_planner/box2d.hpp"

#include <tf2/utils.h>
#include <autoware_utils/autoware_utils.hpp>

Box2d::Box2d(const geometry_msgs::msg::Pose & center_pose, const double length, const double width)
 : center_(center_pose.position), length_(length), width_(width), half_length_(length/2.0), half_width_(width/2.0)
{
    max_x_ = std::numeric_limits<double>::min();
    max_y_ = std::numeric_limits<double>::min();
    min_x_ = std::numeric_limits<double>::max();
    min_y_ = std::numeric_limits<double>::max();
    heading_ = tf2::getYaw(center_pose.orientation);
    cos_heading_ = std::cos(heading_);
    sin_heading_ = std::sin(heading_);
    InitCorners();
}

void Box2d::InitCorners() {
    const double dx1 = cos_heading_ * half_length_;
    const double dy1 = sin_heading_ * half_length_;
    const double dx2 = sin_heading_ * half_width_;
    const double dy2 = -cos_heading_ * half_width_;

    const auto p1 = autoware_utils::createPoint(center_.x + dx1 + dx2, center_.y + dy1 + dy2, center_.z);
    const auto p2 = autoware_utils::createPoint(center_.x + dx1 - dx2, center_.y + dy1 - dy2, center_.z);
    const auto p3 = autoware_utils::createPoint(center_.x - dx1 - dx2, center_.y - dy1 - dy2, center_.z);
    const auto p4 = autoware_utils::createPoint(center_.x - dx1 + dx2, center_.y - dy1 + dy2, center_.z);
    corners_.clear();
    corners_.resize(4);
    corners_[0] = p1;
    corners_[1] = p2;
    corners_[2] = p3;
    corners_[3] = p4;

    for (auto &corner : corners_) {
        max_x_ = std::fmax(corner.x, max_x_);
        min_x_ = std::fmin(corner.x, min_x_);
        max_y_ = std::fmax(corner.y, max_y_);
        min_y_ = std::fmin(corner.y, min_y_);
    }
}

bool Box2d::HasOverlap(const Box2d &box) const {
    if (box.max_x() < min_x() || box.min_x() > max_x() || box.max_y() < min_y() ||
        box.min_y() > max_y()) {
        return false;
    }

    const double shift_x = box.center_x() - center_.x;
    const double shift_y = box.center_y() - center_.y;

    const double dx1 = cos_heading_ * half_length_;
    const double dy1 = sin_heading_ * half_length_;
    const double dx2 = sin_heading_ * half_width_;
    const double dy2 = -cos_heading_ * half_width_;
    const double dx3 = box.cos_heading() * box.half_length();
    const double dy3 = box.sin_heading() * box.half_length();
    const double dx4 = box.sin_heading() * box.half_width();
    const double dy4 = -box.cos_heading() * box.half_width();

    return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
           std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
           std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
           half_length_ &&
           std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
           std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
           std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
           half_width_ &&
           std::abs(shift_x * box.cos_heading() + shift_y * box.sin_heading()) <=
           std::abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) +
           std::abs(dx2 * box.cos_heading() + dy2 * box.sin_heading()) +
           box.half_length() &&
           std::abs(shift_x * box.sin_heading() - shift_y * box.cos_heading()) <=
           std::abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) +
           std::abs(dx2 * box.sin_heading() - dy2 * box.cos_heading()) +
           box.half_width();
}
