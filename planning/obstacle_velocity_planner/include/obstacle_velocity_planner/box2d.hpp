#ifndef OBSTACLE_VELOCITY_PLANNER_BOX2D_HPP_
#define OBSTACLE_VELOCITY_PLANNER_BOX2D_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include <vector>

class Box2d
{
public:
    Box2d(const geometry_msgs::msg::Pose & center_pose, const double length, const double width);

    bool HasOverlap(const Box2d & box) const ;

    void InitCorners();

    /**
   * @brief Getter of the center of the box
   * @return The center of the box
   */
    const geometry_msgs::msg::Point& center() const { return center_; }

    /**
     * @brief Getter of the x-coordinate of the center of the box
     * @return The x-coordinate of the center of the box
     */
    double center_x() const { return center_.x; }

    /**
     * @brief Getter of the y-coordinate of the center of the box
     * @return The y-coordinate of the center of the box
     */
    double center_y() const { return center_.y; }

    /**
     * @brief Getter of the length
     * @return The length of the heading-axis
     */
    double length() const { return length_; }

    /**
     * @brief Getter of the width
     * @return The width of the box taken perpendicularly to the heading
     */
    double width() const { return width_; }

    /**
     * @brief Getter of half the length
     * @return Half the length of the heading-axis
     */
    double half_length() const { return half_length_; }

    /**
     * @brief Getter of half the width
     * @return Half the width of the box taken perpendicularly to the heading
     */
    double half_width() const { return half_width_; }

    /**
     * @brief Getter of the heading
     * @return The counter-clockwise angle between the x-axis and the heading-axis
     */
    double heading() const { return heading_; }

    /**
     * @brief Getter of the cosine of the heading
     * @return The cosine of the heading
     */
    double cos_heading() const { return cos_heading_; }

    /**
     * @brief Getter of the sine of the heading
     * @return The sine of the heading
     */
    double sin_heading() const { return sin_heading_; }

    /**
   * @brief Getter of the area of the box
   * @return The product of its length and width
   */
    double area() const { return length_ * width_; }

    /**
     * @brief Getter of the size of the diagonal of the box
     * @return The diagonal size of the box
     */
    double diagonal() const { return std::hypot(length_, width_); }

    /**
     * @brief Getter of the corners of the box
     * @param corners The vector where the corners are listed
     */
    std::vector<geometry_msgs::msg::Point> GetAllCorners() const { return corners_; }

    double max_x() const { return max_x_; }
    double min_x() const { return min_x_; }
    double max_y() const { return max_y_; }
    double min_y() const { return min_y_; }

private:
    geometry_msgs::msg::Point center_;
    double length_ = 0.0;
    double width_ = 0.0;
    double half_length_ = 0.0;
    double half_width_ = 0.0;
    double heading_ = 0.0;
    double cos_heading_ = 1.0;
    double sin_heading_ = 0.0;

    std::vector<geometry_msgs::msg::Point> corners_;

    double max_x_ = std::numeric_limits<double>::lowest();
    double min_x_ = std::numeric_limits<double>::max();
    double max_y_ = std::numeric_limits<double>::lowest();
    double min_y_ = std::numeric_limits<double>::max();
};

#endif // OBSTACLE_VELOCITY_PLANNER_BOX2D_HPP_
