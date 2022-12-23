#pragma once
#include <Eigen/StdVector>
#include <opencv4/opencv2/core.hpp>
#include <pcdless_common/gamma_converter.hpp>
#include <rclcpp/node.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/functional/hash.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless::common
{
struct Area
{
  Area() {}
  Area(const Eigen::Vector2f & v)
  {
    if (unit_length_ < 0) throw_error();
    x = static_cast<long>(std::floor(v.x() / unit_length_));
    y = static_cast<long>(std::floor(v.y() / unit_length_));
  }

  Eigen::Vector2f real_scale() const { return {x * unit_length_, y * unit_length_}; };

  void throw_error() const
  {
    std::cerr << "Are::unit_length_ is not initialized" << std::endl;
    exit(EXIT_FAILURE);
  }
  int x, y;
  static float unit_length_;
  static float image_size_;

  friend bool operator==(const Area & one, const Area & other)
  {
    return one.x == other.x && one.y == other.y;
  }
  friend bool operator!=(const Area & one, const Area & other) { return !(one == other); }
  size_t operator()(const Area & index) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, index.x);
    boost::hash_combine(seed, index.y);
    return seed;
  }
};

class HierarchicalCostMap
{
public:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Pose = geometry_msgs::msg::Pose;

  HierarchicalCostMap(rclcpp::Node * node);

  void set_cloud(const pcl::PointCloud<pcl::PointNormal> & cloud);
  void set_unmapped_area(const pcl::PointCloud<pcl::PointXYZ> & polygon);

  float at(const Eigen::Vector2f & position);
  cv::Vec2b at2(const Eigen::Vector2f & position);
  cv::Vec3b at3(const Eigen::Vector2f & position);

  MarkerArray show_map_range() const;

  cv::Mat get_map_image(const Pose & pose);

  void erase_obsolete();

  void set_height(float height);

private:
  const float max_range_;
  const float image_size_;
  const size_t max_map_count_;
  rclcpp::Logger logger_;
  std::optional<float> height_{std::nullopt};

  common::GammaConverter gamma_converter{4.0f};

  std::unordered_map<Area, bool, Area> map_accessed_;

  std::list<Area> generated_map_history_;
  std::optional<pcl::PointCloud<pcl::PointNormal>> cloud_;
  pcl::PointCloud<pcl::PointXYZ> unmapped_polygon_;
  std::unordered_map<Area, cv::Mat, Area> cost_maps_;

  cv::Point to_cv_point(const Area & are, const Eigen::Vector2f);
  void build_map(const Area & area);

  cv::Mat create_available_area_image(const Area & area);
};
}  // namespace pcdless::common