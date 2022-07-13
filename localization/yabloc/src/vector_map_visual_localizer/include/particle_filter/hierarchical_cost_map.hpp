#pragma once
#include "common/gammma_conveter.hpp"

#include <eigen3/Eigen/StdVector>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/logger.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/functional/hash.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace particle_filter
{
struct Area
{
  Area() {}
  Area(const Eigen::Vector2f & v)
  {
    if (unit_length_ < 0) throwError();
    x = static_cast<long>(std::floor(v.x() / unit_length_));
    y = static_cast<long>(std::floor(v.y() / unit_length_));
  }

  Eigen::Vector2f realScale() const { return {x * unit_length_, y * unit_length_}; };

  void throwError() const
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

  HierarchicalCostMap(
    const rclcpp::Logger & logger, float max_range, float image_size, float gamma);

  void setCloud(const pcl::PointCloud<pcl::PointNormal> & cloud);

  float at(const Eigen::Vector2f & position);

  MarkerArray showMapRange() const;

  cv::Mat getMapImage(const Pose & pose);

  void eraseObsolete();

private:
  const float max_range_;
  const float image_size_;
  const int max_map_count_;
  rclcpp::Logger logger_;

  cv::Point toCvPoint(const Area & are, const Eigen::Vector2f);
  void buildMap(const Area & area);
  GammaConverter gamma_converter{4.0f};

  std::unordered_map<Area, bool, Area> map_accessed_;

  std::list<Area> generated_map_history_;
  std::optional<pcl::PointCloud<pcl::PointNormal>> cloud_;
  std::unordered_map<Area, cv::Mat, Area> cost_maps_;
};
}  // namespace particle_filter