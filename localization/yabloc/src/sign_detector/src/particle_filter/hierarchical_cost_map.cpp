#include "particle_filter/hierarchical_cost_map.hpp"

#include "common/gammma_conveter.hpp"

#include <opencv4/opencv2/imgproc.hpp>

namespace particle_filter
{
float Area::unit_length_ = -1;

cv::Point HierarchicalCostMap::toCvPoint(const Area & area, const Eigen::Vector2f p)
{
  Eigen::Vector2f relative = p - area.realScale();
  float px = relative.x() / max_range_ * image_size_;
  float py = relative.y() / max_range_ * image_size_;
  return {px, py};
}

HierarchicalCostMap::HierarchicalCostMap(float max_range, float image_size, float gamma)
: max_range_(max_range_), image_size_(image_size)
{
  Area::unit_length_ = max_range;
  gamma_converter.reset(gamma);
}

float HierarchicalCostMap::at(const Eigen::Vector2f & position)
{
  Area key(position);
  if (cost_maps.count(key) == 0) {
    buildMap(key);
  }
  return cost_maps.at(key).at<uchar>(toCvPoint(key, position));
}

void HierarchicalCostMap::setCloud(const pcl::PointCloud<pcl::PointNormal> & cloud)
{
  cloud_ = cloud;
}

void HierarchicalCostMap::buildMap(const Area & area)
{
  cv::Mat cost_map;
  cv::Mat image = 255 - cv::Mat::ones(cv::Size(image_size_, image_size_), CV_8UC1);

  auto cvPoint = [this, area](const Eigen::Vector3f & p) -> cv::Point {
    return this->toCvPoint(area, p.topRows(2));
  };

  for (const auto pn : cloud_.value()) {
    cv::line(
      image, cvPoint(pn.getVector3fMap()), cvPoint(pn.getNormalVector3fMap()), cv::Scalar::all(0),
      1);
  }
  cv::Mat distance;
  cv::distanceTransform(image, distance, cv::DIST_L2, 3);
  cv::threshold(distance, distance, 100, 100, cv::THRESH_TRUNC);
  distance.convertTo(distance, CV_8UC1, -2.55, 255);

  cost_maps[area] = gamma_converter(distance);
  generated_map_history_.push_back(area);

  // It can have a maximum of 10 local maps.
  if (generated_map_history_.size() > 10) {
    auto key = generated_map_history_.front();
    generated_map_history_.pop_front();
    cost_maps.erase(key);
    std::cerr << "old cost map is poped" << std::endl;
  }
}

}  // namespace particle_filter