#include "particle_filter/hierarchical_cost_map.hpp"

#include "common/gammma_conveter.hpp"
#include "common/util.hpp"

#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

namespace particle_filter
{
float Area::unit_length_ = -1;

HierarchicalCostMap::HierarchicalCostMap(rclcpp::Node * node)
: max_range_(node->declare_parameter<float>("max_range")),
  image_size_(node->declare_parameter<int>("image_size")),
  max_map_count_(10),
  logger_(node->get_logger())
{
  Area::unit_length_ = max_range_;
  float gamma = node->declare_parameter<float>("gamma");
  gamma_converter.reset(gamma);
}

cv::Point2i HierarchicalCostMap::toCvPoint(const Area & area, const Eigen::Vector2f p)
{
  Eigen::Vector2f relative = p - area.realScale();
  float px = relative.x() / max_range_ * image_size_;
  float py = relative.y() / max_range_ * image_size_;
  return {static_cast<int>(px), static_cast<int>(py)};
}

float HierarchicalCostMap::at(const Eigen::Vector2f & position)
{
  Area key(position);
  if (cost_maps_.count(key) == 0) {
    buildMap(key);
  }
  map_accessed_[key] = true;

  cv::Point2i tmp = toCvPoint(key, position);
  return cost_maps_.at(key).at<uchar>(tmp);
}

void HierarchicalCostMap::setCloud(const pcl::PointCloud<pcl::PointNormal> & cloud)
{
  cloud_ = cloud;
}

void visualizeAngleMap(const cv::Mat & angle_image, const cv::Mat & intensity_image)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  cloud->points.reserve(2000);
  for (int i = 0; i < angle_image.rows; i++) {
    const uchar * p1 = angle_image.ptr<uchar>(i);
    for (int j = 0; j < angle_image.cols; j++) {
      if (p1[j] == 0) continue;
      pcl::PointXYZI xyz;
      xyz.intensity = p1[j];
      xyz.x = i, xyz.y = j, xyz.z = 0;
      cloud->push_back(xyz);
    }
  }
  pcl::search::KdTree<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(cloud);

  auto getNearest = [cloud, &kdtree](int i, int j) -> float {
    if (cloud->empty()) {
      return 0;
    }
    std::vector<int> indices;
    std::vector<float> distances;
    pcl::PointXYZI query;
    query.x = i, query.y = j, query.z = 0;
    kdtree.nearestKSearch(query, 1, indices, distances);
    return cloud->at(indices.front()).intensity;
  };

  cv::Mat show_image = cv::Mat::zeros(angle_image.size(), CV_8UC3);

  for (int i = 0; i < angle_image.rows; i++) {
    const uchar * p1 = angle_image.ptr<uchar>(i);
    const uchar * p2 = intensity_image.ptr<uchar>(i);
    uchar * q = show_image.ptr<uchar>(i);
    for (int j = 0; j < angle_image.cols; j++) {
      cv::Scalar color(0, 0, 0);
      if (p2[j] > 25) color = cv::Scalar(getNearest(i, j), p2[j], p2[j]);
      // if (p2[j] > 10) color = cv::Scalar(p1[j], p2[j], p2[j]);

      q[j * 3] = (uchar)color[0];
      q[j * 3 + 1] = (uchar)color[1];
      q[j * 3 + 2] = (uchar)color[2];
    }
  }
  cv::cvtColor(show_image, show_image, cv::COLOR_HSV2BGR);

  cv::imshow("angle", show_image);
  cv::waitKey(5);
}

void HierarchicalCostMap::buildMap(const Area & area)
{
  if (!cloud_.has_value()) return;

  cv::Mat image = 255 * cv::Mat::ones(cv::Size(image_size_, image_size_), CV_8UC1);
  cv::Mat angle_image = cv::Mat::zeros(cv::Size(image_size_, image_size_), CV_8UC1);

  auto cvPoint = [this, area](const Eigen::Vector3f & p) -> cv::Point {
    return this->toCvPoint(area, p.topRows(2));
  };

  for (const auto pn : cloud_.value()) {
    cv::line(
      image, cvPoint(pn.getVector3fMap()), cvPoint(pn.getNormalVector3fMap()), cv::Scalar::all(0),
      1);

    Eigen::Vector3f d = pn.getVector3fMap() - pn.getNormalVector3fMap();
    float rad = std::atan2(d.y(), d.x());
    if (rad > M_PI) rad -= M_PI;
    int deg = 180.f / 3.14 * rad;
    cv::line(
      angle_image, cvPoint(pn.getVector3fMap()), cvPoint(pn.getNormalVector3fMap()),
      cv::Scalar::all(deg), 1);
  }
  cv::Mat distance;
  cv::distanceTransform(image, distance, cv::DIST_L2, 3);
  cv::threshold(distance, distance, 100, 100, cv::THRESH_TRUNC);
  distance.convertTo(distance, CV_8UC1, -2.55, 255);

  cost_maps_[area] = gamma_converter(distance);
  generated_map_history_.push_back(area);

  RCLCPP_INFO_STREAM(
    logger_, "successed to build map " << area(area) << " " << area.realScale().transpose());

  visualizeAngleMap(angle_image, distance);
}

HierarchicalCostMap::MarkerArray HierarchicalCostMap::showMapRange() const
{
  MarkerArray array_msg;

  auto gpoint = [](float x, float y) -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point gp;
    gp.x = x;
    gp.y = y;
    return gp;
  };

  int id = 0;
  for (const Area & area : generated_map_history_) {
    Marker marker;
    marker.header.frame_id = "map";
    marker.id = id++;
    marker.type = Marker::LINE_STRIP;
    marker.color = util::color(0, 0, 1.0f, 1.0f);
    marker.scale.x = 0.1;
    Eigen::Vector2f xy = area.realScale();
    marker.points.push_back(gpoint(xy.x(), xy.y()));
    marker.points.push_back(gpoint(xy.x() + area.unit_length_, xy.y()));
    marker.points.push_back(gpoint(xy.x() + area.unit_length_, xy.y() + area.unit_length_));
    marker.points.push_back(gpoint(xy.x(), xy.y() + area.unit_length_));
    marker.points.push_back(gpoint(xy.x(), xy.y()));
    array_msg.markers.push_back(marker);
  }
  return array_msg;
}

cv::Mat HierarchicalCostMap::getMapImage(const Pose & pose)
{
  if (generated_map_history_.empty())
    return cv::Mat::zeros(cv::Size(image_size_, image_size_), CV_8UC3);

  Eigen::Vector2f center;
  center << pose.position.x, pose.position.y;

  float w = pose.orientation.w;
  float z = pose.orientation.z;
  Eigen::Matrix2f R = Eigen::Rotation2Df(2.f * std::atan2(z, w) - M_PI_2).toRotationMatrix();

  auto toVector2f = [this, center, R](float h, float w) -> Eigen::Vector2f {
    Eigen::Vector2f offset;
    offset.x() = (w / this->image_size_ - 0.5f) * this->max_range_;
    offset.y() = -(h / this->image_size_ - 0.5f) * this->max_range_;
    return center + R * offset;
  };

  cv::Mat image = cv::Mat::zeros(cv::Size(image_size_, image_size_), CV_8UC1);
  for (int w = 0; w < image_size_; w++) {
    for (int h = 0; h < image_size_; h++) {
      image.at<uchar>(h, w) = this->at(toVector2f(h, w));
    }
  }

  cv::Mat rgb_image;
  cv::applyColorMap(image, rgb_image, cv::COLORMAP_JET);
  return rgb_image;
}

void HierarchicalCostMap::eraseObsolete()
{
  if (cost_maps_.size() < max_map_count_) return;

  for (auto itr = generated_map_history_.begin(); itr != generated_map_history_.end();) {
    if (map_accessed_[*itr]) {
      ++itr;
      continue;
    }
    cost_maps_.erase(*itr);
    itr = generated_map_history_.erase(itr);
  }

  map_accessed_.clear();
}

}  // namespace particle_filter