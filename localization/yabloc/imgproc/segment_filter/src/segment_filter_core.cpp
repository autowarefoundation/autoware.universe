#include "segment_filter/segment_filter.hpp"

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <vml_common/pub_sub.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace vmvl_imgproc
{
SegmentFilter::SegmentFilter()
: Node("segment_filter"),
  image_size_(declare_parameter<int>("image_size", 800)),
  max_range_(declare_parameter<float>("max_range", 20.f)),
  truncate_pixel_threshold_(declare_parameter<int>("truncate_pixel_threshold", -1)),
  min_segment_length_(declare_parameter<float>("min_segment_length", -1)),
  max_segment_distance_(declare_parameter<float>("max_segment_distance", -1)),
  max_lateral_distance_(declare_parameter<float>("max_lateral_distance", -1)),
  info_(this),
  synchro_subscriber_(this, "lsd_cloud", "graph_segmented"),
  tf_subscriber_(this->get_clock())
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  auto cb = std::bind(&SegmentFilter::execute, this, _1, _2);
  synchro_subscriber_.setCallback(std::move(cb));

  pub_cloud_ = create_publisher<PointCloud2>("projected_lsd_cloud", 10);
  pub_image_ = create_publisher<Image>("projected_image", 10);
}

cv::Point2i SegmentFilter::toCvPoint(const Eigen::Vector3f & v) const
{
  cv::Point pt;
  pt.x = -v.y() / max_range_ * image_size_ * 0.5f + image_size_ / 2;
  pt.y = -v.x() / max_range_ * image_size_ * 0.5f + image_size_;
  return pt;
}

void SegmentFilter::execute(const PointCloud2 & lsd_msg, const PointCloud2 & segment_msg)
{
  const rclcpp::Time stamp = lsd_msg.header.stamp;
  pcl::PointCloud<pcl::PointXYZ>::Ptr mask{new pcl::PointCloud<pcl::PointXYZ>()};
  pcl::PointCloud<pcl::PointNormal>::Ptr lsd{new pcl::PointCloud<pcl::PointNormal>()};
  pcl::fromROSMsg(segment_msg, *mask);
  pcl::fromROSMsg(lsd_msg, *lsd);

  if (info_.isCameraInfoNullOpt()) return;

  Eigen::Matrix3f K = info_.intrinsic();
  Eigen::Matrix3f Kinv = K.inverse();

  std::optional<Eigen::Affine3f> camera_extrinsic = tf_subscriber_(info_.getFrameId(), "base_link");
  if (!camera_extrinsic.has_value()) return;
  if (mask->empty()) {
    RCLCPP_WARN_STREAM(this->get_logger(), "there are no contours");
    return;
  }
  const Eigen::Vector3f t = camera_extrinsic->translation();
  const Eigen::Quaternionf q(camera_extrinsic->rotation());

  // TODO: This will take into account ground tilt and camera vibration someday.
  ProjectFunc project_func = [Kinv, q,
                              t](const Eigen::Vector3f & u) -> std::optional<Eigen::Vector3f> {
    Eigen::Vector3f u3(u.x(), u.y(), 1);
    Eigen::Vector3f u_bearing = (q * Kinv * u3).normalized();
    if (u_bearing.z() > -0.01) return std::nullopt;
    float u_distance = -t.z() / u_bearing.z();
    Eigen::Vector3f v;
    v.x() = t.x() + u_bearing.x() * u_distance;
    v.y() = t.y() + u_bearing.y() * u_distance;
    v.z() = 0;
    return v;
  };

  pcl::PointCloud<pcl::PointXYZ>::Ptr projected_mask{new pcl::PointCloud<pcl::PointXYZ>()};
  pcl::PointCloud<pcl::PointNormal>::Ptr projected_lines{new pcl::PointCloud<pcl::PointNormal>()};
  *projected_mask = projectMask(*mask, project_func);
  *projected_lines = projectLines(*lsd, project_func);

  // Here, "reliable" means having overlap in the mask area.

  // Extract reliable point's indices
  pcl::PointIndices indices = filtByMask(*projected_mask, *projected_lines);

  // Publish point cloud
  pcl::ExtractIndices<pcl::PointNormal> extract;
  pcl::PointIndicesPtr indices_ptr = pcl::make_shared<pcl::PointIndices>(indices);
  extract.setIndices(indices_ptr);
  extract.setInputCloud(projected_lines);
  pcl::PointCloud<pcl::PointNormal> reliable_edges;
  extract.filter(reliable_edges);
  vml_common::publishCloud(*pub_cloud_, reliable_edges, stamp);

  // Draw image
  cv::Mat reliable_line_image = cv::Mat::zeros(cv::Size{image_size_, image_size_}, CV_8UC3);
  {
    // Draw mask areas
    std::vector<cv::Point2i> contour;
    std::vector<std::vector<cv::Point2i>> contours(1);
    for (const auto p : projected_mask->points)
      contours.front().push_back(toCvPoint(p.getVector3fMap()));
    cv::drawContours(reliable_line_image, contours, 0, cv::Scalar(0, 155, 155), -1);

    // Draw lines
    std::unordered_set<int> reliable_set;
    for (int index : indices.indices) reliable_set.insert(index);
    for (size_t i = 0; i < projected_lines->size(); i++) {
      auto & pn = projected_lines->at(i);
      cv::Point2i p1 = toCvPoint(pn.getVector3fMap());
      cv::Point2i p2 = toCvPoint(pn.getNormalVector3fMap());
      if (reliable_set.count(i) == 0)
        cv::line(reliable_line_image, p1, p2, cv::Scalar(255, 255, 255), 2, cv::LineTypes::LINE_8);
      else
        cv::line(reliable_line_image, p1, p2, cv::Scalar(100, 100, 255), 5, cv::LineTypes::LINE_8);
    }
  }

  vml_common::publishImage(*pub_image_, reliable_line_image, stamp);
}

bool SegmentFilter::isNearElement(
  const pcl::PointNormal & pn, pcl::PointNormal & truncated_pn) const
{
  float min_distance = std::min(pn.x, pn.normal_x);
  float max_distance = std::max(pn.x, pn.normal_x);
  if (min_distance > max_segment_distance_) return false;
  if (max_distance < max_segment_distance_) {
    truncated_pn = pn;
    return true;
  }

  truncated_pn = pn;
  Eigen::Vector3f t = pn.getVector3fMap() - pn.getNormalVector3fMap();
  float not_zero_tx = t.x() > 0 ? std::max(t.x(), 1e-3f) : std::min(t.x(), -1e-3f);
  float lambda = (max_segment_distance_ - pn.x) / not_zero_tx;
  Eigen::Vector3f m = pn.getVector3fMap() + lambda * t;
  if (pn.x > pn.normal_x)
    truncated_pn.getVector3fMap() = m;
  else
    truncated_pn.getNormalVector3fMap() = m;
  return true;
}

bool SegmentFilter::isLowerElement(
  const pcl::PointNormal & pn, pcl::PointNormal & truncated_pn) const
{
  float lower_px = std::max(pn.y, pn.normal_y);
  float higher_px = std::min(pn.y, pn.normal_y);
  if (lower_px < truncate_pixel_threshold_) return false;
  if (higher_px > truncate_pixel_threshold_) {
    truncated_pn = pn;
    return true;
  }

  truncated_pn = pn;
  Eigen::Vector3f t = pn.getVector3fMap() - pn.getNormalVector3fMap();
  float not_zero_ty = t.y() > 0 ? std::max(t.y(), 1e-3f) : std::min(t.y(), -1e-3f);
  float lambda = (truncate_pixel_threshold_ - pn.y) / not_zero_ty;
  Eigen::Vector3f m = pn.getVector3fMap() + lambda * t;
  if (pn.y < pn.normal_y)
    truncated_pn.getVector3fMap() = m;
  else
    truncated_pn.getNormalVector3fMap() = m;
  return true;
}

std::set<ushort> getUniquePixelValue(cv::Mat & image)
{
  // `image` is a set of ushort.
  // The purpose is to find the unduplicated set of values contained in `image`.
  // For example, if `image` is {0,1,2,0,1,2,3}, this function returns {0,1,2,3}.
  auto begin = image.begin<ushort>();
  auto last = std::unique(begin, image.end<ushort>());
  std::sort(begin, last);
  last = std::unique(begin, last);
  return std::set<ushort>(begin, last);
}

pcl::PointCloud<pcl::PointXYZ> SegmentFilter::projectMask(
  const pcl::PointCloud<pcl::PointXYZ> & points, ProjectFunc project) const
{
  pcl::PointCloud<pcl::PointXYZ> projected_points;
  for (const auto & p : points) {
    std::optional<Eigen::Vector3f> opt = project(p.getVector3fMap());
    if (!opt.has_value()) continue;
    pcl::PointXYZ xyz(opt->x(), opt->y(), opt->z());
    projected_points.push_back(xyz);
  }
  return projected_points;
}

pcl::PointCloud<pcl::PointNormal> SegmentFilter::projectLines(
  const pcl::PointCloud<pcl::PointNormal> & points, ProjectFunc project) const
{
  pcl::PointCloud<pcl::PointNormal> projected_points;
  for (const auto & pn : points) {
    pcl::PointNormal truncated_pn = pn;
    if (truncate_pixel_threshold_ > 0)
      if (!isLowerElement(pn, truncated_pn)) continue;

    std::optional<Eigen::Vector3f> opt1 = project(truncated_pn.getVector3fMap());
    std::optional<Eigen::Vector3f> opt2 = project(truncated_pn.getNormalVector3fMap());
    if (!opt1.has_value()) continue;
    if (!opt2.has_value()) continue;

    // If linesegment has shoter length than config, it is excluded
    if (min_segment_length_ > 0) {
      float length = (opt1.value() - opt2.value()).norm();
      if (length < min_segment_length_) continue;
    }
    if (max_lateral_distance_ > 0) {
      float abs_lateral1 = std::abs(opt1.value().y());
      float abs_lateral2 = std::abs(opt2.value().y());
      if (std::min(abs_lateral1, abs_lateral2) > max_lateral_distance_) continue;
    }

    pcl::PointNormal xyz;
    xyz.x = opt1->x();
    xyz.y = opt1->y();
    xyz.z = opt1->z();
    xyz.normal_x = opt2->x();
    xyz.normal_y = opt2->y();
    xyz.normal_z = opt2->z();

    //
    pcl::PointNormal truncated_xyz = xyz;
    if (max_segment_distance_ > 0)
      if (!isNearElement(xyz, truncated_xyz)) continue;

    projected_points.push_back(truncated_xyz);
  }
  return projected_points;
}

pcl::PointIndices SegmentFilter::filtByMask(
  const pcl::PointCloud<pcl::PointXYZ> & mask, const pcl::PointCloud<pcl::PointNormal> & edges)
{
  // Create line image and assign different color to each segment.
  cv::Mat line_image = cv::Mat::zeros(cv::Size{image_size_, image_size_}, CV_16UC1);
  for (size_t i = 0; i < edges.size(); i++) {
    auto & pn = edges.at(i);
    cv::Point2i p1 = toCvPoint(pn.getVector3fMap());
    cv::Point2i p2 = toCvPoint(pn.getNormalVector3fMap());
    cv::Scalar color = cv::Scalar::all(i + 1);
    cv::line(line_image, p1, p2, color, 1, cv::LineTypes::LINE_4);
  }

  // Create mask image
  cv::Mat mask_image = cv::Mat::zeros(cv::Size{image_size_, image_size_}, CV_16UC1);
  std::vector<std::vector<cv::Point2i>> projected_hull(1);
  for (const auto p : mask) projected_hull.front().push_back(toCvPoint(p.getVector3fMap()));
  auto max_color = cv::Scalar::all(std::numeric_limits<u_short>::max());
  cv::drawContours(mask_image, projected_hull, 0, max_color, -1);

  // TODO: Using boost::geometry is more intuitive.
  // https://boostjp.github.io/tips/geometry.html#disjoint

  // And operator
  cv::Mat masked_line;
  cv::bitwise_and(mask_image, line_image, masked_line);
  std::set<ushort> pixel_values = getUniquePixelValue(masked_line);

  // Extract edges within masks
  pcl::PointIndices reliable_indices;
  for (size_t i = 0; i < edges.size(); i++) {
    if (pixel_values.count(i + 1) != 0) reliable_indices.indices.push_back(i);
  }

  return reliable_indices;
}

}  // namespace vmvl_imgproc
