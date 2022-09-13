#include "particle_initializer/particle_initializer.hpp"

#include <vmvl_map/ll2_util.hpp>

#include <boost/range/adaptors.hpp>

namespace modularized_particle_filter
{
ParticleInitializer::ParticleInitializer()
: Node("particle_initializer"),
  cov_xx_yy_{declare_parameter("cov_xx_yy", std::vector<double>{4.0, 0.25}).data()}
{
  using std::placeholders::_1;
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();

  // Publisher
  pub_initialpose_ = create_publisher<PoseCovStamped>("rectified/initialpose", 10);
  pub_marker_ = create_publisher<Marker>("init/marker", 10);

  // Subscriber
  auto on_initialpose = std::bind(&ParticleInitializer::onInitialpose, this, _1);
  auto on_map = std::bind(&ParticleInitializer::onMap, this, _1);
  sub_initialpose_ =
    create_subscription<PoseCovStamped>("initialpose", 10, std::move(on_initialpose));
  sub_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, std::move(on_map));
}

void ParticleInitializer::onInitialpose(const PoseCovStamped & initialpose)
{
  if (kdtree_ == nullptr) {
    RCLCPP_WARN_STREAM(get_logger(), "kdtree is nullptr");
    return;
  }

  auto position = initialpose.pose.pose.position;
  Eigen::Vector3f pos_vec3f;
  pos_vec3f << position.x, position.y, position.z;

  int nearest_index = searchNearestPointIndex(pos_vec3f);
  lanelet::Id id = lanelet::InvalId;
  if (nearest_index >= 0) id = point_id_to_lanelet_id_.at(nearest_index);
  if (id == lanelet::InvalId) return;

  pos_vec3f = cloud_->at(nearest_index).getArray3fMap();

  const lanelet::Lanelet lane = *lanelet_map_->laneletLayer.find(id);
  Eigen::Vector3f tangent = tangentDirection(lane, pos_vec3f);
  publishRangeMarker(pos_vec3f, tangent);
  publishRectifiedInitialpose(pos_vec3f, tangent, initialpose);
}

void ParticleInitializer::onMap(const HADMapBin & bin_map)
{
  lanelet_map_ = map::fromBinMsg(bin_map);

  cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  auto toPointXYZ = [](const lanelet::ConstPoint3d & p) -> pcl::PointXYZ {
    pcl::PointXYZ xyz;
    xyz.x = p.x();
    xyz.y = p.y();
    xyz.z = p.z();
    return xyz;
  };

  for (const lanelet::Lanelet & lane : lanelet_map_->laneletLayer) {
    for (const lanelet::ConstPoint3d & p : lane.centerline3d()) {
      cloud_->push_back(toPointXYZ(p));
      point_id_to_lanelet_id_[cloud_->size()] = lane.id();
    }
  }

  kdtree_ = pcl::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  kdtree_->setInputCloud(cloud_);
}

int ParticleInitializer::searchNearestPointIndex(const Eigen::Vector3f & pos)
{
  pcl::PointXYZ query(pos.x(), pos.y(), pos.z());
  std::vector<int> indices;
  std::vector<float> distances;
  kdtree_->nearestKSearch(query, 1, indices, distances);

  pcl::PointXYZ xyz = cloud_->at(indices.front());
  Eigen::Vector3f ref = xyz.getVector3fMap();
  float xy_distance = (ref - pos).topRows(2).norm();

  if (xy_distance > (3 * 3)) {
    RCLCPP_WARN_STREAM(get_logger(), "nearest neighbor is too far " << xy_distance);
    RCLCPP_WARN_STREAM(get_logger(), ref.transpose() << "   " << pos.transpose());
    return -1;
  }
  return indices.front();
}

void ParticleInitializer::publishRangeMarker(
  const Eigen::Vector3f & pos, const Eigen::Vector3f & tangent)
{
  Marker msg;
  msg.type = Marker::LINE_STRIP;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = "map";
  msg.scale.x = 0.2;
  msg.scale.y = 0.2;

  msg.color.r = 0;
  msg.color.g = 1;
  msg.color.b = 0;
  msg.color.a = 1;

  auto cast2gp = [](const Eigen::Vector3f & vec3f) -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point p;
    p.x = vec3f.x();
    p.y = vec3f.y();
    p.z = vec3f.z();
    return p;
  };

  Eigen::Vector3f binormal;
  binormal << -tangent.y(), tangent.x(), tangent.z();

  msg.points.push_back(cast2gp(pos + tangent + binormal));
  msg.points.push_back(cast2gp(pos + tangent - binormal));
  msg.points.push_back(cast2gp(pos - tangent - binormal));
  msg.points.push_back(cast2gp(pos - tangent + binormal));
  msg.points.push_back(cast2gp(pos + tangent + binormal));

  pub_marker_->publish(msg);
}

Eigen::Vector3f ParticleInitializer::tangentDirection(
  const lanelet::Lanelet & lane, const Eigen::Vector3f & position)
{
  auto castVec3f = [](const lanelet::ConstPoint3d & p) -> Eigen::Vector3f {
    Eigen::Vector3d vec_3d(p.x(), p.y(), p.z());
    return vec_3d.cast<float>();
  };

  float min_distance = std::numeric_limits<float>::max();
  size_t min_index = -1;
  auto centerline = lane.centerline3d();

  for (const auto & e : centerline | boost::adaptors::indexed(0)) {
    const lanelet::ConstPoint3d & p = e.value();
    float dp = (castVec3f(p) - position).norm();
    if (dp < min_distance) {
      min_distance = dp;
      min_index = e.index();
    }
  }

  Eigen::Vector3f from, to;
  if (min_index == centerline.size() - 1) {
    from = castVec3f(*std::next(centerline.begin(), min_index - 1));
    to = castVec3f(*std::next(centerline.begin(), min_index));
  } else {
    from = castVec3f(*std::next(centerline.begin(), min_index));
    to = castVec3f(*std::next(centerline.begin(), min_index + 1));
  }

  return (to - from).normalized();
}

void ParticleInitializer::publishRectifiedInitialpose(
  const Eigen::Vector3f & pos, const Eigen::Vector3f & tangent,
  const PoseCovStamped & raw_initialpose)
{
  PoseCovStamped msg = raw_initialpose;
  msg.pose.pose.position.x = pos.x();
  msg.pose.pose.position.y = pos.y();
  msg.pose.pose.position.z = pos.z();

  float theta = std::atan2(tangent.y(), tangent.x());

  msg.pose.pose.orientation.w = std::cos(theta / 2);
  msg.pose.pose.orientation.x = 0;
  msg.pose.pose.orientation.y = 0;
  msg.pose.pose.orientation.z = std::sin(theta / 2);

  Eigen::Matrix2f cov;
  cov << cov_xx_yy_(0), 0, 0, cov_xx_yy_(1);
  Eigen::Rotation2D r(theta);
  cov = r * cov * r.inverse();

  msg.pose.covariance.at(6 * 0 + 0) = cov(0, 0);
  msg.pose.covariance.at(6 * 0 + 1) = cov(0, 1);
  msg.pose.covariance.at(6 * 1 + 0) = cov(1, 0);
  msg.pose.covariance.at(6 * 1 + 1) = cov(1, 1);
  msg.pose.covariance.at(6 * 5 + 5) = 0.0076;  // 0.0076 = (5deg)^2

  pub_initialpose_->publish(msg);
}

}  // namespace modularized_particle_filter