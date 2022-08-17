#include "map/ll2_util.hpp"
#include "particle_filter/particle_initializer.hpp"

#include <boost/range/adaptors.hpp>

namespace particle_filter
{
ParticleInitializer::ParticleInitializer() : Node("particle_initializer")
{
  using std::placeholders::_1;
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();

  // Publisher
  pub_particle_ = create_publisher<ParticleArray>("initial_particles", 10);
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

  int nearest_index = searchLandingLanelet(pos_vec3f);
  lanelet::Id id = lanelet::InvalId;
  if (nearest_index >= 0) id = point_id_to_lanelet_id_.at(nearest_index);
  if (id == lanelet::InvalId) return;

  pos_vec3f.z() = cloud_->at(nearest_index).z;

  const lanelet::Lanelet lane = *lanelet_map_->laneletLayer.find(id);
  Eigen::Vector3f tangent = tangentDirection(lane, pos_vec3f);
  publishRangeMarker(pos_vec3f, tangent);
}

void ParticleInitializer::onMap(const HADMapBin & bin_map)
{
  lanelet_map_ = fromBinMsg(bin_map);

  cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

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

  kdtree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  kdtree_->setInputCloud(cloud_);
}

int ParticleInitializer::searchLandingLanelet(const Eigen::Vector3f & pos)
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
  msg.scale.x = 0.5;
  msg.scale.y = 0.5;

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

  msg.points.push_back(cast2gp(pos + tangent));
  msg.points.push_back(cast2gp(pos + binormal));
  msg.points.push_back(cast2gp(pos - tangent));
  msg.points.push_back(cast2gp(pos - binormal));
  msg.points.push_back(cast2gp(pos + tangent));

  pub_marker_->publish(msg);
}

Eigen::Vector3f ParticleInitializer::tangentDirection(
  const lanelet::Lanelet & lane, const Eigen::Vector3f & position)
{
  auto castVec3f = [](const lanelet::ConstPoint3d & p) -> Eigen::Vector3f {
    return {p.x(), p.y(), p.z()};
  };

  float min_distance = std::numeric_limits<float>::max();
  int min_index;
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

}  // namespace particle_filter