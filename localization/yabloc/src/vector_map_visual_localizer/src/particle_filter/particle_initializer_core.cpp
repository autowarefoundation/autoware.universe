#include "map/ll2_util.hpp"
#include "particle_filter/particle_initializer.hpp"

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
  searchLandingLanelet(pos_vec3f);
  publishRangeMarker(pos_vec3f);
}

void ParticleInitializer::onMap(const HADMapBin & bin_map)
{
  lanelet::LaneletMapPtr lanelet_map = fromBinMsg(bin_map);

  cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  auto toPointXYZ = [](const lanelet::ConstPoint3d & p) -> pcl::PointXYZ {
    pcl::PointXYZ xyz;
    xyz.x = p.x();
    xyz.x = p.y();
    xyz.x = p.z();
    return xyz;
  };

  for (const lanelet::Lanelet & lane : lanelet_map->laneletLayer) {
    for (const lanelet::ConstPoint3d & p : lane.centerline3d()) {
      cloud_->push_back(toPointXYZ(p));
    }
  }

  kdtree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  kdtree_->setInputCloud(cloud_);
}

void ParticleInitializer::searchLandingLanelet(const Eigen::Vector3f & pos)
{
  pcl::PointXYZ query(pos.x(), pos.y(), pos.z());
  std::vector<int> indices;
  std::vector<float> distances;
  kdtree_->nearestKSearch(query, 1, indices, distances);

  if (distances.front() > (3 * 3)) {
    RCLCPP_WARN_STREAM(get_logger(), "nearest neighbor is too far");
  }

  cloud_->at(indices.front());

  return;
}

void ParticleInitializer::publishRangeMarker(const Eigen::Vector3f & pos)
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

  msg.points.push_back(cast2gp(pos + Eigen::Vector3f(1, 1, 0)));
  msg.points.push_back(cast2gp(pos + Eigen::Vector3f(1, -1, 0)));
  msg.points.push_back(cast2gp(pos + Eigen::Vector3f(-1, -1, 0)));
  msg.points.push_back(cast2gp(pos + Eigen::Vector3f(-1, 1, 0)));
  msg.points.push_back(cast2gp(pos + Eigen::Vector3f(1, 1, 0)));

  pub_marker_->publish(msg);
}

}  // namespace particle_filter