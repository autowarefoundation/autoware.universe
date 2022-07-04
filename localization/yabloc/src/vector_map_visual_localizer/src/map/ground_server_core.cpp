#include "map/ground_server.hpp"
#include "map/ll2_util.hpp"

namespace map
{
GroundServer::GroundServer() : Node("ground_server")
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();

  auto cb_map = std::bind(&GroundServer::mapCallback, this, _1);
  auto cb_service = std::bind(&GroundServer::computeGround, this, _1, _2);

  sub_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, cb_map);
  service_ = create_service<Ground>("ground", cb_service);
}

void GroundServer::mapCallback(const HADMapBin & msg)
{
  lanelet::LaneletMapPtr lanelet_map = fromBinMsg(msg);

  cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  auto toPointXYZ = [](const lanelet::ConstPoint3d & p) -> pcl::PointXYZ {
    pcl::PointXYZ q;
    q.x = p.x();
    q.y = p.y();
    q.z = p.z();
    return q;
  };

  const std::set<std::string> visible_labels = {
    "zebra_marking", "virtual", "line_thin", "line_thick", "pedestrian_marking", "stop_line"};

  for (lanelet::LineString3d & line : lanelet_map->lineStringLayer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;

    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;
    for (const lanelet::ConstPoint3d p : line) {
      cloud_->push_back(toPointXYZ(p));
    }
  }

  kdtree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  kdtree_->setInputCloud(cloud_);
}

void GroundServer::computeGround(
  const std::shared_ptr<Ground::Request> request, std::shared_ptr<Ground::Response> response)
{
  if (kdtree_ == nullptr) return;

  constexpr int K = 10;
  std::vector<int> indices;
  std::vector<float> distances;
  pcl::PointXYZ p;
  p.x = request->point.x;
  p.y = request->point.y;
  p.z = 0;
  kdtree_->nearestKSearch(p, K, indices, distances);

  float height = std::numeric_limits<float>::max();
  for (int index : indices) {
    Eigen::Vector3f v = cloud_->at(index).getVector3fMap();
    height = std::min(height, v.z());
  }

  response->pose.position.x = request->point.x;
  response->pose.position.y = request->point.y;
  response->pose.position.z = height;

  // TODO: compute orientation
}

}  // namespace map
