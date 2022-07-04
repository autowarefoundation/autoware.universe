#include "map/ground_server.hpp"
#include "map/ll2_util.hpp"

namespace map
{
GroundServer::GroundServer() : Node("ground_server")
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();

  auto cb_pose = std::bind(&GroundServer::callbackPoseStamped, this, _1);
  auto cb_map = std::bind(&GroundServer::callbackMap, this, _1);
  auto cb_service = std::bind(&GroundServer::callbackService, this, _1, _2);

  sub_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, cb_map);
  sub_pose_stamped_ = create_subscription<PoseStamped>("/particle_pose", 10, cb_pose);

  pub_ground_height_ = create_publisher<Float32>("/height", 10);

  service_ = create_service<Ground>("ground", cb_service);
}

void GroundServer::callbackPoseStamped(const PoseStamped & msg)
{
  if (kdtree_ == nullptr) return;
  Pose pose = computeGround(msg.pose.position);

  Float32 data;
  data.data = pose.position.z;
  pub_ground_height_->publish(data);
}

void GroundServer::callbackMap(const HADMapBin & msg)
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

geometry_msgs::msg::Pose GroundServer::computeGround(const geometry_msgs::msg::Point & point)
{
  constexpr int K = 10;
  std::vector<int> indices;
  std::vector<float> distances;
  pcl::PointXYZ p;
  p.x = point.x;
  p.y = point.y;
  p.z = 0;
  kdtree_->nearestKSearch(p, K, indices, distances);

  float height = std::numeric_limits<float>::max();
  for (int index : indices) {
    Eigen::Vector3f v = cloud_->at(index).getVector3fMap();
    height = std::min(height, v.z());
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = point.x;
  pose.position.y = point.y;
  pose.position.z = height;

  // TODO: compute orientation

  return pose;
}

void GroundServer::callbackService(
  const std::shared_ptr<Ground::Request> request, std::shared_ptr<Ground::Response> response)
{
  if (kdtree_ == nullptr) return;
  Pose pose = computeGround(request->point);
  response->pose.position.x = pose.position.x;
  response->pose.position.y = pose.position.y;
  response->pose.position.z = pose.position.z;
}

}  // namespace map
