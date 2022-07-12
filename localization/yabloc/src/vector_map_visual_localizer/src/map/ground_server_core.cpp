#include "common/util.hpp"
#include "map/ground_server.hpp"
#include "map/ll2_util.hpp"

#include <Eigen/Eigenvalues>

namespace map
{
GroundServer::GroundServer() : Node("ground_server"), vector_buffer_(40)
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
  pub_ground_plane_ = create_publisher<Float32Array>("/ground", 10);
  pub_marker_ = create_publisher<Marker>("/ground_marker", 10);
  pub_string_ = create_publisher<String>("/ground_status", 10);
  pub_near_cloud_ = create_publisher<PointCloud2>("/near_cloud", 10);

  service_ = create_service<Ground>("ground", cb_service);
}

void GroundServer::callbackPoseStamped(const PoseStamped & msg)
{
  if (kdtree_ == nullptr) return;
  common::GroundPlane ground_plane = computeGround(msg.pose.position, true);

  Eigen::Vector3f normal = ground_plane.normal;
  vector_buffer_.push_back(normal);

  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (const Eigen::Vector3f & v : vector_buffer_) {
    mean += v;
  }
  mean /= vector_buffer_.size();
  ground_plane.normal = mean.normalized();

  Float32 data;
  data.data = ground_plane.height();

  pub_ground_height_->publish(data);
  pub_ground_plane_->publish(ground_plane.msg());

  {
    std::stringstream ss;
    ss << "--- Ground Estimator Status ----" << std::endl;
    ss << std::fixed << std::setprecision(2);
    ss << "height: " << ground_plane.height() << std::endl;
    float cos = ground_plane.normal.dot(Eigen::Vector3f::UnitZ());
    ss << "tilt: " << std::acos(cos) * 180 / 3.14 << " deg" << std::endl;
    float raw_cos = normal.dot(Eigen::Vector3f::UnitZ());
    ss << "raw tilt: " << std::acos(raw_cos) * 180 / 3.14 << " deg" << std::endl;

    String string_msg;
    string_msg.data = ss.str();
    pub_string_->publish(string_msg);
  }

  {
    pcl::PointCloud<pcl::PointXYZ> near_cloud;
    for (int index : last_near_point_indices_) near_cloud.push_back(cloud_->at(index));
    if (!near_cloud.empty()) util::publishCloud(*pub_near_cloud_, near_cloud, msg.header.stamp);
  }

  // TODO: current visual marker is so useless
  // publishMarker(ground_plane);
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

common::GroundPlane GroundServer::computeGround(
  const geometry_msgs::msg::Point & point, bool logging)
{
  constexpr int K = 30;
  std::vector<int> indices;
  std::vector<float> distances;
  pcl::PointXYZ p;
  p.x = point.x;
  p.y = point.y;
  p.z = 0;
  kdtree_->nearestKSearch(p, K, indices, distances);

  if (logging) last_near_point_indices_ = indices;

  float height = std::numeric_limits<float>::max();
  for (int index : indices) {
    Eigen::Vector3f v = cloud_->at(index).getVector3fMap();
    height = std::min(height, v.z());
  }

  common::GroundPlane plane;
  plane.xyz.x() = point.x;
  plane.xyz.y() = point.y;
  plane.xyz.z() = height;
  plane.normal = Eigen::Vector3f::UnitZ();

  Eigen::Matrix3f covariance;
  Eigen::Vector4f centroid;
  centroid << point.x, point.y, height, 0;
  pcl::computeCovarianceMatrix(*cloud_, indices, centroid, covariance);

  Eigen::Vector4f plane_parameter;
  float curvature;
  pcl::solvePlaneParameters(covariance, centroid, plane_parameter, curvature);
  Eigen::Vector3f normal = plane_parameter.topRows(3);
  if (normal.z() < 0) normal = -normal;

  Eigen::Vector3f nn = cloud_->at(indices.front()).getVector3fMap();
  height = (normal.dot(nn) - nn.x() * normal.x() - nn.y() * normal.y()) / normal.z();
  plane.xyz.z() = height;
  plane.normal = normal;
  return plane;
}

void GroundServer::callbackService(
  const std::shared_ptr<Ground::Request> request, std::shared_ptr<Ground::Response> response)
{
  if (kdtree_ == nullptr) return;
  common::GroundPlane plane = computeGround(request->point);
  response->pose.position.x = plane.xyz.x();
  response->pose.position.y = plane.xyz.y();
  response->pose.position.z = plane.xyz.z();
}

void GroundServer::publishMarker(const common::GroundPlane & plane)
{
  Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = get_clock()->now();
  marker.type = Marker::TRIANGLE_LIST;
  marker.id = 0;
  marker.color = util::color(0.0, 0.5, 0.0, 0.1);
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.pose.position.x = plane.xyz.x();
  marker.pose.position.y = plane.xyz.y();
  marker.pose.position.z = plane.xyz.z();

  Eigen::Vector3f n = plane.normal;
  const int L = 20;
  auto toPoint = [](float x, float y, float z) -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  };
  auto p1 = toPoint(L, 0, -L * n.x() / n.z());
  auto p2 = toPoint(0, L, -L * n.y() / n.z());
  auto p3 = toPoint(-L, 0, L * n.x() / n.z());
  auto p4 = toPoint(0, -L, L * n.y() / n.z());

  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.points.push_back(p3);

  marker.points.push_back(p3);
  marker.points.push_back(p4);
  marker.points.push_back(p1);
  pub_marker_->publish(marker);
}

}  // namespace map
