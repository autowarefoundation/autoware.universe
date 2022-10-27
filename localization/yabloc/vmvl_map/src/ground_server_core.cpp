#include "vmvl_map/ground_server.hpp"
#include "vmvl_map/ll2_util.hpp"
#include "vmvl_map/polygon_operation.hpp"

#include <Eigen/Eigenvalues>
#include <vml_common/color.hpp>
#include <vml_common/pub_sub.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace map
{
GroundServer::GroundServer()
: Node("ground_server"),
  force_zero_tilt_(declare_parameter("force_zero_tilt", false)),
  R(declare_parameter("R", 20)),
  K(declare_parameter("K", 50)),
  vector_buffer_(50)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();

  auto on_pose = std::bind(&GroundServer::onPoseStamped, this, _1);
  auto on_map = std::bind(&GroundServer::onMap, this, _1);
  auto on_service = std::bind(&GroundServer::onService, this, _1, _2);

  service_ = create_service<Ground>("ground", on_service);
  sub_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, on_map);
  sub_pose_stamped_ = create_subscription<PoseStamped>("particle_pose", 10, on_pose);

  pub_ground_height_ = create_publisher<Float32>("height", 10);
  pub_ground_plane_ = create_publisher<Float32Array>("ground", 10);
  pub_marker_ = create_publisher<Marker>("ground_marker", 10);
  pub_string_ = create_publisher<String>("ground_status", 10);
  pub_near_cloud_ = create_publisher<PointCloud2>("near_cloud", 10);
}

void GroundServer::onPoseStamped(const PoseStamped & msg)
{
  if (kdtree_ == nullptr) return;
  GroundPlane ground_plane = estimateGround(msg.pose.position);

  // Publish value msg
  Float32 data;
  data.data = ground_plane.height();
  pub_ground_height_->publish(data);
  pub_ground_plane_->publish(ground_plane.msg());

  // Publish string msg
  {
    std::stringstream ss;
    ss << "--- Ground Estimator Status ----" << std::endl;
    ss << std::fixed << std::setprecision(2);
    ss << "height: " << ground_plane.height() << std::endl;
    float cos = ground_plane.normal.dot(Eigen::Vector3f::UnitZ());
    ss << "tilt: " << std::acos(cos) * 180 / 3.14 << " deg" << std::endl;
    float raw_cos = vector_buffer_.back().dot(Eigen::Vector3f::UnitZ());
    ss << "raw tilt: " << std::acos(raw_cos) * 180 / 3.14 << " deg" << std::endl;

    String string_msg;
    string_msg.data = ss.str();
    pub_string_->publish(string_msg);
  }

  // Publish nearest point cloud for debug
  {
    pcl::PointCloud<pcl::PointXYZ> near_cloud;
    for (int index : last_near_point_indices_) near_cloud.push_back(cloud_->at(index));
    if (!near_cloud.empty())
      vml_common::publishCloud(*pub_near_cloud_, near_cloud, msg.header.stamp);
  }
}

void GroundServer::upsampleLineString(
  const lanelet::ConstPoint3d & from, const lanelet::ConstPoint3d & to,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  Eigen::Vector3f f(from.x(), from.y(), from.z());
  Eigen::Vector3f t(to.x(), to.y(), to.z());
  float length = (t - f).norm();
  Eigen::Vector3f d = (t - f).normalized();
  for (float l = 0; l < length; l += 0.5f) {
    pcl::PointXYZ xyz;
    xyz.getVector3fMap() = (f + l * d);
    cloud->push_back(xyz);
  }
};

pcl::PointCloud<pcl::PointXYZ> GroundServer::sampleFromPolygons(
  const lanelet::PolygonLayer & polygons)
{
  pcl::PointCloud<pcl::PointXYZ> raw_cloud;

  if (polygons.size() >= 2) {
    RCLCPP_FATAL_STREAM(get_logger(), "current ground server does not handle multi polygons");
  }

  for (const lanelet::ConstPolygon3d & polygon : polygons) {
    for (const lanelet::ConstPoint3d & p : polygon) {
      pcl::PointXYZ xyz;
      xyz.x = p.x();
      xyz.y = p.y();
      xyz.z = p.z();
      raw_cloud.push_back(xyz);
    }
  }
  return fillPointsInPolygon(raw_cloud);
}

void GroundServer::onMap(const HADMapBin & msg)
{
  lanelet::LaneletMapPtr lanelet_map = fromBinMsg(msg);

  // TODO: has to be loaded from rosparm
  const std::set<std::string> visible_labels = {
    "zebra_marking",      "virtual",   "line_thin", "line_thick",
    "pedestrian_marking", "stop_line", "curbstone"};

  pcl::PointCloud<pcl::PointXYZ>::Ptr upsampled_cloud =
    pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  for (lanelet::LineString3d & line : lanelet_map->lineStringLayer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;

    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;

    lanelet::ConstPoint3d const * from = nullptr;
    for (const lanelet::ConstPoint3d & p : line) {
      if (from != nullptr) upsampleLineString(*from, p, upsampled_cloud);
      from = &p;
    }
  }

  *upsampled_cloud += sampleFromPolygons(lanelet_map->polygonLayer);

  cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  // Voxel
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(upsampled_cloud);
  filter.setLeafSize(1.0f, 1.0f, 1.0f);
  filter.filter(*cloud_);

  kdtree_ = pcl::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  kdtree_->setInputCloud(cloud_);
}

std::vector<int> mergeIndices(const std::vector<int> & indices1, const std::vector<int> & indices2)
{
  std::unordered_set<int> set;
  for (int i : indices1) set.insert(i);
  for (int i : indices2) set.insert(i);

  std::vector<int> indices;
  indices.assign(set.begin(), set.end());
  return indices;
}

float GroundServer::estimateHeightSimply(const geometry_msgs::msg::Point & point) const
{
  // Return the height of the nearest point
  // NOTE: Sometimes it might offer not accurate height
  std::vector<int> k_indices;
  std::vector<float> distances;
  pcl::PointXYZ p;
  p.x = point.x;
  p.y = point.y;
  p.z = 0;
  kdtree_->nearestKSearch(p, 1, k_indices, distances);
  return cloud_->at(k_indices.front()).z;
}

std::vector<int> GroundServer::ransacEstimation(const std::vector<int> & indices_raw)
{
  pcl::PointIndicesPtr indices(new pcl::PointIndices);
  indices->indices = indices_raw;

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(1.0);
  seg.setProbability(0.6);

  seg.setInputCloud(cloud_);
  seg.setIndices(indices);
  seg.segment(*inliers, *coefficients);
  return inliers->indices;
}

GroundServer::GroundPlane GroundServer::estimateGround(const Point & point)
{
  std::vector<int> k_indices, r_indices;
  std::vector<float> distances;
  pcl::PointXYZ xyz;
  xyz.x = point.x;
  xyz.y = point.y;
  xyz.z = estimateHeightSimply(point);
  kdtree_->nearestKSearch(xyz, K, k_indices, distances);
  kdtree_->radiusSearch(xyz, R, r_indices, distances);

  std::vector<int> raw_indices = mergeIndices(k_indices, r_indices);
  std::vector<int> indices = ransacEstimation(raw_indices);
  if (indices.empty()) indices = raw_indices;
  last_near_point_indices_ = indices;

  // Estimate normal vector using covariance matrix around the target point
  Eigen::Matrix3f covariance;
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud_, indices, centroid);
  pcl::computeCovarianceMatrix(*cloud_, indices, centroid, covariance);

  Eigen::Vector4f plane_parameter;
  float curvature;
  pcl::solvePlaneParameters(covariance, centroid, plane_parameter, curvature);
  Eigen::Vector3f normal = plane_parameter.topRows(3);
  if (normal.z() < 0) normal = -normal;

  // Remove NaN
  if (!normal.allFinite()) {
    normal = Eigen::Vector3f::UnitZ();
    RCLCPP_WARN_STREAM(get_logger(), "Reject NaN tilt");
  }
  // Remove too large tilt
  if ((normal.dot(Eigen::Vector3f::UnitZ())) < 0.707) {
    normal = Eigen::Vector3f::UnitZ();
    RCLCPP_WARN_STREAM(get_logger(), "Reject too large tilt of ground");
  }

  // Smoothing (moving averaging)
  vector_buffer_.push_back(normal);
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (const Eigen::Vector3f & v : vector_buffer_) mean += v;
  mean /= vector_buffer_.size();

  GroundPlane plane;
  plane.xyz.x() = point.x;
  plane.xyz.y() = point.y;
  plane.normal = mean.normalized();

  if (force_zero_tilt_) plane.normal = Eigen::Vector3f::UnitZ();

  {
    Eigen::Vector3f center = centroid.topRows(3);
    float inner = center.dot(plane.normal);
    float px_nx = point.x * plane.normal.x();
    float py_ny = point.y * plane.normal.y();
    plane.xyz.z() = (inner - px_nx - py_ny) / plane.normal.z();
  }

  return plane;
}

void GroundServer::onService(
  const std::shared_ptr<Ground::Request> request, std::shared_ptr<Ground::Response> response)
{
  if (kdtree_ == nullptr) return;
  float z = estimateHeightSimply(request->point);
  response->pose.position.x = request->point.x;
  response->pose.position.y = request->point.y;
  response->pose.position.z = z;
}

void GroundServer::publishMarker(const GroundPlane & plane)
{
  // TODO: current visual marker is so useless
  Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = get_clock()->now();
  marker.type = Marker::TRIANGLE_LIST;
  marker.id = 0;
  marker.color = vml_common::Color(0.0, 0.5, 0.0, 0.1);
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
