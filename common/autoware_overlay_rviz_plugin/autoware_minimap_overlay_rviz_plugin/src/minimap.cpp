// Copyright 2024 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "include/minimap.hpp"

#include <QPainter>
#include <QPainterPath>
#include <QVBoxLayout>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_rendering/render_system.hpp>

#include <qimage.h>

#include <cmath>

namespace autoware_minimap_overlay_rviz_plugin
{

VehicleMapDisplay::VehicleMapDisplay() : rviz_common::Display(), overlay_(nullptr)
{
  property_width_ = new rviz_common::properties::IntProperty(
    "Width", 256, "Width of the overlay max:500", this, SLOT(updateOverlaySize()));

  property_width_->setMax(500);
  property_width_->setShouldBeSaved(true);

  property_height_ = new rviz_common::properties::IntProperty(
    "Height", 256, "Height of the overlay max:500", this, SLOT(updateOverlaySize()));

  property_height_->setMax(500);
  property_height_->setShouldBeSaved(true);

  property_left_ = new rviz_common::properties::IntProperty(
    "Left", 10, "Left position of the overlay", this, SLOT(updateOverlayPosition()));
  property_top_ = new rviz_common::properties::IntProperty(
    "Top", 10, "Top position of the overlay", this, SLOT(updateOverlayPosition()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0, "Amount of transparency to apply to the overlay.", this, SLOT(updateOverlaySize()));

  background_color_property_ = new rviz_common::properties::ColorProperty(
    "Background Color", QColor(0, 0, 0), "Color to draw the background.", this,
    SLOT(updateOverlaySize()));

  property_zoom_ = new rviz_common::properties::IntProperty(
    "Zoom", 15, "Zoom level of the map 15-18", this, SLOT(updateZoomLevel()));

  property_zoom_->setMin(15);
  property_zoom_->setMax(18);
  property_zoom_->setShouldBeSaved(true);

  property_latitude_ = new rviz_common::properties::FloatProperty(
    "Latitude", 0.0, "Latitude of the ego", this, SLOT(updateLatitude()));

  property_longitude_ = new rviz_common::properties::FloatProperty(
    "Longitude", 0.0, "Longitude of the ego", this, SLOT(updateLongitude()));

  property_goal_lat = new rviz_common::properties::FloatProperty(
    "Goal Latitude", 0.0, "Goal pose Latitude", this, SLOT(updateGoalPose()));

  property_goal_lon = new rviz_common::properties::FloatProperty(
    "Goal Longitude", 0.0, "Goal pose Longitude", this, SLOT(updateGoalPose()));

  property_origin_lat_ = new rviz_common::properties::FloatProperty(
    "Map Origin Latitude", 35.23808753540768, "Latitude of the map origin position", this,
    SLOT(updateLatitude()));

  property_origin_lon_ = new rviz_common::properties::FloatProperty(
    "Map Origin Longitude", 139.9009591876285, "Longitude of the map origin position", this,
    SLOT(updateLongitude()));

  zoom_ = property_zoom_->getInt();

  latitude_ = property_latitude_->getFloat();
  longitude_ = property_longitude_->getFloat();

  tile_field_ = std::make_unique<TileField>(this);
  connect(tile_field_.get(), &TileField::tilesUpdated, this, &VehicleMapDisplay::onTilesUpdated);
}

VehicleMapDisplay::~VehicleMapDisplay()
{
}

void VehicleMapDisplay::onInitialize()
{
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  static int count = 0;
  std::stringstream ss;
  ss << "AerialMapDisplayObject" << count++;
  overlay_.reset(new rviz_satellite::OverlayObject(ss.str()));
  overlay_->show();
  updateOverlaySize();
  updateOverlayPosition();

  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();

  // Initialize the goal pose subscriber
  goal_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/planning/mission_planning/echo_back_goal_pose", 10,
    std::bind(&VehicleMapDisplay::goalPoseCallback, this, std::placeholders::_1));

  // pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
  //   "/localization/kinematic_state", 10,
  //   std::bind(&VehicleMapDisplay::poseCallback, this, std::placeholders::_1));

  pose_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::VehicleKinematics>(
    "/api/vehicle/kinematics", rclcpp::QoS(10).best_effort(),
    std::bind(&VehicleMapDisplay::poseCallback, this, std::placeholders::_1));

  route_state_sub_ = node_->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state", 10,
    std::bind(&VehicleMapDisplay::routeStateCallback, this, std::placeholders::_1));

  route_points_sub_ = node_->create_subscription<autoware_planning_msgs::msg::Path>(
    "/planning/scenario_planning/lane_driving/behavior_planning/path", 10,
    std::bind(&VehicleMapDisplay::routePointsCallback, this, std::placeholders::_1));
}

void VehicleMapDisplay::reset()
{
}

void VehicleMapDisplay::onEnable()
{
  if (overlay_) {
    overlay_->show();
  }
}

void VehicleMapDisplay::onDisable()
{
  if (overlay_) {
    overlay_->hide();
  }
}

void VehicleMapDisplay::update(float, float)
{
  if (!overlay_) {
    return;
  }

  rviz_satellite::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);
  hud.fill(Qt::transparent);
  drawWidget(hud);
}

void VehicleMapDisplay::updateOverlaySize()
{
  if (!overlay_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  queueRender();
}

void VehicleMapDisplay::updateOverlayPosition()
{
  if (!overlay_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  overlay_->setPosition(
    property_left_->getInt(), property_top_->getInt(), rviz_satellite::HorizontalAlignment::LEFT,
    rviz_satellite::VerticalAlignment::BOTTOM);
  queueRender();
}

void VehicleMapDisplay::drawWidget(QImage & hud)
{
  std::lock_guard<std::mutex> lock(property_mutex_);

  if (!overlay_->isVisible()) {
    return;
  }

  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);

  QRectF backgroundRect(0, 0, hud.width(), hud.height());
  drawCircle(painter, backgroundRect);

  painter.end();
}

void VehicleMapDisplay::drawCircle(QPainter & painter, const QRectF & backgroundRect)
{
  painter.setRenderHint(QPainter::Antialiasing, true);
  QColor colorFromHSV;
  colorFromHSV.setHsv(
    background_color_property_->getColor().hue(),
    background_color_property_->getColor().saturation(),
    background_color_property_->getColor().value());
  colorFromHSV.setAlphaF(alpha_property_->getFloat());

  painter.setBrush(colorFromHSV);

  // Define the visible rectangle
  QRectF visibleRect(
    backgroundRect.width() / 2 - property_width_->getInt() / 2,
    backgroundRect.height() / 2 - property_height_->getInt() / 2, property_width_->getInt(),
    property_height_->getInt());

  // Define the circular clipping path
  QPainterPath path;
  path.addEllipse(visibleRect);
  painter.setClipPath(path);

  // Draw the background
  painter.setPen(Qt::NoPen);
  painter.drawRect(backgroundRect);

  // Get the tile field image
  QImage tile_field_image = tile_field_->getTileFieldImage();

  // Calculate the target rectangle for the tile field image
  auto [x_pixel, y_pixel] = tile_field_->getTileOffsets(latitude_, longitude_);

  // Adjust the target rectangle such that the position icon is centered in the
  // visible rect
  QRectF target(
    backgroundRect.width() / 2 - x_pixel - 128, backgroundRect.height() / 2 - y_pixel - 128,
    tile_field_image.width(), tile_field_image.height());

  QRectF source(0, 0, tile_field_image.width(), tile_field_image.height());

  // Draw the tile field image
  painter.drawImage(target, tile_field_image, source);

  // Draw the position icon
  std::string package_path =
    ament_index_cpp::get_package_share_directory("autoware_minimap_overlay_rviz_plugin");
  std::string image_path = package_path + "/icons/pos.png";
  QImage pos_image = QImage(image_path.c_str());
  pos_image = pos_image.scaled(20, 20, Qt::KeepAspectRatio);

  QPointF positionInOverlay =
    backgroundRect.center() - QPointF(pos_image.width() / 2, pos_image.height() - 10);

  painter.drawImage(positionInOverlay, pos_image);

  // Draw the goal pose only if it is set

  if (route_state_msg_ && route_state_msg_->state == autoware_adapi_v1_msgs::msg::RouteState::SET) {
    goal_pose_.draw(painter, backgroundRect, zoom_);
    path_overlay_.draw(painter, backgroundRect, zoom_);
  }

  queueRender();
}

void VehicleMapDisplay::onTilesUpdated()
{
  queueRender();
}

void VehicleMapDisplay::updateZoomLevel()
{
  zoom_ = property_zoom_->getInt();  // Update the zoom level

  // Update the map position based on the new zoom level
  if (pose_msg_) {
    poseCallback(pose_msg_);
  }

  if (goal_pose_msg_) {
    goalPoseCallback(goal_pose_msg_);
  }

  if (route_points_msg_) {
    routePointsCallback(route_points_msg_);
  }

  tile_field_->fetchTiles(zoom_, center_x_tile_, center_y_tile_);
  queueRender();  // Request re-rendering
}

void VehicleMapDisplay::updateLatitude()
{
  latitude_ = property_latitude_->getFloat();
  int new_center_x_tile = tile_field_->long_to_tile_x(longitude_, zoom_);
  int new_center_y_tile = tile_field_->lat_to_tile_y(latitude_, zoom_);
  center_x_tile_ = new_center_x_tile;
  center_y_tile_ = new_center_y_tile;
  tile_field_->fetchTiles(zoom_, center_x_tile_, center_y_tile_);
  queueRender();
}

void VehicleMapDisplay::updateLongitude()
{
  longitude_ = property_longitude_->getFloat();
  int new_center_x_tile = tile_field_->long_to_tile_x(longitude_, zoom_);
  int new_center_y_tile = tile_field_->lat_to_tile_y(latitude_, zoom_);
  center_x_tile_ = new_center_x_tile;
  center_y_tile_ = new_center_y_tile;
  tile_field_->fetchTiles(zoom_, center_x_tile_, center_y_tile_);
  queueRender();
}

void VehicleMapDisplay::updateMapPosition()
{
  int new_center_x_tile = tile_field_->long_to_tile_x(longitude_, zoom_);
  int new_center_y_tile = tile_field_->lat_to_tile_y(latitude_, zoom_);
  center_x_tile_ = new_center_x_tile;
  center_y_tile_ = new_center_y_tile;
  tile_field_->fetchTiles(zoom_, new_center_x_tile, new_center_y_tile);
  queueRender();
}

void VehicleMapDisplay::updateGoalPose()
{
  double goal_x = property_goal_lat->getFloat();
  double goal_y = property_goal_lon->getFloat();
  double origin_lat = property_origin_lat_->getFloat();
  double origin_lon = property_origin_lon_->getFloat();
  goal_pose_.setGoalPosition(goal_x, goal_y, origin_lat, origin_lon);
  if (!goal_pose_msg_) {
    return;
  }
  goalPoseCallback(goal_pose_msg_);

  if (route_points_msg_) routePointsCallback(route_points_msg_);

  queueRender();
}

void VehicleMapDisplay::routePointsCallback(const autoware_planning_msgs::msg::Path::SharedPtr msg)
{
  route_points_msg_ = msg;

  // Convert the route points to local coordinates and set them in the RouteOverlay
  std::vector<PathPoint> path_points;
  for (const auto & pose : msg->points) {
    PathPoint point;
    point.x = pose.pose.position.x;
    point.y = pose.pose.position.y;
    path_points.push_back(point);
  }

  double origin_lat = property_origin_lat_->getFloat();
  double origin_lon = property_origin_lon_->getFloat();
  path_overlay_.setPathPoints(path_points, origin_lat, origin_lon);
  path_overlay_.setVehiclePosition(latitude_, longitude_);

  queueRender();
}

void VehicleMapDisplay::routeStateCallback(
  const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg)
{
  route_state_msg_ = msg;

  queueRender();
}

// void VehicleMapDisplay::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
void VehicleMapDisplay::poseCallback(
  const autoware_adapi_v1_msgs::msg::VehicleKinematics::SharedPtr msg)
{
  pose_msg_ = msg;

  latitude_ = msg->geographic_pose.position.latitude;
  longitude_ = msg->geographic_pose.position.longitude;

  property_longitude_->setFloat(longitude_);
  property_latitude_->setFloat(latitude_);

  // Set the vehicle position in the goal pose (in case it was not set yet)
  goal_pose_.setVehiclePosition(latitude_, longitude_);
  path_overlay_.setVehiclePosition(latitude_, longitude_);

  updateMapPosition();  // Method to update map position based on new latitude and longitude
}

void VehicleMapDisplay::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_pose_msg_ = msg;

  double origin_lat = property_origin_lat_->getFloat();
  double origin_lon = property_origin_lon_->getFloat();
  goal_pose_.setGoalPosition(msg->pose.position.x, msg->pose.position.y, origin_lat, origin_lon);

  // Set the vehicle position in the goal pose (in case it was not set yet)
  goal_pose_.setVehiclePosition(latitude_, longitude_);
  path_overlay_.setVehiclePosition(latitude_, longitude_);

  property_goal_lat->setFloat(goal_pose_.getGoalLatitude());
  property_goal_lon->setFloat(goal_pose_.getGoalLongitude());
  queueRender();
}

std::pair<double, double> VehicleMapDisplay::localXYZToLatLon(double x, double y)
{
  int zone;
  bool northp;
  double origin_lat = property_origin_lat_->getFloat();
  double origin_lon = property_origin_lon_->getFloat();
  double origin_x, origin_y, gamma, k;

  // Convert origin to UTM coordinates
  GeographicLib::UTMUPS::Forward(
    origin_lat, origin_lon, zone, northp, origin_x, origin_y, gamma, k);

  // Calculate global UTM coordinates by adding local offsets
  double global_x = origin_x + x;
  double global_y = origin_y + y;

  // Convert back to geographic coordinates
  double lat, lon;
  GeographicLib::UTMUPS::Reverse(zone, northp, global_x, global_y, lat, lon);

  return {lat, lon};
}

}  // namespace autoware_minimap_overlay_rviz_plugin

PLUGINLIB_EXPORT_CLASS(
  autoware_minimap_overlay_rviz_plugin::VehicleMapDisplay, rviz_common::Display)
