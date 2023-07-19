// Copyright 2021 Apex.AI, Inc.
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
#ifndef OBJECT_DETECTION__OBJECT_POLYGON_DISPLAY_BASE_HPP_
#define OBJECT_DETECTION__OBJECT_POLYGON_DISPLAY_BASE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_common.hpp"

#include <common/color_alpha_property.hpp>
#include <object_detection/object_polygon_detail.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <rviz_default_plugins/displays/marker_array/marker_array_display.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <visibility_control.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <boost/geometry.hpp>
#include <boost/optional.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/pcl_search.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <bitset>
#include <deque>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{

using Polygon2d = tier4_autoware_utils::Polygon2d;
using Shape = autoware_auto_perception_msgs::msg::Shape;

// struct for creating objects buffer
struct object_info
{
  Shape shape;
  geometry_msgs::msg::Pose position;
  std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> classification;
};

inline pcl::PointXYZRGB toPCL(const double x, const double y, const double z)
{
  pcl::PointXYZRGB pcl_point;
  pcl_point.x = x;
  pcl_point.y = y;
  pcl_point.z = z;
  return pcl_point;
}

inline pcl::PointXYZRGB toPCL(const geometry_msgs::msg::Point & point)
{
  return toPCL(point.x, point.y, point.z);
}

// Define a custom comparator function to compare pointclouds by timestamp
inline bool comparePointCloudsByTimestamp(
  const sensor_msgs::msg::PointCloud2 & pc1, const sensor_msgs::msg::PointCloud2 & pc2)
{
  return (static_cast<int>(pc1.header.stamp.nanosec - pc2.header.stamp.nanosec)) < 0;
}

/// \brief Base rviz plugin class for all object msg types. The class defines common properties
///        for the plugin and also defines common helper functions that can be used by its derived
///        classes.
/// \tparam MsgT PredictedObjects or TrackedObjects or DetectedObjects type
template <typename MsgT>
class AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC ObjectPolygonDisplayBase
: public rviz_common::RosTopicDisplay<MsgT>
{
public:
  using Color = std::array<float, 3U>;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerCommon = rviz_default_plugins::displays::MarkerCommon;
  using ObjectClassificationMsg = autoware_auto_perception_msgs::msg::ObjectClassification;
  using RosTopicDisplay = rviz_common::RosTopicDisplay<MsgT>;

  using PolygonPropertyMap =
    std::unordered_map<ObjectClassificationMsg::_label_type, common::ColorAlphaProperty>;

  explicit ObjectPolygonDisplayBase(
    const std::string & default_topic, const std::string & default_pointcloud_topic)
  : point_cloud_common(new rviz_default_plugins::PointCloudCommon(this)),
    m_marker_common(this),
    m_display_label_property{"Display Label", true, "Enable/disable label visualization", this},
    m_display_uuid_property{"Display UUID", true, "Enable/disable uuid visualization", this},
    m_display_pose_with_covariance_property{
      "Display PoseWithCovariance", true, "Enable/disable pose with covariance visualization",
      this},
    m_display_velocity_text_property{
      "Display Velocity", true, "Enable/disable velocity text visualization", this},
    m_display_acceleration_text_property{
      "Display Acceleration", true, "Enable/disable acceleration text visualization", this},
    m_display_twist_property{"Display Twist", true, "Enable/disable twist visualization", this},
    m_display_predicted_paths_property{
      "Display Predicted Paths", true, "Enable/disable predicted paths visualization", this},
    m_display_path_confidence_property{
      "Display Predicted Path Confidence", true, "Enable/disable predicted paths visualization",
      this},
    m_line_width_property{"Line Width", 0.03, "Line width of object-shape", this},
    m_default_topic{default_topic},
    qos_profile_points(5)
  {
    m_display_type_property = new rviz_common::properties::EnumProperty(
      "Polygon Type", "3d", "Type of the polygon to display object.", this);
    // Option values here must correspond to indices in palette_textures_ array in onInitialize()
    // below.
    m_display_type_property->addOption("3d", 0);
    m_display_type_property->addOption("2d", 1);
    m_display_type_property->addOption("Disable", 2);
    m_simple_visualize_mode_property = new rviz_common::properties::EnumProperty(
      "Visualization Type", "Normal", "Simplicity of the polygon to display object.", this);
    m_simple_visualize_mode_property->addOption("Normal", 0);
    m_simple_visualize_mode_property->addOption("Simple", 1);
    m_publish_objs_pointcloud = new rviz_common::properties::BoolProperty(
      "Publish Objects Pointcloud", true, "Enable/disable objects pointcloud publishing", this,
      SLOT(updatePalette()));
    m_default_pointcloud_topic = new rviz_common::properties::RosTopicProperty(
      "Input pointcloud topic", QString::fromStdString(default_pointcloud_topic), "",
      "Input for pointcloud visualization of objects detection pipeline.", this,
      SLOT(updateTopic()));
    qos_profile_points_property = new rviz_common::properties::QosProfileProperty(
      m_default_pointcloud_topic, qos_profile_points);
    // iterate over default values to create and initialize the properties.
    for (const auto & map_property_it : detail::kDefaultObjectPropertyValues) {
      const auto & class_property_values = map_property_it.second;
      const auto & color = class_property_values.color;
      // This is just a parent property to contain the necessary properties for the given class:
      m_class_group_properties.emplace_back(
        class_property_values.label.c_str(), QVariant(),
        "Groups polygon properties for the given class", this);
      auto & parent_property = m_class_group_properties.back();
      // Associate a color and opacity property for the given class and attach them to the
      // parent property of the class so they can have a drop down view from the label property:
      m_polygon_properties.emplace(
        std::piecewise_construct, std::forward_as_tuple(map_property_it.first),
        std::forward_as_tuple(
          QColor{color[0], color[1], color[2]}, class_property_values.alpha, &parent_property));
    }
    init_color_list(predicted_path_colors);
  }

  void onInitialize() override
  {
    RosTopicDisplay::RTDClass::onInitialize();
    m_marker_common.initialize(this->context_, this->scene_node_);
    QString message_type = QString::fromStdString(rosidl_generator_traits::name<MsgT>());
    this->topic_property_->setMessageType(message_type);
    this->topic_property_->setValue(m_default_topic.c_str());
    this->topic_property_->setDescription("Topic to subscribe to.");

    qos_profile_points_property->initialize([this](rclcpp::QoS profile) {
      this->qos_profile_points = profile;
      RosTopicDisplay::updateTopic();
    });

    // get weak_ptr to properly init RosTopicProperty
    rviz_ros_node = this->context_->getRosNodeAbstraction();

    m_default_pointcloud_topic->initialize(rviz_ros_node);
    QString points_message_type =
      QString::fromStdString(rosidl_generator_traits::name<sensor_msgs::msg::PointCloud2>());
    m_default_pointcloud_topic->setMessageType(points_message_type);

    // get access to rviz node to sub and to pub to topics
    rclcpp::Node::SharedPtr raw_node =
      this->context_->getRosNodeAbstraction().lock()->get_raw_node();

    tf_buffer = std::make_unique<tf2_ros::Buffer>(raw_node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    point_cloud_common->initialize(this->context_, this->scene_node_);
  }

  void load(const rviz_common::Config & config) override
  {
    RosTopicDisplay::Display::load(config);
    m_marker_common.load(config);
  }

  void update(float wall_dt, float ros_dt) override
  {
    m_marker_common.update(wall_dt, ros_dt);
    point_cloud_common->update(wall_dt, ros_dt);
  }

  void reset() override
  {
    RosTopicDisplay::reset();
    m_marker_common.clearMarkers();
    point_cloud_common->reset();
  }

  void subscribe()
  {
    if (!RosTopicDisplay::isEnabled()) {
      return;
    }

    RosTopicDisplay::subscribe();
    points_subscribe();
  }

  void updateTopic() override
  {
    unsubscribe();
    reset();
    subscribe();
    this->context_->queueRender();
  }

  void points_subscribe()
  {
    if (!m_publish_objs_pointcloud->getBool()) {
      return;
    }

    if (m_default_pointcloud_topic->isEmpty()) {
      RosTopicDisplay::setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        QString("Error subscribing: Empty topic name"));
      return;
    }

    try {
      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.event_callbacks.message_lost_callback = [&](rclcpp::QOSMessageLostInfo & info) {
        std::ostringstream sstm;
        sstm << "Some messages were lost:\n>\tNumber of new lost messages: "
             << info.total_count_change
             << " \n>\tTotal number of messages lost: " << info.total_count;
        RosTopicDisplay::setStatus(
          rviz_common::properties::StatusProperty::Warn, "Topic", QString(sstm.str().c_str()));
      };

      rclcpp::Node::SharedPtr raw_node =
        this->context_->getRosNodeAbstraction().lock()->get_raw_node();
      pointcloud_subscription = raw_node->create_subscription<sensor_msgs::msg::PointCloud2>(
        m_default_pointcloud_topic->getTopicStd(), rclcpp::SensorDataQoS(),
        std::bind(&ObjectPolygonDisplayBase::pointCloudCallback, this, std::placeholders::_1));
      RosTopicDisplay::setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
    } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
      RosTopicDisplay::setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        QString("Error subscribing: ") + e.what());
    }
  }

  void unsubscribe()
  {
    RosTopicDisplay::unsubscribe();
    pointcloud_subscription.reset();
    RCLCPP_INFO(rclcpp::get_logger("autoware_auto_perception_plugin"), "Unsubscribe called");
  }

  void clear_markers() { m_marker_common.clearMarkers(); }

  void add_marker(visualization_msgs::msg::Marker::ConstSharedPtr marker_ptr)
  {
    m_marker_common.addMessage(marker_ptr);
  }

  void add_marker(visualization_msgs::msg::MarkerArray::ConstSharedPtr markers_ptr)
  {
    m_marker_common.addMessage(markers_ptr);
  }

  void add_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    point_cloud_common->addMessage(cloud);
  }

  // transform detected object pose to target frame and return bool result
  bool transformObjects(
    const MsgT & input_msg, const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
    MsgT & output_msg)
  {
    output_msg = input_msg;

    // transform to world coordinate
    if (input_msg.header.frame_id != target_frame_id) {
      output_msg.header.frame_id = target_frame_id;
      tf2::Transform tf_target2objects_world;
      tf2::Transform tf_target2objects;
      tf2::Transform tf_objects_world2objects;
      {
        const auto ros_target2objects_world = getTransform(
          tf_buffer, input_msg.header.frame_id, target_frame_id, input_msg.header.stamp);
        if (!ros_target2objects_world) {
          return false;
        }
        tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
      }
      for (auto & object : output_msg.objects) {
        tf2::fromMsg(object.kinematics.pose_with_covariance.pose, tf_objects_world2objects);
        tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
        tf2::toMsg(tf_target2objects, object.kinematics.pose_with_covariance.pose);
      }
    }
    return true;
  }

  // get transformation from tf2
  boost::optional<geometry_msgs::msg::Transform> getTransform(
    const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
    const std::string & target_frame_id, const rclcpp::Time & time)
  {
    try {
      geometry_msgs::msg::TransformStamped self_transform_stamped;
      self_transform_stamped = tf_buffer.lookupTransform(
        target_frame_id, source_frame_id, time, rclcpp::Duration::from_seconds(0.5));
      return self_transform_stamped.transform;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("autoware_auto_perception_plugin"), ex.what());
      return boost::none;
    }
  }

  // variables for transfer detected objects information between callbacks
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

protected:
  /// \brief Convert given shape msg into a Marker
  /// \tparam ClassificationContainerT List type with ObjectClassificationMsg
  /// \param shape_msg Shape msg to be converted
  /// \param centroid Centroid position of the shape in Object.header.frame_id frame
  /// \param orientation Orientation of the shape in Object.header.frame_id frame
  /// \param labels List of ObjectClassificationMsg objects
  /// \param line_width Line thickness around the object
  /// \return Marker ptr. Id and header will have to be set by the caller
  template <typename ClassificationContainerT>
  std::optional<Marker::SharedPtr> get_shape_marker_ptr(
    const autoware_auto_perception_msgs::msg::Shape & shape_msg,
    const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
    const ClassificationContainerT & labels, const double & line_width) const
  {
    const std_msgs::msg::ColorRGBA color_rgba = get_color_rgba(labels);
    if (m_display_type_property->getOptionInt() == 0) {
      return detail::get_shape_marker_ptr(shape_msg, centroid, orientation, color_rgba, line_width);
    } else if (m_display_type_property->getOptionInt() == 1) {
      return detail::get_2d_shape_marker_ptr(
        shape_msg, centroid, orientation, color_rgba, line_width);
    } else {
      return std::nullopt;
    }
  }

  template <typename ClassificationContainerT>
  visualization_msgs::msg::Marker::SharedPtr get_2d_shape_marker_ptr(
    const autoware_auto_perception_msgs::msg::Shape & shape_msg,
    const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
    const std_msgs::msg::ColorRGBA & color_rgba, const double & line_width);

  /// \brief Convert given shape msg into a Marker to visualize label name
  /// \tparam ClassificationContainerT List type with ObjectClassificationMsg
  /// \param centroid Centroid position of the shape in Object.header.frame_id frame
  /// \param labels List of ObjectClassificationMsg objects
  /// \return Marker ptr. Id and header will have to be set by the caller
  template <typename ClassificationContainerT>
  std::optional<Marker::SharedPtr> get_label_marker_ptr(
    const geometry_msgs::msg::Point & centroid, const geometry_msgs::msg::Quaternion & orientation,
    const ClassificationContainerT & labels) const
  {
    if (m_display_label_property.getBool()) {
      const std::string label = get_best_label(labels);
      const std_msgs::msg::ColorRGBA color_rgba = get_color_rgba(labels);
      return detail::get_label_marker_ptr(centroid, orientation, label, color_rgba);
    } else {
      return std::nullopt;
    }
  }

  template <typename ClassificationContainerT>
  std::optional<Marker::SharedPtr> get_uuid_marker_ptr(
    const unique_identifier_msgs::msg::UUID & uuid, const geometry_msgs::msg::Point & centroid,
    const ClassificationContainerT & labels) const
  {
    if (m_display_uuid_property.getBool()) {
      const std_msgs::msg::ColorRGBA color_rgba = get_color_rgba(labels);
      const std::string uuid_str = uuid_to_string(uuid);
      return detail::get_uuid_marker_ptr(uuid_str, centroid, color_rgba);
    } else {
      return std::nullopt;
    }
  }

  std::optional<Marker::SharedPtr> get_pose_with_covariance_marker_ptr(
    const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance) const
  {
    if (m_display_pose_with_covariance_property.getBool()) {
      return detail::get_pose_with_covariance_marker_ptr(pose_with_covariance);
    } else {
      return std::nullopt;
    }
  }

  template <typename ClassificationContainerT>
  std::optional<Marker::SharedPtr> get_velocity_text_marker_ptr(
    const geometry_msgs::msg::Twist & twist, const geometry_msgs::msg::Point & vis_pos,
    const ClassificationContainerT & labels) const
  {
    if (m_display_velocity_text_property.getBool()) {
      const std_msgs::msg::ColorRGBA color_rgba = get_color_rgba(labels);
      return detail::get_velocity_text_marker_ptr(twist, vis_pos, color_rgba);
    } else {
      return std::nullopt;
    }
  }

  template <typename ClassificationContainerT>
  std::optional<Marker::SharedPtr> get_acceleration_text_marker_ptr(
    const geometry_msgs::msg::Accel & accel, const geometry_msgs::msg::Point & vis_pos,
    const ClassificationContainerT & labels) const
  {
    if (m_display_acceleration_text_property.getBool()) {
      const std_msgs::msg::ColorRGBA color_rgba = get_color_rgba(labels);
      return detail::get_acceleration_text_marker_ptr(accel, vis_pos, color_rgba);
    } else {
      return std::nullopt;
    }
  }

  std::optional<Marker::SharedPtr> get_twist_marker_ptr(
    const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance,
    const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance) const
  {
    if (m_display_twist_property.getBool()) {
      return detail::get_twist_marker_ptr(pose_with_covariance, twist_with_covariance);
    } else {
      return std::nullopt;
    }
  }

  std::optional<Marker::SharedPtr> get_predicted_path_marker_ptr(
    const unique_identifier_msgs::msg::UUID & uuid,
    const autoware_auto_perception_msgs::msg::Shape & shape,
    const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path) const
  {
    if (m_display_predicted_paths_property.getBool()) {
      const std::string uuid_str = uuid_to_string(uuid);
      const std_msgs::msg::ColorRGBA predicted_path_color = get_color_from_uuid(uuid_str);
      return detail::get_predicted_path_marker_ptr(
        shape, predicted_path, predicted_path_color,
        m_simple_visualize_mode_property->getOptionInt() == 1);
    } else {
      return std::nullopt;
    }
  }

  std::optional<Marker::SharedPtr> get_path_confidence_marker_ptr(
    const unique_identifier_msgs::msg::UUID & uuid,
    const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path) const
  {
    if (m_display_path_confidence_property.getBool()) {
      const std::string uuid_str = uuid_to_string(uuid);
      const std_msgs::msg::ColorRGBA path_confidence_color = get_color_from_uuid(uuid_str);
      return detail::get_path_confidence_marker_ptr(predicted_path, path_confidence_color);
    } else {
      return std::nullopt;
    }
  }

  /// \brief Get color and alpha values based on the given list of classification values
  /// \tparam ClassificationContainerT Container of ObjectClassification
  /// \param labels list of classifications
  /// \return Color and alpha for the best class in the given list. Unknown class is used in
  ///         degenerate cases
  template <typename ClassificationContainerT>
  std_msgs::msg::ColorRGBA get_color_rgba(const ClassificationContainerT & labels) const
  {
    static const std::string kLoggerName("ObjectPolygonDisplayBase");
    const auto label = detail::get_best_label(labels, kLoggerName);
    auto it = m_polygon_properties.find(label);
    if (it == m_polygon_properties.end()) {
      it = m_polygon_properties.find(ObjectClassificationMsg::UNKNOWN);
    }
    return it->second;
  }

  /// \brief Get color and alpha values based on the given list of classification values
  /// \tparam ClassificationContainerT Container of ObjectClassification
  /// \param labels list of classifications
  /// \return best label string
  template <typename ClassificationContainerT>
  std::string get_best_label(const ClassificationContainerT & labels) const
  {
    static const std::string kLoggerName("ObjectPolygonDisplayBase");
    const auto label = detail::get_best_label(labels, kLoggerName);
    auto it = detail::kDefaultObjectPropertyValues.find(label);
    if (it == detail::kDefaultObjectPropertyValues.end()) {
      it = detail::kDefaultObjectPropertyValues.find(ObjectClassificationMsg::UNKNOWN);
    }
    return (it->second).label;
  }

  std::string uuid_to_string(const unique_identifier_msgs::msg::UUID & u) const
  {
    std::stringstream ss;
    for (auto i = 0; i < 16; ++i) {
      ss << std::hex << std::setfill('0') << std::setw(2) << +u.uuid[i];
    }
    return ss.str();
  }

  std_msgs::msg::ColorRGBA AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC
  get_color_from_uuid(const std::string & uuid) const
  {
    int i = (static_cast<int>(uuid.at(0)) * 4 + static_cast<int>(uuid.at(1))) %
            static_cast<int>(predicted_path_colors.size());

    std_msgs::msg::ColorRGBA color;
    color.r = predicted_path_colors.at(i).r;
    color.g = predicted_path_colors.at(i).g;
    color.b = predicted_path_colors.at(i).b;
    return color;
  }

  void init_color_list(std::vector<std_msgs::msg::ColorRGBA> & colors) const
  {
    std_msgs::msg::ColorRGBA sample_color;
    sample_color.r = 1.0;
    sample_color.g = 0.0;
    sample_color.b = 1.0;
    colors.push_back(sample_color);  // magenta
    sample_color.r = 0.69;
    sample_color.g = 1.0;
    sample_color.b = 0.18;
    colors.push_back(sample_color);  // green yellow
    sample_color.r = 0.59;
    sample_color.g = 1.0;
    sample_color.b = 0.59;
    colors.push_back(sample_color);  // pale green
    sample_color.r = 0.5;
    sample_color.g = 1.0;
    sample_color.b = 0.0;
    colors.push_back(sample_color);  // chartreuse green
    sample_color.r = 0.12;
    sample_color.g = 0.56;
    sample_color.b = 1.0;
    colors.push_back(sample_color);  // dodger blue
    sample_color.r = 0.0;
    sample_color.g = 1.0;
    sample_color.b = 1.0;
    colors.push_back(sample_color);  // cyan
    sample_color.r = 0.54;
    sample_color.g = 0.168;
    sample_color.b = 0.886;
    colors.push_back(sample_color);  // blueviolet
    sample_color.r = 0.0;
    sample_color.g = 1.0;
    sample_color.b = 0.5;
    colors.push_back(sample_color);  // spring green
  }

  double get_line_width() { return m_line_width_property.getFloat(); }

  // helper function to get radius for kd-search
  std::optional<float> getMaxRadius(object_info & object)
  {
    if (object.shape.type == Shape::BOUNDING_BOX || object.shape.type == Shape::CYLINDER) {
      return std::hypot(object.shape.dimensions.x * 0.5f, object.shape.dimensions.y * 0.5f);
    }

    if (object.shape.type == Shape::POLYGON) {
      float max_dist = 0.0;
      for (const auto & point : object.shape.footprint.points) {
        const float dist = std::hypot(point.x, point.y);
        max_dist = std::max(max_dist, dist);
      }
      return max_dist;
    }

    return std::nullopt;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2 input_pointcloud_msg)
  {
    if (!m_publish_objs_pointcloud->getBool()) {
      return;
    }
    pointCloudBuffer_mutex.lock();
    pointCloudBuffer.push_front(input_pointcloud_msg);
    if (pointCloudBuffer.size() > 5) {
      pointCloudBuffer.pop_back();
    }
    pointCloudBuffer_mutex.unlock();
  }

  void filterPolygon(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr & in_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud, const object_info & object)
  {
    pcl::PointCloud<pcl::PointXYZRGB> filtered_cloud;
    std::vector<pcl::Vertices> vertices_array;
    pcl::Vertices vertices;

    Polygon2d poly2d = tier4_autoware_utils::toPolygon2d(object.position, object.shape);
    if (boost::geometry::is_empty(poly2d)) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < poly2d.outer().size(); ++i) {
      vertices.vertices.emplace_back(i);
      vertices_array.emplace_back(vertices);
      hull_cloud->emplace_back(
        static_cast<float>(poly2d.outer().at(i).x()), static_cast<float>(poly2d.outer().at(i).y()),
        static_cast<float>(0.0));
    }

    pcl::CropHull<pcl::PointXYZRGB> crop_hull_filter;
    crop_hull_filter.setInputCloud(in_cloud);
    crop_hull_filter.setDim(2);
    crop_hull_filter.setHullIndices(vertices_array);
    crop_hull_filter.setHullCloud(hull_cloud);
    crop_hull_filter.setCropOutside(true);

    crop_hull_filter.filter(filtered_cloud);

    const std_msgs::msg::ColorRGBA color_rgba =
      get_color_rgba(object.classification);  // need to be converted to int

    for (auto cloud_it = filtered_cloud.begin(); cloud_it != filtered_cloud.end(); ++cloud_it) {
      cloud_it->r = std::max(0, std::min(255, static_cast<int>(floor(color_rgba.r * 256.0))));
      cloud_it->g = std::max(0, std::min(255, static_cast<int>(floor(color_rgba.g * 256.0))));
      cloud_it->b = std::max(0, std::min(255, static_cast<int>(floor(color_rgba.b * 256.0))));
    }

    *out_cloud += filtered_cloud;
  }

  // Function that returns the pointcloud with the nearest timestamp from a buffer of pointclouds
  sensor_msgs::msg::PointCloud2 getNearestPointCloud(
    std::deque<sensor_msgs::msg::PointCloud2> & buffer, const rclcpp::Time & timestamp)
  {
    if (buffer.empty()) {
      // Handle the case where the buffer is empty
      throw std::runtime_error("Buffer is empty");
    }
    sensor_msgs::msg::PointCloud2 result = buffer.front();
    rclcpp::Duration diff = timestamp - rclcpp::Time(result.header.stamp);
    pointCloudBuffer_mutex.lock();
    for (size_t i = 0; i != buffer.size(); i++) {
      if (diff.nanoseconds() > (timestamp - rclcpp::Time(buffer[i].header.stamp)).nanoseconds()) {
        diff = timestamp - rclcpp::Time(buffer[i].header.stamp);
        result = buffer[i];
      }
    }
    pointCloudBuffer_mutex.unlock();

    return result;
  }

  rviz_common::properties::RosTopicProperty * m_default_pointcloud_topic;
  // Property to enable/disable objects pointcloud publishing
  rviz_common::properties::BoolProperty * m_publish_objs_pointcloud;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription;
  // plugin to visualize pointclouds
  std::unique_ptr<rviz_default_plugins::PointCloudCommon> point_cloud_common;
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node;
  std::deque<sensor_msgs::msg::PointCloud2> pointCloudBuffer;
  std::mutex pointCloudBuffer_mutex;

private:
  // All rviz plugins should have this. Should be initialized with pointer to this class
  MarkerCommon m_marker_common;
  // List is used to store the properties for classification in case we need to access them:
  std::list<rviz_common::properties::Property> m_class_group_properties;
  // Map to store class labels and its corresponding properties
  PolygonPropertyMap m_polygon_properties;
  // Property to choose type of visualization polygon
  rviz_common::properties::EnumProperty * m_display_type_property;
  // Property to choose simplicity of visualization polygon
  rviz_common::properties::EnumProperty * m_simple_visualize_mode_property;
  // Property to enable/disable label visualization
  rviz_common::properties::BoolProperty m_display_label_property;
  // Property to enable/disable uuid visualization
  rviz_common::properties::BoolProperty m_display_uuid_property;
  // Property to enable/disable pose with covariance visualization
  rviz_common::properties::BoolProperty m_display_pose_with_covariance_property;
  // Property to enable/disable velocity text visualization
  rviz_common::properties::BoolProperty m_display_velocity_text_property;
  // Property to enable/disable acceleration text visualization
  rviz_common::properties::BoolProperty m_display_acceleration_text_property;
  // Property to enable/disable twist visualization
  rviz_common::properties::BoolProperty m_display_twist_property;
  // Property to enable/disable predicted paths visualization
  rviz_common::properties::BoolProperty m_display_predicted_paths_property;
  // Property to enable/disable predicted path confidence visualization
  rviz_common::properties::BoolProperty m_display_path_confidence_property;
  // Property to decide line width of object shape
  rviz_common::properties::FloatProperty m_line_width_property;
  // Default topic name to be visualized
  std::string m_default_topic;

  std::vector<std_msgs::msg::ColorRGBA> predicted_path_colors;
  rclcpp::QoS qos_profile_points;
  rviz_common::properties::QosProfileProperty * qos_profile_points_property;
};
}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

#endif  // OBJECT_DETECTION__OBJECT_POLYGON_DISPLAY_BASE_HPP_
