#include "cycle.hpp"

#include "util.hpp"

#include <rviz_common/display_context.hpp>

#include <algorithm>
#include <random>
#include <string>

namespace rviz_plugins
{

cycleInitialPoseTool::cycleInitialPoseTool()
{
  // short cut below cyclist
  shortcut_key_ = 'c';

  enable_interactive_property_ = new rviz_common::properties::BoolProperty(
    "Interactive", false, "Enable/Disable interactive action by manipulating mouse.",
    getPropertyContainer());
  property_frame_ = new rviz_common::properties::TfFrameProperty(
    "Target Frame", rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING,
    "The TF frame where the point cloud is output.", getPropertyContainer(), nullptr, true);
  topic_property_ = new rviz_common::properties::StringProperty(
    "Pose Topic", "/simulation/dummy_perception_publisher/object_info",
    "The topic on which to publish dummy object info.", getPropertyContainer(), SLOT(updateTopic()),
    this);
  std_dev_x_ = new rviz_common::properties::FloatProperty(
    "X std deviation", 0.03, "X standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_y_ = new rviz_common::properties::FloatProperty(
    "Y std deviation", 0.03, "Y standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_z_ = new rviz_common::properties::FloatProperty(
    "Z std deviation", 0.03, "Z standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_theta_ = new rviz_common::properties::FloatProperty(
    "Theta std deviation", 5.0 * M_PI / 180.0, "Theta standard deviation for initial pose [rad]",
    getPropertyContainer());
  length_ = new rviz_common::properties::FloatProperty(
    "L vehicle length", 1.5, "L length for vehicle [m]", getPropertyContainer());
  width_ = new rviz_common::properties::FloatProperty(
    "W vehicle width", 0.5, "W width for vehicle [m]", getPropertyContainer());
  height_ = new rviz_common::properties::FloatProperty(
    "H vehicle height", 1.0, "H height for vehicle [m]", getPropertyContainer());
  position_z_ = new rviz_common::properties::FloatProperty(
    "Z position", 0.0, "Z position for initial pose [m]", getPropertyContainer());
  velocity_ = new rviz_common::properties::FloatProperty(
    "Velocity", 0.0, "velocity [m/s]", getPropertyContainer());
  accel_ = new rviz_common::properties::FloatProperty(
    "Acceleration", 0.0, "acceleration [m/s^2]", getPropertyContainer());
  max_velocity_ = new rviz_common::properties::FloatProperty(
    "Max velocity", 33.3, "Max velocity [m/s]", getPropertyContainer());
  min_velocity_ = new rviz_common::properties::FloatProperty(
    "Min velocity", -33.3, "Min velocity [m/s]", getPropertyContainer());
  
  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_z_->setMin(0);
  std_dev_theta_->setMin(0);
  width_->setMin(0);
  length_->setMin(0);
  height_->setMin(0);
  
}

void cycleInitialPoseTool::onInitialize()
{
  rviz_plugins::InteractiveObjectTool::onInitialize();
  setName("2D Dummy Bicycle");
  updateTopic();
}

Object cycleInitialPoseTool::createObjectMsg() const
{
  Object object{};
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  // header
  object.header.frame_id = fixed_frame;
  object.header.stamp = clock_->now();

  // semantic
  object.classification.label = label_->getOptionInt();
  object.classification.probability = 1.0;

  // shape
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = length_->getFloat();
  object.shape.dimensions.y = width_->getFloat();
  object.shape.dimensions.z = height_->getFloat();

  // initial state
  object.initial_state.pose_covariance.pose.position.z = position_z_->getFloat();
  object.initial_state.pose_covariance.covariance[0] =
    std_dev_x_->getFloat() * std_dev_x_->getFloat();
  object.initial_state.pose_covariance.covariance[7] =
    std_dev_y_->getFloat() * std_dev_y_->getFloat();
  object.initial_state.pose_covariance.covariance[14] =
    std_dev_z_->getFloat() * std_dev_z_->getFloat();
  object.initial_state.pose_covariance.covariance[35] =
    std_dev_theta_->getFloat() * std_dev_theta_->getFloat();

  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(object.id.uuid.begin(), object.id.uuid.end(), bit_eng);

  return object;
}

}  // end namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::cycleInitialPoseTool, rviz_common::Tool)
