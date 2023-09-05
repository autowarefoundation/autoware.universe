#ifndef TOOLS__CYCLE_POSE_HPP_
#define TOOLS__CYCLE_POSE_HPP_

#include "interactive_object.hpp"

namespace rviz_plugins
{

using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::Shape;
using dummy_perception_publisher::msg::Object;

class cycleInitialPoseTool : public InteractiveObjectTool
{
public:
  cycleInitialPoseTool();
  void onInitialize() override;
  [[nodiscard]] Object createObjectMsg() const override;
};

}  // namespace rviz_plugins

#endif  
