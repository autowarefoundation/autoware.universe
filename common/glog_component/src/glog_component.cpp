#include "glog_component/glog_component.hpp"

GlogComponent::GlogComponent(const rclcpp::NodeOptions & node_options)
: Node("glog_component", node_options)
{
  google::InitGoogleLogging("glog_component");
  google::InstallFailureSignalHandler();
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(GlogComponent)
