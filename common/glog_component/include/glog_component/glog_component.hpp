#ifndef GLOG_COMPONENT__GLOG_COMPONENT_HPP_
#define GLOG_COMPONENT__GLOG_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <glog/logging.h>

class GlogComponent : public rclcpp::Node
{
public:
  explicit GlogComponent(const rclcpp::NodeOptions & node_options);
};

#endif  // GLOG_COMPONENT__GLOG_COMPONENT_HPP_
