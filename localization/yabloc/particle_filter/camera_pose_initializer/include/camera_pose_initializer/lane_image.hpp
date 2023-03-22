#pragma once
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>

namespace pcdless
{
class LaneImage
{
public:
  using Pose = geometry_msgs::msg::Pose;
  using SharedPtr = std::shared_ptr<LaneImage>;
  LaneImage(lanelet::LaneletMapPtr map);

  cv::Mat get_image(const Pose & pose);

private:
  lanelet::LaneletMapPtr map_;
};
}  // namespace pcdless