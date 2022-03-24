// Copyright 2022 Tier IV, Inc.
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

#ifndef IMAGE_PROJECTION_BASED_FUSION__UTILS__UTILS_HPP_
#define IMAGE_PROJECTION_BASED_FUSION__UTILS__UTILS_HPP_

// #define EIGEN_MPL2_ONLY

// #include <Eigen/Core>
// #include <Eigen/Geometry>

// #include <boost/optional.hpp>

// #include <tf2_eigen/tf2_eigen.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// #include <tf2_ros/buffer.h>

// #include <memory>
// #include <vector>

namespace image_projection_based_fusion
{

using tier4_perception_msgs::msg::DetectedObjectWithFeature;

// void extractRois(
//   const std::vector<DetectedObjectWithFeature> & feature_objects,
//   std::vector<sensor_msgs::msg::RegionOfInterest> & rois)
// {
//   rois.reserve(feature_objects.size());
//   for (const auto & obj : feature_objects) {
//     rois.push_back(obj.feature.roi);
//   }
// }

// boost::optional<geometry_msgs::msg::Transform> getTransform(
//   const tf2_ros::Buffer & tf_buffer, const std::string & target_frame_id,
//   const std::string & source_frame_id, const rclcpp::Time & stamp)
// {
//   try {
//     geometry_msgs::msg::TransformStamped transform_stamped;
//     transform_stamped = tf_buffer.lookupTransform(
//       target_frame_id, source_frame_id, stamp, rclcpp::Duration::from_seconds(0.5));
//     return transform_stamped.transform;
//   } catch (tf2::TransformException & ex) {
//     RCLCPP_WARN_STREAM(rclcpp::get_logger("image_projection_based_fusion"), ex.what());
//     return boost::none;
//   }
// }

// Eigen::Affine3d transformToEigen(const geometry_msgs::msg::Transform & t)
// {
//   Eigen::Affine3d a;
//   a.matrix() = tf2::transformToEigen(t).matrix();
//   return a;
// }

}  // namespace image_projection_based_fusion

#endif  // IMAGE_PROJECTION_BASED_FUSION__UTILS__UTILS_HPP_
