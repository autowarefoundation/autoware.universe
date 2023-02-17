// Copyright 2020 Tier IV, Inc.
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
#ifndef IMAGE_RECTIFIER__IMAGE_RECTIFIER_HPP_
#define IMAGE_RECTIFIER__IMAGE_RECTIFIER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <memory>
#include <string>
#include <utility>

namespace image_preprocessor
{
class ImageRectifier : public rclcpp::Node
{
public:
  explicit ImageRectifier(const rclcpp::NodeOptions & node_options);

private:
    image_transport::CameraSubscriber sub_camera_;

    int queue_size_;
    int interpolation;
    image_transport::Publisher pub_rect_;

    // Processing state (note: only safe because we're using single-threaded NodeHandle!)
    image_geometry::PinholeCameraModel model_;



    void imageCb(
            const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);
};

}  // namespace image_preprocessor

#endif  // IMAGE_RECTIFIER__IMAGE_RECTIFIER_HPP_
