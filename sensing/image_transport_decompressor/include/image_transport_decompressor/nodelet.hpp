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

#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <limits>
#include <vector>

namespace image_preprocessor
{
class ImageTransportDecompressorNodelet : public nodelet::Nodelet
{
private:
  virtual void onInit() override;

  void compressedImageCallback(
    const sensor_msgs::CompressedImageConstPtr & input_compressed_image_msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber compressed_image_sub_;
  ros::Publisher raw_image_pub_;
  std::string enc_;
};

}  // namespace image_preprocessor
