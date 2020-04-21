/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>

// STL
#include <chrono>
#include <memory>
#include <string>

// local
#include "TrtNet.h"
#include "data_reader.h"

class TensorrtYoloROS
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_image_;
  ros::Publisher pub_objects_;
  ros::Publisher pub_image_;

  std::unique_ptr<Tn::trtNet> net_ptr_;

  void imageCallback(const sensor_msgs::Image::ConstPtr & in_image);
  std::vector<float> prepareImage(cv::Mat & in_img);
  std::vector<Tn::Bbox> postProcessImg(
    std::vector<Yolo::Detection> & detections, const int classes, cv::Mat & img,
    autoware_perception_msgs::DynamicObjectWithFeatureArray & out_objects);
  void doNms(std::vector<Yolo::Detection> & detections, int classes, float nmsThresh);
  /* data */
public:
  TensorrtYoloROS(/* args */);
  ~TensorrtYoloROS();

  void createROSPubSub();
};
