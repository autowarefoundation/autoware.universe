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

#include <rclcpp/rclcpp.hpp>
#include <autoware_perception_msgs/DynamicObjectArray.h>

class EmptyObjectsPublisher
{
public:
  EmptyObjectsPublisher() : nh_(""), pnh_("~")
  {
    empty_objects_pub_ = pnh_.advertise<autoware_perception_msgs::DynamicObjectArray>("output/objects", 1);
    timer_ = nh_.createTimer(ros::Duration(0.1), &EmptyObjectsPublisher::timerCallback, this);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher empty_objects_pub_;
  ros::Timer timer_;

  void timerCallback(const ros::TimerEvent & e)
  {
    autoware_perception_msgs::DynamicObjectArray empty_objects;
    empty_objects.header.frame_id = "/map";
    empty_objects.header.stamp = ros::Time::now();
    empty_objects_pub_.publish(empty_objects);
  }

};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "empty_objects_publisher");
  EmptyObjectsPublisher empty_objects_publisher;
  ros::spin();
  return 0;
};
