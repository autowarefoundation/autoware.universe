// Copyright 2015-2019 Autoware Foundation
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

#include "kitti_evaluator/kitti_evaluator_core.hpp"
#include "kitti_evaluator/geodetic.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <eigen3/Eigen/Geometry> 
#include <tf2/utils.h>
#include <deque>

// double rotationError(Eigen::Matrix4d &pose_error) {
//   double a = pose_error(0, 0);
//   double b = pose_error(1, 1);
//   double c = pose_error(2, 2);
//   double d = 0.5*(a+b+c-1.0);
//   return acos(max(min(d,1.0),-1.0));
// }

// double translationError(Eigen::Matrix4d &pose_error) {
//   double dx = pose_error(0, 3);
//   double dy = pose_error(1, 3);
//   double dz = pose_error(2, 3);
//   return sqrt(dx*dx+dy*dy+dz*dz);
// }

void msgToMatrix(geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_ptr, Eigen::Affine3d & out)
{
    // out.setIdentity();
    // out(0,3) = pose_ptr->pose.position.x;
    // out(1,3) = pose_ptr->pose.position.y;
    // out(2,3) = pose_ptr->pose.position.z;
    Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(pose_ptr->pose.position.x, pose_ptr->pose.position.y, pose_ptr->pose.position.z)));
    Eigen::Quaternion q(pose_ptr->pose.orientation.w, pose_ptr->pose.orientation.x, pose_ptr->pose.orientation.y, pose_ptr->pose.orientation.z);
    // out.block<3,3>(0,0) = q.toRotationMatrix();
    out = t * q;
}

KittiEvaluator::KittiEvaluator()
: rclcpp::Node("kitti_evaluator")
{
    // nav_sat_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    //     "/gps_fix", rclcpp::QoS{100},
    //     std::bind(&KittiEvaluator::callbackNavSatFix, this, std::placeholders::_1));
    vehicle_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/localization/pose_twist_fusion_filter/pose", rclcpp::QoS{100},
        std::bind(&KittiEvaluator::callbackVehicleOdometry, this, std::placeholders::_1));
    groud_truth_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/groud_truth", rclcpp::QoS{100},
        std::bind(&KittiEvaluator::callbackGroundTruthOdometry, this, std::placeholders::_1));
    init = false;
    // last_vehicle_pose.setIdentity();
    // last_groud_truth_pose.setIdentity();
}

KittiEvaluator::~KittiEvaluator() {}

void KittiEvaluator::initialize(double lat, double lon, double alt)
{
    init = true;
    scale = cos(lat * geodetic::pi / 180.0);
    std::vector<double> t = geodetic::se3_translation(lat, lon, alt, scale);
    x0 = t[0];
    y0 = t[1];
    z0 = t[2];
}

// void KittiEvaluator::callbackNavSatFix(sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_msg_ptr)
// {
//     RCLCPP_INFO(get_logger(), "Get GPS");
//     if(!init)
//     {
//         initialize(nav_sat_fix_msg_ptr->latitude, nav_sat_fix_msg_ptr->longitude, nav_sat_fix_msg_ptr->altitude);
//     }
//     auto curr_time = nav_sat_fix_msg_ptr->header.stamp;
//     while(!odom_queue.empty() && rclcpp::Time(odom_queue.front()->header.stamp) < rclcpp::Time(curr_time) - rclcpp::Duration::from_seconds(delta))
//     {
//         odom_queue.pop_front();
//     }
//     if(!odom_queue.empty())
//     {
//         // calculateError(odom_queue.front(), nav_sat_fix_msg_ptr);
//     }
//     else
//     {
//         RCLCPP_INFO(get_logger(), "Cannot Find Valid Pose!");
//     }
// }

void KittiEvaluator::callbackGroundTruthOdometry(geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_ground_truth_msg_ptr)
{
    RCLCPP_INFO(get_logger(), "Get Ground Truth");

    if(!vehicle_pose_queue.empty())
    {
        RCLCPP_INFO(get_logger(), "We have ekf pose! Size : %ld", vehicle_pose_queue.size());
    }

    auto curr_time = pose_ground_truth_msg_ptr->header.stamp;
    while(!vehicle_pose_queue.empty() && rclcpp::Time(vehicle_pose_queue.front()->header.stamp) < rclcpp::Time(curr_time) - rclcpp::Duration::from_seconds(delta))
    {
        // last_vehicle_pose = vehicle_pose_queue.front();
        vehicle_pose_queue.pop_front();
    }
    if(!vehicle_pose_queue.empty())
    {
        // Eigen::Affine3d transform_groud_truth;
        // tf2::fromMsg(*pose_ground_truth_msg_ptr, transform_groud_truth);
        // Eigen::Affine3d transform_vehicle;
        auto pose_vehicle = vehicle_pose_queue.front();
        vehicle_pose_queue.pop_front();
        // tf2::fromMsg(*pose_vehicle, transform_vehicle);
        calculateError(pose_vehicle, pose_ground_truth_msg_ptr);
        last_vehicle_pose = pose_vehicle;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Cannot Find Valid Pose!");
    }

    last_groud_truth_pose = pose_ground_truth_msg_ptr;
}

void KittiEvaluator::callbackVehicleOdometry(geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg_ptr)
{
    RCLCPP_INFO(get_logger(), "Get Vehicle Pose");
    vehicle_pose_queue.push_back(pose_msg_ptr);
}

void KittiEvaluator::calculateError(
    geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_vehicle,
    geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_groud_truth)
{
    Eigen::Affine3d curr_transform_groud_truth;
    msgToMatrix(pose_groud_truth, curr_transform_groud_truth);
    Eigen::Affine3d curr_transform_vehicle;
    msgToMatrix(pose_vehicle, curr_transform_vehicle);
    Eigen::Affine3d last_transform_groud_truth;
    if(last_groud_truth_pose)
    {
        msgToMatrix(last_groud_truth_pose, last_transform_groud_truth);
    }
    else
    {
        last_transform_groud_truth.setIdentity();
    }
    Eigen::Affine3d last_transform_vehicle;
    if(last_vehicle_pose)
    {
        msgToMatrix(last_vehicle_pose, last_transform_vehicle);
    }
    else
    {
        last_transform_vehicle.setIdentity();
    }
    // RCLCPP_INFO(get_logger(), "%lf %lf %lf %lf", curr_transform_groud_truth(0,0), curr_transform_groud_truth(0,1), curr_transform_groud_truth(0,2), curr_transform_groud_truth(0,3));


    Eigen::Affine3d delta_ground_truth = last_transform_groud_truth.inverse() * curr_transform_groud_truth;
    Eigen::Affine3d delta_vehicle = last_transform_vehicle.inverse() * curr_transform_vehicle;
    Eigen::Affine3d transform_error = delta_vehicle.inverse() * delta_ground_truth;

    Eigen::Matrix4d trans = transform_error.matrix();

    RCLCPP_INFO(get_logger(), "Translation Error: x = %lf, y = %lf, z = %lf ", trans(0,3), trans(1,3), trans(2,3));

    Eigen::Vector3d euler = trans.block<3, 3>(0, 0).eulerAngles(0,1,2);
    RCLCPP_INFO(get_logger(), "Rotation Error: roll = %lf, pitch = %lf, yaw = %lf ", euler(0), euler(1), euler(2));
}

