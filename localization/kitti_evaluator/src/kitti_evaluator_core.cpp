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
#include "kitti_evaluator/interpolation.hpp"

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
    pose_error_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/localization_error", 1);
    init = false;
    last_vehicle_trans.setIdentity();
    last_groud_truth_trans.setIdentity();
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
    // RCLCPP_INFO(get_logger(), "Get Ground Truth");

    if(!vehicle_pose_queue.empty())
    {
        RCLCPP_INFO(get_logger(), "We have ekf pose! Size : %ld", vehicle_pose_queue.size());
    }

    Eigen::Affine3d ground_truth_trans;
    msgToMatrix(pose_ground_truth_msg_ptr, ground_truth_trans);

    rclcpp::Time curr_time(pose_ground_truth_msg_ptr->header.stamp);
    geometry_msgs::msg::PoseStamped::ConstSharedPtr prev_pose;
    while(!vehicle_pose_queue.empty() && rclcpp::Time(vehicle_pose_queue.front()->header.stamp) < curr_time - rclcpp::Duration::from_seconds(delta))
    {
        prev_pose = vehicle_pose_queue.front();
        vehicle_pose_queue.pop_front();
    }
    if(!vehicle_pose_queue.empty())
    {
        auto post_pose = vehicle_pose_queue.front();
        vehicle_pose_queue.pop_front();

        Eigen::Affine3d prev_trans;
        rclcpp::Time prev_time = get_clock()->now();
        prev_trans.setIdentity();
        if(prev_pose)
        {
            prev_time = rclcpp::Time(prev_pose->header.stamp);
            msgToMatrix(prev_pose, prev_trans);
        }
        Eigen::Affine3d post_trans;
        rclcpp::Time post_time(post_pose->header.stamp);
        msgToMatrix(post_pose, post_trans);

        rclcpp::Time fake_time = curr_time - rclcpp::Duration::from_seconds(delta);
        double t = interpolation::getTimeCoeffients(fake_time, prev_time, post_time);
        if(t>1 || t<0)
        {
            RCLCPP_INFO(get_logger(), "????????????");
        }
        RCLCPP_INFO(get_logger(), "Interpolate Coefficient : %lf", t);
        Eigen::Affine3d vehicle_trans;
        interpolation::interpolateTransform(t, prev_trans, post_trans, vehicle_trans);

        // RCLCPP_INFO(get_logger(), "%lf %lf %lf %lf", vehicle_trans(0,0), vehicle_trans(0,1), vehicle_trans(0,2), vehicle_trans(0,3));
        // RCLCPP_INFO(get_logger(), "%lf %lf %lf %lf", vehicle_trans(1,0), vehicle_trans(1,1), vehicle_trans(1,2), vehicle_trans(1,3));
        // RCLCPP_INFO(get_logger(), "%lf %lf %lf %lf", vehicle_trans(2,0), vehicle_trans(2,1), vehicle_trans(2,2), vehicle_trans(2,3));
        // RCLCPP_INFO(get_logger(), "%lf %lf %lf %lf", vehicle_trans(3,0), vehicle_trans(3,1), vehicle_trans(3,2), vehicle_trans(3,3));

        calculateError(vehicle_trans, ground_truth_trans);
        // last_vehicle_pose = pose_vehicle;
        last_vehicle_trans = vehicle_trans;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Cannot Find Valid Pose!");
    }

    // last_groud_truth_pose = pose_ground_truth_msg_ptr;
    last_groud_truth_trans = ground_truth_trans;
}

void KittiEvaluator::callbackVehicleOdometry(geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg_ptr)
{
    // RCLCPP_INFO(get_logger(), "Get Vehicle Pose");
    vehicle_pose_queue.push_back(pose_msg_ptr);
}

void KittiEvaluator::calculateError(Eigen::Affine3d & vehicle_trans, Eigen::Affine3d & groud_truth_trans)
{
    Eigen::Affine3d delta_ground_truth = last_groud_truth_trans.inverse() * groud_truth_trans;
    Eigen::Affine3d delta_vehicle = last_vehicle_trans.inverse() * vehicle_trans;
    Eigen::Affine3d transform_error = delta_vehicle.inverse() * delta_ground_truth;

    Eigen::Matrix4d trans = transform_error.matrix();

    RCLCPP_INFO(get_logger(), "Translation Error: x = %lf, y = %lf, z = %lf ", trans(0,3), trans(1,3), trans(2,3));

    Eigen::Vector3d euler = trans.block<3, 3>(0, 0).eulerAngles(0,1,2);
    RCLCPP_INFO(get_logger(), "Rotation Error: roll = %lf, pitch = %lf, yaw = %lf ", euler(0), euler(1), euler(2));
}
