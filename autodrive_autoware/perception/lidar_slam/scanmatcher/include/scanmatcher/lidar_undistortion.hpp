// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#ifndef LIDAR_UNDISTORTION_HPP_
#define LIDAR_UNDISTORTION_HPP_

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

class LidarUndistortion
{
public:
  LidarUndistortion() {}

  // Ref:LeGO-LOAM(BSD-3 LICENSE)
  // https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/blob/master/LeGO-LOAM/src/featureAssociation.cpp#L431-L459
  void getImu(
    Eigen::Vector3f angular_velo, Eigen::Vector3f acc, const Eigen::Quaternionf quat,
    const double imu_time /*[sec]*/)
  {
    float roll, pitch, yaw;
    Eigen::Affine3f affine(quat);
    pcl::getEulerAngles(affine, roll, pitch, yaw);

    imu_ptr_last_ = (imu_ptr_last_ + 1) % imu_que_length_;

    if ((imu_ptr_last_ + 1) % imu_que_length_ == imu_ptr_front_) {
      imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_que_length_;
    }

    imu_time_[imu_ptr_last_] = imu_time;
    imu_roll_[imu_ptr_last_] = roll;
    imu_pitch_[imu_ptr_last_] = pitch;
    imu_yaw_[imu_ptr_last_] = yaw;
    imu_acc_x_[imu_ptr_last_] = acc.x();
    imu_acc_y_[imu_ptr_last_] = acc.y();
    imu_acc_z_[imu_ptr_last_] = acc.z();
    imu_angular_velo_x_[imu_ptr_last_] = angular_velo.x();
    imu_angular_velo_y_[imu_ptr_last_] = angular_velo.y();
    imu_angular_velo_z_[imu_ptr_last_] = angular_velo.z();

    Eigen::Matrix3f rot = quat.toRotationMatrix();
    acc = rot * acc;
    // angular_velo = rot * angular_velo;

    int imu_ptr_back = (imu_ptr_last_ - 1 + imu_que_length_) % imu_que_length_;
    double time_diff = imu_time_[imu_ptr_last_] - imu_time_[imu_ptr_back];
    if (time_diff < scan_period_) {
      imu_shift_x_[imu_ptr_last_] =
        imu_shift_x_[imu_ptr_back] + imu_velo_x_[imu_ptr_back] * time_diff + acc(0) * time_diff *
        time_diff * 0.5;
      imu_shift_y_[imu_ptr_last_] =
        imu_shift_y_[imu_ptr_back] + imu_velo_y_[imu_ptr_back] * time_diff + acc(1) * time_diff *
        time_diff * 0.5;
      imu_shift_z_[imu_ptr_last_] =
        imu_shift_z_[imu_ptr_back] + imu_velo_z_[imu_ptr_back] * time_diff + acc(2) * time_diff *
        time_diff * 0.5;

      imu_velo_x_[imu_ptr_last_] = imu_velo_x_[imu_ptr_back] + acc(0) * time_diff;
      imu_velo_y_[imu_ptr_last_] = imu_velo_y_[imu_ptr_back] + acc(1) * time_diff;
      imu_velo_z_[imu_ptr_last_] = imu_velo_z_[imu_ptr_back] + acc(2) * time_diff;

      imu_angular_rot_x_[imu_ptr_last_] = imu_angular_rot_x_[imu_ptr_back] + angular_velo(0) *
        time_diff;
      imu_angular_rot_y_[imu_ptr_last_] = imu_angular_rot_y_[imu_ptr_back] + angular_velo(1) *
        time_diff;
      imu_angular_rot_z_[imu_ptr_last_] = imu_angular_rot_z_[imu_ptr_back] + angular_velo(2) *
        time_diff;
    }
  }

  // Ref:LeGO-LOAM(BSD-3 LICENSE)
  // https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/blob/master/LeGO-LOAM/src/featureAssociation.cpp#L491-L619
  void adjustDistortion(
    pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    const double scan_time /*[sec]*/)
  {
    bool half_passed = false;
    int cloud_size = cloud->points.size();

    float start_ori = -std::atan2(cloud->points[0].y, cloud->points[0].x);
    float end_ori = -std::atan2(cloud->points[cloud_size - 1].y, cloud->points[cloud_size - 1].x);
    if (end_ori - start_ori > 3 * M_PI) {
      end_ori -= 2 * M_PI;
    } else if (end_ori - start_ori < M_PI) {
      end_ori += 2 * M_PI;
    }
    float ori_diff = end_ori - start_ori;

    Eigen::Vector3f rpy_start, shift_start, velo_start, rpy_cur, shift_cur, velo_cur;
    Eigen::Vector3f shift_from_start;
    Eigen::Matrix3f r_s_i, r_c;
    Eigen::Vector3f adjusted_p;
    float ori_h;
    for (int i = 0; i < cloud_size; ++i) {
      pcl::PointXYZI & p = cloud->points[i];
      ori_h = -std::atan2(p.y, p.x);
      if (!half_passed) {
        if (ori_h < start_ori - M_PI * 0.5) {
          ori_h += 2 * M_PI;
        } else if (ori_h > start_ori + M_PI * 1.5) {
          ori_h -= 2 * M_PI;
        }

        if (ori_h - start_ori > M_PI) {
          half_passed = true;
        }
      } else {
        ori_h += 2 * M_PI;
        if (ori_h < end_ori - 1.5 * M_PI) {
          ori_h += 2 * M_PI;
        } else if (ori_h > end_ori + 0.5 * M_PI) {
          ori_h -= 2 * M_PI;
        }
      }

      float rel_time = (ori_h - start_ori) / ori_diff * scan_period_;

      if (imu_ptr_last_ > 0) {
        imu_ptr_front_ = imu_ptr_last_iter_;
        while (imu_ptr_front_ != imu_ptr_last_) {
          if (scan_time + rel_time < imu_time_[imu_ptr_front_]) {
            break;
          }
          imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_que_length_;
        }

        if (std::abs(scan_time + rel_time - imu_time_[imu_ptr_front_]) > scan_period_) {
          continue;
        }

        if (scan_time + rel_time > imu_time_[imu_ptr_front_]) {
          rpy_cur(0) = imu_roll_[imu_ptr_front_];
          rpy_cur(1) = imu_pitch_[imu_ptr_front_];
          rpy_cur(2) = imu_yaw_[imu_ptr_front_];
          shift_cur(0) = imu_shift_x_[imu_ptr_front_];
          shift_cur(1) = imu_shift_y_[imu_ptr_front_];
          shift_cur(2) = imu_shift_z_[imu_ptr_front_];
          velo_cur(0) = imu_velo_x_[imu_ptr_front_];
          velo_cur(1) = imu_velo_y_[imu_ptr_front_];
          velo_cur(2) = imu_velo_z_[imu_ptr_front_];
        } else {
          int imu_ptr_back = (imu_ptr_front_ - 1 + imu_que_length_) % imu_que_length_;
          float ratio_front = (scan_time + rel_time - imu_time_[imu_ptr_back]) /
            (imu_time_[imu_ptr_front_] - imu_time_[imu_ptr_back]);
          float ratio_back = 1.0 - ratio_front;
          rpy_cur(0) = imu_roll_[imu_ptr_front_] * ratio_front + imu_roll_[imu_ptr_back] *
            ratio_back;
          rpy_cur(1) = imu_pitch_[imu_ptr_front_] * ratio_front + imu_pitch_[imu_ptr_back] *
            ratio_back;
          rpy_cur(2) = imu_yaw_[imu_ptr_front_] * ratio_front + imu_yaw_[imu_ptr_back] * ratio_back;
          shift_cur(0) = imu_shift_x_[imu_ptr_front_] * ratio_front + imu_shift_x_[imu_ptr_back] *
            ratio_back;
          shift_cur(1) = imu_shift_y_[imu_ptr_front_] * ratio_front + imu_shift_y_[imu_ptr_back] *
            ratio_back;
          shift_cur(2) = imu_shift_z_[imu_ptr_front_] * ratio_front + imu_shift_z_[imu_ptr_back] *
            ratio_back;
          velo_cur(0) = imu_velo_x_[imu_ptr_front_] * ratio_front + imu_velo_x_[imu_ptr_back] *
            ratio_back;
          velo_cur(1) = imu_velo_y_[imu_ptr_front_] * ratio_front + imu_velo_y_[imu_ptr_back] *
            ratio_back;
          velo_cur(2) = imu_velo_z_[imu_ptr_front_] * ratio_front + imu_velo_z_[imu_ptr_back] *
            ratio_back;
        }

        r_c =
          (Eigen::AngleAxisf(
            rpy_cur(2),
            Eigen::Vector3f::UnitZ()) *
          Eigen::AngleAxisf(
            rpy_cur(1),
            Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(rpy_cur(0), Eigen::Vector3f::UnitX())).toRotationMatrix();

        if (i == 0) {
          rpy_start = rpy_cur;
          shift_start = shift_cur;
          velo_start = velo_cur;
          r_s_i = r_c.inverse();
        } else {
          shift_from_start = shift_cur - shift_start - velo_start * rel_time;
          adjusted_p = r_s_i * (r_c * Eigen::Vector3f(p.x, p.y, p.z) + shift_from_start);
          p.x = adjusted_p.x();
          p.y = adjusted_p.y();
          p.z = adjusted_p.z();
        }
      }
      imu_ptr_last_iter_ = imu_ptr_front_;
    }
  }


  void setScanPeriod(const double scan_period /*[sec]*/)
  {
    scan_period_ = scan_period;
  }

private:
  double scan_period_{0.1};
  static const int imu_que_length_{200};
  int imu_ptr_front_{0}, imu_ptr_last_{-1}, imu_ptr_last_iter_{0};

  std::array<double, imu_que_length_> imu_time_;
  std::array<float, imu_que_length_> imu_roll_;
  std::array<float, imu_que_length_> imu_pitch_;
  std::array<float, imu_que_length_> imu_yaw_;

  std::array<float, imu_que_length_> imu_acc_x_;
  std::array<float, imu_que_length_> imu_acc_y_;
  std::array<float, imu_que_length_> imu_acc_z_;
  std::array<float, imu_que_length_> imu_velo_x_;
  std::array<float, imu_que_length_> imu_velo_y_;
  std::array<float, imu_que_length_> imu_velo_z_;
  std::array<float, imu_que_length_> imu_shift_x_;
  std::array<float, imu_que_length_> imu_shift_y_;
  std::array<float, imu_que_length_> imu_shift_z_;

  std::array<float, imu_que_length_> imu_angular_velo_x_;
  std::array<float, imu_que_length_> imu_angular_velo_y_;
  std::array<float, imu_que_length_> imu_angular_velo_z_;
  std::array<float, imu_que_length_> imu_angular_rot_x_;
  std::array<float, imu_que_length_> imu_angular_rot_y_;
  std::array<float, imu_que_length_> imu_angular_rot_z_;
};

#endif  // LIDAR_UNDISTORTION_HPP_
