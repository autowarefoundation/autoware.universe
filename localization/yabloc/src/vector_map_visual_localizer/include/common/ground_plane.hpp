#pragma once
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>

#include <std_msgs/msg/float32_multi_array.hpp>

struct GroundPlane
{
  using Float32Array = std_msgs::msg::Float32MultiArray;
  Eigen::Vector3f xyz;
  Eigen::Vector3f normal;

  GroundPlane()
  {
    xyz.setZero();
    normal = Eigen::Vector3f::UnitZ();
  }
  GroundPlane(const Float32Array & array) { set(array); }

  void set(const Float32Array & array)
  {
    if (array.data.size() != 6) exit(EXIT_FAILURE);
    for (int i = 0; i < 3; i++) {
      xyz(i) = array.data.at(i);
      normal(i) = array.data.at(3 + i);
    }
  }

  float height() const { return xyz.z(); }

  Eigen::Affine3f alineWithSlope(const Eigen::Affine3f & pose) const
  {
    Eigen::Matrix3f R = pose.rotation();
    Eigen::Vector3f t = pose.translation();
    {
      Eigen::Vector3f rz = this->normal;
      Eigen::Vector3f azimuth = R * Eigen::Vector3f::UnitX();
      Eigen::Vector3f ry = (rz.cross(azimuth)).normalized();
      Eigen::Vector3f rx = ry.cross(rz);
      R.col(0) = rx;
      R.col(1) = ry;
      R.col(2) = rz;
    }
    return Eigen::Translation3f(t) * R;
  }

  Float32Array msg() const
  {
    Float32Array array;
    for (int i = 0; i < 3; i++) array.data.push_back(xyz(i));
    for (int i = 0; i < 3; i++) array.data.push_back(normal(i));
    return array;
  }
};
