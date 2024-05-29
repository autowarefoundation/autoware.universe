#ifndef GNSS_POSER__MSG_HPP_
#define GNSS_POSER__MSG_HPP_

#include <iostream>
#include <vector>

struct Header{
    // 使用与 Python 类中相同的数据类型和字段名称
    std::string frame_id;
    std::string stamp;
    uint32_t seq;
    int sec;
    int nanosec;
};
struct NavSatStatus{
  static const char STATUS_NO_FIX = -1;
  static const char STATUS_FIX = 0;
  static const char STATUS_SBAS_FIX = 1;
  static const char STATUS_GBAS_FIX = 2;
  static const uint SERVICE_GPS = 1;
  static const uint SERVICE_GLONASS = 2;
  static const uint SERVICE_COMPASS = 4;
  static const uint SERVICE_GALILEO = 8;
  // char STATUS_NO_FIX=-1;
  // char STATUS_FIX=0;
  // char STATUS_SBAS_FIX=1;
  // char STATUS_GBAS_FIX=2;
  // uint SERVICE_GPS=1;
  // uint SERVICE_GLONASS=2;
  // uint SERVICE_COMPASS=4;
  // uint SERVICE_GALILEO=8;
  // 使用与 Python 类中相同的数据类型和字段名称
  int status;
  int service;
};


// 定义一个结构体来表示 DoraNavSatFix 对象的数据结构
struct DoraNavSatFix {
    // 使用与 Python 类中相同的数据类型和字段名称
    Header header;
    NavSatStatus status;
    double latitude;
    double longitude;
    double altitude;
    std::vector<double> position_covariance;
    int position_covariance_type;
};


struct Point final {
  double x;
  double y;
  double z;

  bool operator==(Point const &) const noexcept;
  bool operator!=(Point const &) const noexcept;
  using IsRelocatable = ::std::true_type;
};

namespace geometry_msgs
{
struct Quaternion final {
  double x;
  double y;
  double z;
  double w;

  bool operator==(Quaternion const &) const noexcept;
  bool operator!=(Quaternion const &) const noexcept;
  using IsRelocatable = ::std::true_type;
};
}

struct GnssInsOrientation final {
  geometry_msgs::Quaternion orientation;
  float rmse_rotation_x;
  float rmse_rotation_y;
  float rmse_rotation_z;
};

struct GnssInsOrientationStamped final {
  Header header;
  GnssInsOrientation orientation;
};

// struct Pose final {
//   float x;
//   float y;
//   float theta;
//   float linear_velocity;
//   float angular_velocity;

//   bool operator==(Pose const &) const noexcept;
//   bool operator!=(Pose const &) const noexcept;
//   using IsRelocatable = ::std::true_type;
// };

namespace geometry_msgs
{
  struct Pose final {
    Point position;
    Quaternion orientation;

    bool operator==(Pose const &) const noexcept;
    bool operator!=(Pose const &) const noexcept;
    using IsRelocatable = ::std::true_type;
  };
};


struct PoseWithCovariance
{
    geometry_msgs::Pose pose;
    float covariance[36];
};
namespace geometry_msgs
{

};


namespace geometry_msgs 
{
  struct PoseStamped
  {
    Header header;
    Pose pose;
  };
  struct PoseWithCovarianceStamped
  {
    Header header;
    PoseWithCovariance pose;
  };
  #ifndef CXXBRIDGE1_STRUCT_geometry_msgs$Vector3
  #define CXXBRIDGE1_STRUCT_geometry_msgs$Vector3
  struct Vector3 final {
    double x;
    double y;
    double z;

    bool operator==(Vector3 const &) const noexcept;
    bool operator!=(Vector3 const &) const noexcept;
    using IsRelocatable = ::std::true_type;
  };
  #endif  // CXXBRIDGE1_STRUCT_geometry_msgs$Vector3

  #ifndef CXXBRIDGE1_STRUCT_geometry_msgs$Transform
  #define CXXBRIDGE1_STRUCT_geometry_msgs$Transform
  struct Transform final {
    geometry_msgs::Vector3 translation;
    geometry_msgs::Quaternion rotation;

    bool operator==(Transform const &) const noexcept;
    bool operator!=(Transform const &) const noexcept;
    using IsRelocatable = ::std::true_type;
  };
  #endif // CXXBRIDGE1_STRUCT_geometry_msgs$Transform

  #ifndef CXXBRIDGE1_STRUCT_geometry_msgs$TransformStamped
  #define CXXBRIDGE1_STRUCT_geometry_msgs$TransformStamped
  struct TransformStamped final {
    Header header;
    std::string child_frame_id;
    geometry_msgs::Transform transform;

    bool operator==(TransformStamped const &) const noexcept;
    bool operator!=(TransformStamped const &) const noexcept;
    using IsRelocatable = ::std::true_type;
  };
  #endif // CXXBRIDGE1_STRUCT_geometry_msgs$TransformStamped


}


namespace tf2
{
struct Transform final {
  geometry_msgs::Vector3 translation;
  geometry_msgs::Quaternion rotation;

  bool operator==(Transform const &) const noexcept;
  bool operator!=(Transform const &) const noexcept;
  using IsRelocatable = ::std::true_type;
};
}


namespace builtin_interfaces {
#ifndef CXXBRIDGE1_STRUCT_builtin_interfaces$Time
#define CXXBRIDGE1_STRUCT_builtin_interfaces$Time
struct Time final {
  ::std::int32_t sec;
  ::std::uint32_t nanosec;

  bool operator==(Time const &) const noexcept;
  bool operator!=(Time const &) const noexcept;
  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_builtin_interfaces$Time
} // namespace builtin_interfaces


#endif  // GNSS_POSER__MSG_HPP_