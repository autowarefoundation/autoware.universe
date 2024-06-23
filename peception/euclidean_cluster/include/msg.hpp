#ifndef MSG_HPP_
#define MSG_HPP_

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
struct Point final {
  double x;
  double y;
  double z;

  bool operator==(Point const &) const noexcept;
  bool operator!=(Point const &) const noexcept;
  using IsRelocatable = ::std::true_type;
};
struct Quaternion final {
  double x;
  double y;
  double z;
  double w;

  bool operator==(Quaternion const &) const noexcept;
  bool operator!=(Quaternion const &) const noexcept;
  using IsRelocatable = ::std::true_type;
};
struct Pose final {
  Point position;
  Quaternion orientation;

  bool operator==(Pose const &) const noexcept;
  bool operator!=(Pose const &) const noexcept;
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
struct PoseWithCovariance
{
    geometry_msgs::Pose pose;
    float covariance[36];
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


  #ifndef CXXBRIDGE1_STRUCT_builtin_interfaces$Duration
  #define CXXBRIDGE1_STRUCT_builtin_interfaces$Duration
  struct Duration final {
    ::std::int32_t sec;
    ::std::uint32_t nanosec;

    bool operator==(Duration const &) const noexcept;
    bool operator!=(Duration const &) const noexcept;
    using IsRelocatable = ::std::true_type;
  };
  #endif // CXXBRIDGE1_STRUCT_builtin_interfaces$Duration

} // namespace builtin_interfaces


namespace std_msgs
{
    // struct Float32
    // {
    //   float data;
    // };

    #ifndef CXXBRIDGE1_STRUCT_std_msgs$ColorRGBA
    #define CXXBRIDGE1_STRUCT_std_msgs$ColorRGBA
    struct ColorRGBA final {
      float r;
      float g;
      float b;
      float a;

      bool operator==(ColorRGBA const &) const noexcept;
      bool operator!=(ColorRGBA const &) const noexcept;
      using IsRelocatable = ::std::true_type;
    };
    #endif // CXXBRIDGE1_STRUCT_std_msgs$ColorRGBA
}


namespace visualization_msgs {
  #ifndef CXXBRIDGE1_STRUCT_visualization_msgs$Marker
  #define CXXBRIDGE1_STRUCT_visualization_msgs$Marker
  struct Marker final {
    Header header;
    ::std::string ns;
    ::std::int32_t id;
    ::std::int32_t type_;
    ::std::int32_t action;
    ::geometry_msgs::Pose pose;
    ::geometry_msgs::Vector3 scale;
    ::std_msgs::ColorRGBA color;
    ::builtin_interfaces::Duration lifetime;
    bool frame_locked;
    ::std::vector<::geometry_msgs::Point> points;
    ::std::vector<::std_msgs::ColorRGBA> colors;
    ::std::string text;
    ::std::string mesh_resource;
    bool mesh_use_embedded_materials;

    bool operator==(Marker const &) const noexcept;
    bool operator!=(Marker const &) const noexcept;
    using IsRelocatable = ::std::true_type;
  };
  #endif // CXXBRIDGE1_STRUCT_visualization_msgs$Marker

  #ifndef CXXBRIDGE1_STRUCT_visualization_msgs$MarkerArray
  #define CXXBRIDGE1_STRUCT_visualization_msgs$MarkerArray
  struct MarkerArray final {
    std::vector<::visualization_msgs::Marker> markers;

    bool operator==(MarkerArray const &) const noexcept;
    bool operator!=(MarkerArray const &) const noexcept;
    using IsRelocatable = ::std::true_type;
  };
  #endif // CXXBRIDGE1_STRUCT_visualization_msgs$MarkerArray
}




namespace custom_msgs
{
  struct LidarRawObject  
  {
    Point bbox_point[8];
    // geometry_msgs::Point32[8] bbox_point   // Bounding box points
    geometry_msgs::Vector3 lwh;             // Length, Width, Height
    float x_pos;                         // X position
    float y_pos;                         // Y position
    float z_pos;                         // Z position

    
  };

  struct LidarRawObjectArray
  {
      pcl::PCLHeader head;
      std::vector<LidarRawObject> objs;
  };
}



#endif  // MSG_HPP_