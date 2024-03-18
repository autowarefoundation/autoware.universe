#ifndef NDT_MAPPER_H
#define NDT_MAPPER_H

#include <fstream>
// #include <glog/logging.h>
#include <iostream>
#include <math.h>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <time.h>
#include <cmath>
#include <iomanip> 

#include <eigen3/Eigen/Dense>

// #include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/registration/ndt.h>
#include <pcl/conversions.h> //#include <pcl_conversions/pcl_conversions.h>


#include "ndt_cpu/NormalDistributionsTransform.h"

// #include <nlohmann/json.hpp>  // send tf and path to python for visualizing in rviz2
#include <time.h>  // replace the ros::time
#include "yaml-cpp/yaml.h"


#define M_PI       3.14159265358979323846

// using json = nlohmann::json;

namespace ndt_mapping{
class Pose {   // use directly
public:
  Pose() { x = y = z = roll = pitch = yaw = 0.0; }
  virtual ~Pose() = default;
  void setPose(const Eigen::Matrix4f &t, Eigen::Matrix3f &m) {  
    x = t(0, 3);
    y = t(1, 3);
    z = t(2, 3);
    tf_getrpy(roll, pitch, yaw, m);
  }

  void tf_getrpy(double &roll, double &pitch, double &yaw, Eigen::Matrix3f &m){
    struct Euler{
      double yaw;
      double pitch;
      double roll;
    };
    Euler euler_out;
    if(fabs(m(2,0)) >= 1){
      euler_out.yaw = 0;
      double delta = atan2(m(2,1), m(2,2));  // tf2Scalar delta = tf2Atan2(m_el[2].y(),m_el[2].z());
      if(m(2,0) < 0){
        euler_out.pitch = M_PI / 2.0;
        euler_out.roll = delta;
      }
      else{
        euler_out.pitch = -M_PI /2.0;
        euler_out.roll = delta;
      }
    }
    else{
      if(m(2,0) < -1){
        euler_out.pitch = asin(-1);
      }
      else if (m(2, 0) > 1)
      {
        euler_out.pitch = asin(1);
      }
      else{
        euler_out.pitch = asin(m(2,0));
      }

      euler_out.roll = atan2(m(2,1)/cos(euler_out.pitch),
                              m(2,2)/cos(euler_out.pitch));

      euler_out.yaw = atan2(m(1,0)/cos(euler_out.pitch),
                              m(0,1)/cos(euler_out.pitch));
      
    }
    yaw = euler_out.yaw;
    pitch = euler_out.pitch;
    roll = euler_out.roll;

  }

  double calDistance() {
    double dis = sqrt(x * x + y * y + z * z);
    return dis;
  }
  void operator=(const Pose &p) {
    x = p.x;
    y = p.y;
    z = p.z;
    roll = p.roll;
    pitch = p.pitch;
    yaw = p.yaw;
  }

  Pose operator+(const Pose &p) const {
    Pose res;
    res.x = x + p.x;
    res.y = y + p.y;
    res.z = z + p.z;
    res.roll = roll + p.roll;
    res.pitch = pitch + p.pitch;
    res.yaw = yaw + p.yaw;
    return res;
  }

  Pose operator-(const Pose &p) const {
    Pose res;
    res.x = x - p.x;
    res.y = y - p.y;
    res.z = z - p.z;

    double diff_rad = yaw - p.yaw;
    if (diff_rad >= M_PI)
      diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
      diff_rad = diff_rad + 2 * M_PI;
    res.yaw = diff_rad;

    // TODO ? is necessary ?
    res.roll = 0;
    res.pitch = 0;

    return res;
  }

  friend inline std::ostream &operator<<(std::ostream &os, const Pose &p) {
    os << std::fixed << std::setprecision(2) << " Position: (x:" << p.x
       << ") (y:" << p.y << ") (z:" << p.z << "); "
       << "Rotation: (roll:" << p.roll << ") (pitch:" << p.pitch
       << ") (yaw:" << p.yaw << ")\n";
    return os;
  }

  double x, y, z, roll, pitch, yaw;
};


class NDTMapper{
public:
  struct Config {

    int save_frame_point = 10;

    // default param
    int max_iter = 30;            // Maximum iterations
    float ndt_res = 1.0;          // Resolution
    double step_size = 0.1;       // Step size
    double trans_eps = 0.01;      // Transformation epsilon
    double voxel_leaf_size = 2.0; // Leaf size of VoxelGrid filter.
    double min_scan_range = 5.0;
    double max_scan_range = 200.0;
    double min_add_scan_shift = 1.0;
  };

  // -----------------------------------------------------------------------
  NDTMapper();
  virtual ~NDTMapper() = default;
  // ------------------------------------------------------------------------
  void points_callback(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &input);

  void saveMap();
  const Config &getConfig() const { return config_; }

  Eigen::Matrix4f Pose2Matrix(const Pose &p);

  Eigen::Matrix3f setValue(const Eigen::Matrix4f &t) {
    Eigen::Matrix3f mat;
    mat << static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)),
                static_cast<double>(t(0, 2)), static_cast<double>(t(1, 0)),
                static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
                static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)),
                static_cast<double>(t(2, 2))  ;
    return mat;
  }

  cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;

private:
  // Setup.
  void setConfig();
  void getTF();
  void setTF();


  // function
  pcl::PointCloud<pcl::PointXYZI>::Ptr
  filterPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr &tmp); //pcl::PointCloud<pcl::PointXYZI>::Ptr filterPoints(const sensor_msgs::PointCloud2::ConstPtr &input);
  
  Eigen::Matrix4f calTranslation(const pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr,
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr);

  // void pubOtherMsgs(const sensor_msgs::PointCloud2::ConstPtr &input);

  // variables
  Config config_;

  // things we'd like to calculate
  Pose previous_pose, current_pose, added_pose;
  Pose imu_pose, odom_pose, guess_lidar_pose, guess_base_pose;
  Pose diff_pose, offset_imu, offset_odom;

  Eigen::Matrix4f tf_btol, tf_ltob;

  //------------------------------------------------------------------------------
 
  pcl::PointCloud<pcl::PointXYZI> map;
  std::vector<float> b2l_tf;
  std::string lidar_frame;

  bool no_b2l_tf = false;
  bool _use_imu = false;
  bool _use_odom = false;
  bool _imu_upside_down = false;
  bool _odom_inverse = false;
  bool _debug_print = false;
  bool _incremental_voxel_update = false;

  std::string _imu_topic = "/imu_raw";
  std::string _lidar_topic = "points_raw";
  std::string _odom_topic = "/vehicle/odom";
  std::string _output_odom_topic = "/guess_pose_odom";
  std::string _odom_lidar_topic = "/odom_lidar";
  std::string _output_offset_odom = "/offset_leg_odom";

  std::string _save_map_path;
  bool _ros_visualize;


  // ndt result
  double fitness_score;
  double transformation_probability;
  int final_num_iteration;
  bool has_converged;

  // save lock resource
  std::mutex lockPointCloud;

  // json tf_message;
  // json path_message;

  time_t current_scan_time;
  time_t previous_scan_time;

};


}

#endif