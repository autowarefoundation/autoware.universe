#ifndef GS_GBS_COMPONENT_H_INCLUDED
#define GS_GBS_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GS_GBS_EXPORT __attribute__ ((dllexport))
    #define GS_GBS_IMPORT __attribute__ ((dllimport))
  #else
    #define GS_GBS_EXPORT __declspec(dllexport)
    #define GS_GBS_IMPORT __declspec(dllimport)
  #endif
  #ifdef GS_GBS_BUILDING_DLL
    #define GS_GBS_PUBLIC GS_GBS_EXPORT
  #else
    #define GS_GBS_PUBLIC GS_GBS_IMPORT
  #endif
  #define GS_GBS_PUBLIC_TYPE GS_GBS_PUBLIC
  #define GS_GBS_LOCAL
#else
  #define GS_GBS_EXPORT __attribute__ ((visibility("default")))
  #define GS_GBS_IMPORT
  #if __GNUC__ >= 4
    #define GS_GBS_PUBLIC __attribute__ ((visibility("default")))
    #define GS_GBS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define GS_GBS_PUBLIC
    #define GS_GBS_LOCAL
  #endif
  #define GS_GBS_PUBLIC_TYPE
#endif

#if __cplusplus
} // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/empty.hpp>

#include <lidarslam_msgs/msg/map_array.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/block_solver.h"

#include "g2o/solvers/eigen/linear_solver_eigen.h"

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"

#include <mutex>

namespace graphslam
{
  class GraphBasedSlamComponent: public rclcpp::Node
  {
public:
    GS_GBS_PUBLIC
    explicit GraphBasedSlamComponent(const rclcpp::NodeOptions & options);

private:
    std::mutex mtx_;

    rclcpp::Clock clock_;
    tf2_ros::Buffer tfbuffer_;
    tf2_ros::TransformListener listener_;
    tf2_ros::TransformBroadcaster broadcaster_;

    pcl::Registration < pcl::PointXYZI, pcl::PointXYZI > ::Ptr registration_;
    pcl::VoxelGrid < pcl::PointXYZI > voxelgrid_;

    lidarslam_msgs::msg::MapArray map_array_msg_;
    rclcpp::Subscription < lidarslam_msgs::msg::MapArray > ::SharedPtr map_array_sub_;
    rclcpp::Publisher < lidarslam_msgs::msg::MapArray > ::SharedPtr modified_map_array_pub_;
    rclcpp::Publisher < nav_msgs::msg::Path > ::SharedPtr modified_path_pub_;
    rclcpp::Publisher < sensor_msgs::msg::PointCloud2 > ::SharedPtr modified_map_pub_;
    rclcpp::TimerBase::SharedPtr loop_detect_timer_;
    rclcpp::Service < std_srvs::srv::Empty > ::SharedPtr map_save_srv_;

    void initializePubSub();
    void searchLoop();
    void doPoseAdjustment(lidarslam_msgs::msg::MapArray map_array_msg, bool do_save_map);
    void publishMapAndPose();

    // loop search parameter
    int loop_detection_period_;
    double threshold_loop_closure_score_;
    double distance_loop_closure_;
    double range_of_searching_loop_closure_;
    int search_submap_num_;

    // pose graph optimization parameter
    int num_adjacent_pose_cnstraints_;
    bool use_save_map_in_loop_ {true};

    bool initial_map_array_received_ {false};
    bool is_map_array_updated_ {false};
    int previous_submaps_num_ {0};

    struct LoopEdge
    {
      std::pair < int, int > pair_id;
      Eigen::Isometry3d relative_pose;
    };
    std::vector < LoopEdge > loop_edges_;

    bool debug_flag_ {false};

  };
}

#endif  //GS_GBS_COMPONENT_H_INCLUDED
