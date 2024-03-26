#pragma once
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <sstream>
#include <string>
#include <Eigen/Geometry>
#include "_DiagnosticStatus.h"
#include "DiagnosticArray.h"
#include "PoseWithCovarianceStamped.h"
#include "TransformStamped.h"
#include "PoseStamped.h"
#include "Pose.h"
#include "Float32.h"
#include <boost/shared_ptr.hpp>
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

class NdtLocalizer
{
public:
    NdtLocalizer();
    ~NdtLocalizer();

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;

    Eigen::Matrix4f base_to_sensor_matrix_;
    Eigen::Matrix4f pre_trans, delta_trans;
    bool init_pose = false;

    std::string base_frame_;
    std::string map_frame_;

    // init guess for ndt
    geometry_msgs::PoseWithCovarianceStamped initial_pose_cov_msg_;

    std::mutex ndt_map_mtx_;

    double converged_param_transform_probability_;
    std::thread diagnostic_thread_;
    std::map<std::string, std::string> key_value_stdmap_;

    // function
    void init_params();
    void timer_diagnostic();

    bool get_transform(const std::string &target_frame,
                       const std::string &source_frame,
                       const std::shared_ptr<geometry_msgs::TransformStamped> &transform_stamped_ptr);
    void publish_tf(const std::string &frame_id, const std::string &child_frame_id,
                    const geometry_msgs::PoseStamped &pose_msg);

    void callback_pointsmap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pointcloud2_msg_ptr);
    void callback_init_pose(std::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> &pose_conv_msg_ptr);
    void callback_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pointcloud2_msg_ptr);

}; // NdtLocalizer Core