#include <string>
#include <vector>
// #include <ros/ros.h>
// #include "ros/package.h"
#include <stdio.h>
#include <stack>
#include <math.h>
#include <time.h>
#include <pcl/common/common.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
// #include <nav_msgs/Odometry.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_datatypes.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <tf/transform_listener.h>
// #include <dora-ros2-bindings.h>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include "tracker_object.h"
#include "../include/msg.hpp"

class Track
{

public:
  Track();
  ~Track() {}
  void setNewObjects(std::vector<tracker> trackers, double frame_time);
  void getObjects(std::vector<tracker> &trackers);
  void getCarPose(Eigen::Matrix3f &carpose);

private:
  void predictOldTrackers(double time);
  void updateTrackers(std::vector<tracker> trackers, std::vector<std::pair<int, int> > assignments,
                      std::vector<int> unassigned_tracks, std::vector<int> unassigned_objects, double diff_time);
  void createNewTrackers(std::vector<tracker> trackers);
  void computeAssociateMatrix(const std::vector<tracker> &tracks, const std::vector<tracker> &new_objects,
                              std::vector<std::vector<double> > &association_mat);
  void AssignObjectsToTracks(std::vector<int> tracks_idx, std::vector<int> objects_idx,
                             std::vector<std::vector<double> > cost,
                             std::vector<std::pair<int, int> > *assignments,
                             std::vector<int> *unassigned_tracks, std::vector<int> *unassigned_objects);

  void draw_box(const tracker &tmp_tracker, const int &marker_id, visualization_msgs::Marker &marker);
  void draw_text(const tracker &pos, const int &marker_id, visualization_msgs::Marker &marker);
  void draw_arrow(const tracker &pos, const int &marker_id, visualization_msgs::Marker &marker);
  std::vector<tracker> Tracked_object_;         //跟踪物体
  std::vector<tracker> Current_Tracked_object_; //与当前帧有关的跟踪物体
  double last_frame_time_;
  double odo_time_;
  // ros::Publisher marker_pub;
  Eigen::Matrix3f car_pose_;
  // tf::TransformListener listener;
};
