/*
 * Copyright 2021 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <bezier_sampler/node.hpp>

namespace bezier_sampler
{
PathSmootherNode::PathSmootherNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
  // TODO read parameters
  path_pub_ = pnh_.advertise<autoware_auto_planning_msgs::msg::Trajectory>("output/trajectory", 1);
  debug_paths_pub_ = pnh_.advertise<::bezier_sampler::DebugPaths>("output/debug/paths", 1);
  path_sub_ = pnh_.subscribe("input/path", 1, &PathSmootherNode::pathCallback, this);
}

// ROS callback functions

void PathSmootherNode::pathCallback(const autoware_auto_planning_msgs::msg::Path & msg)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  autoware_auto_planning_msgs::msg::Trajectory output_traj_msg;
  output_traj_msg.header = msg.header;

  bool debug_mode = true;
  ::bezier_sampler::DebugPaths debug_paths;
  // Sampling parameters
  params_.nb_k = 3;
  params_.mk_min = 0.0;
  params_.mk_max = 10.0;
  params_.nb_t = 10;
  params_.mt_min = 0.3;
  params_.mt_max = 1.7;
  // Constraint checker
  cc_params_.ego_width = 1.2;
  cc_params_.ego_front_length = 2.0;
  cc_params_.ego_rear_length = 0.5;
  cc_params_.nb_points = 20;
  cc_params_.maximum_curvature = 0.5;
  cc_params_.hard_safety_margin = 0.4;
  // TODO use current pose/heading for the first point
  // TODO replanning: reuse the previously valid path
  bezier_sampler::ConstraintChecker cc(msg.drivable_area, cc_params_);
  std::vector<bezier_sampler::Bezier> subpaths;
  // Split path and sample curves for each subpath
  for (const std::pair<
         bezier_sampler::sampler_common::State, bezier_sampler::sampler_common::State> & configs :
       bezier_sampler::splitPath(msg.points, 7.0, 30.0, getCurrentEgoPoseIndex(msg))) {
    std::vector<bezier_sampler::Bezier> sampled_paths =
      sample(configs.first, configs.second, params_);
    // Populate debug message
    if (debug_mode) {
      for (int i = 0; i < sampled_paths.size(); ++i) {
        geometry_msgs::PoseArray debug_path;
        for (const autoware_auto_planning_msgs::msg::PathPoint & p :
             bezier_sampler::toPathPoints(sampled_paths[i], cc_params_.nb_points))
          debug_path.poses.push_back(p.pose);
        debug_paths.paths.push_back(debug_path);

        debug_paths.drivable.push_back(cc.isDrivable(sampled_paths[i]));
        debug_paths.collision_free.push_back(cc.isCollisionFree(sampled_paths[i]));
        // cc.cost(path);
      }
    }
    bool valid = false;
    int drivable = 0;
    int coll_free = 0;
    for (const bezier_sampler::Bezier & sampled_path : sampled_paths) {
      // todo: keep the *best* valid one
      const bool is_drivable = cc.isDrivable(sampled_path);
      const bool is_collision_free = cc.isCollisionFree(sampled_path);
      drivable += is_drivable;
      coll_free += is_collision_free;

      if (is_drivable and is_collision_free) {
        valid = true;
        subpaths.push_back(sampled_path);
        break;
      }
    }
    if (not valid) {
      ROS_WARN("Could not build a valid path");
      ROS_WARN(
        "\tDrivable/CollisionFree (out of %d): %d | %d", sampled_paths.size(), drivable, coll_free);
    }
  }
  // Convert subpaths to msg
  for (const bezier_sampler::Bezier & b : subpaths) {
    bezier_sampler::polygon_t footprint = cc.buildFootprintPolygon(b);
    geometry_msgs::PoseArray poses;
    for (const Eigen::Vector2d & v : footprint.outer()) {
      geometry_msgs::Pose p;
      p.position.x = v.x();
      p.position.y = v.y();
      poses.poses.push_back(p);
    }
    debug_paths.footprints.push_back(poses);
  }
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> path_points =
    bezier_sampler::toPathPoints(subpaths, cc_params_.nb_points);
  // Velocity Planning
  for (autoware_auto_planning_msgs::msg::PathPoint p : path_points) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint traj_point;
    traj_point.pose = p.pose;
    traj_point.twist.linear.x = 5.0;
    output_traj_msg.points.push_back(traj_point);
  }
  // Publish sampled curve for debugging
  if (debug_mode) debug_paths_pub_.publish(debug_paths);
  ROS_WARN(
    "[Path Smoother] Execution time: %d ms",
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start_time));

  path_pub_.publish(output_traj_msg);
}

std::unique_ptr<geometry_msgs::Pose> PathSmootherNode::getCurrentEgoPose()
{
  geometry_msgs::TransformStamped tf_current_pose;

  try {
    tf_current_pose =
      tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("[BlindspotSafetyPlanner] %s", ex.what());
    return nullptr;
  }

  geometry_msgs::Pose p;
  p.orientation = tf_current_pose.transform.rotation;
  p.position.x = tf_current_pose.transform.translation.x;
  p.position.y = tf_current_pose.transform.translation.y;
  p.position.z = tf_current_pose.transform.translation.z;
  std::unique_ptr<geometry_msgs::Pose> p_ptr = std::make_unique<geometry_msgs::Pose>(p);
  return p_ptr;
}
int PathSmootherNode::getCurrentEgoPoseIndex(
  const autoware_auto_planning_msgs::msg::Path & path_msg)
{
  // identify index of the current trajectory point TODO might break if ego pose is beyond the last
  // traj point
  std::unique_ptr<geometry_msgs::Pose> ego_pose = getCurrentEgoPose();
  double prev_dist(std::numeric_limits<double>::max());
  auto traj_point_it = std::find_if(
    path_msg.points.begin(), path_msg.points.end(),
    [&](autoware_auto_planning_msgs::msg::PathPoint p) {
      double dist = std::sqrt(
        (ego_pose->position.x - p.pose.position.x) * (ego_pose->position.x - p.pose.position.x) +
        (ego_pose->position.y - p.pose.position.y) * (ego_pose->position.y - p.pose.position.y));
      if (dist > prev_dist) return true;
      prev_dist = dist;
      return false;
    });
  return std::distance(path_msg.points.begin(), traj_point_it) - 1;
}
}  // namespace bezier_sampler
