// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "freespace_planning_algorithms/rrtstar_core.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <queue>

namespace rrtstar_core
{
CSpace::CSpace(
  const Pose & lo, const Pose & hi, double r, std::function<bool(Pose)> is_obstacle_free)
: lo_(lo), hi_(hi), rsspace_(ReedsSheppStateSpace(r)), is_obstacle_free_(is_obstacle_free)
{
  std::random_device device;
  std::mt19937 gen(device());
  rand_gen_ = gen;
}

bool CSpace::isInside(const Pose & p) const
{
  if (p.x > hi_.x) return false;
  if (p.y > hi_.y) return false;
  if (p.x < lo_.x) return false;
  if (p.y < lo_.y) return false;
  return true;
}

Pose CSpace::uniformSampling()
{
  std::uniform_real_distribution<double> distr_x(lo_.x, hi_.x);
  std::uniform_real_distribution<double> distr_y(lo_.y, hi_.y);
  std::uniform_real_distribution<double> distr_yaw(0.0, 2 * M_PI);
  return Pose{distr_x(rand_gen_), distr_y(rand_gen_), distr_yaw(rand_gen_)};
}

Pose CSpace::ellipticInformedSampling(double c_best, const Pose & p_start, const Pose & p_goal)
{
  const double c_min = distanceLowerBound(p_start, p_goal);
  const double ellipse_r1 = c_best * 0.5;
  const double ellipse_r2 = sqrt(c_best * c_best - c_min * c_min) * 0.5;
  const double angle = atan2(p_goal.y - p_start.y, p_goal.x - p_start.x);
  const double x_center = (p_start.x + p_goal.x) * 0.5;
  const double y_center = (p_start.y + p_goal.y) * 0.5;

  std::uniform_real_distribution<double> distr_x(-ellipse_r1, ellipse_r1);
  std::uniform_real_distribution<double> distr_y(-ellipse_r2, ellipse_r2);
  std::uniform_real_distribution<double> distr_yaw(0, 2 * M_PI);
  while (true) {
    const double x_local = distr_x(rand_gen_);
    const double y_local = distr_y(rand_gen_);

    const bool is_inside_ellipse =
      (pow(x_local / ellipse_r1, 2) + pow(y_local / ellipse_r2, 2) < 1.0);
    if (!is_inside_ellipse)
      // Note that in the original paper, sample are generated directly from
      // the uniform distribution over the super-ellipsoid. However, because
      // our targetted-problem is 2-dim, it is fine to use just a rejection
      // sampling.
      continue;

    const double x_global = cos(angle) * x_local + -sin(angle) * y_local + x_center;
    const double y_global = sin(angle) * x_local + cos(angle) * y_local + y_center;
    const double yaw = distr_yaw(rand_gen_);
    return Pose{x_global, y_global, yaw};
  }
}

Pose CSpace::interpolate_child2parent(
  const Pose & pose_child, const Pose & pose_parent, double seg) const
{
  auto path = rsspace_.reedsShepp(pose_child, pose_parent);
  return interpolate(path, pose_child, seg);
}

Pose CSpace::interpolate(Path & path, const Pose & pose, double seg) const
{
  return rsspace_.interpolate(pose, path, seg / rsspace_.rho_);
}

void CSpace::sampleWayPoints_child2parent(
  const Pose & pose_child, const Pose & pose_parent, double step,
  std::vector<Pose> & pose_seq) const
{
  auto path = rsspace_.reedsShepp(pose_child, pose_parent);
  const double path_true_length = path.length() * rsspace_.rho_;
  for (double seg = 0; seg < path_true_length; seg += step) {
    const Pose pose_sampled = interpolate(path, pose_child, seg);
    pose_seq.push_back(pose_sampled);
  }
}

bool CSpace::isValidPath_child2parent(
  const Pose & pose_child, const Pose & pose_parent, double step) const
{
  auto path = rsspace_.reedsShepp(pose_child, pose_parent);
  const double path_true_length = path.length() * rsspace_.rho_;
  if (!isValidPose(pose_parent)) {
    return false;
  }
  for (double seg = 0; seg < path_true_length; seg += step) {
    Pose && pose_sampled = interpolate(path, pose_child, seg);
    if (!isValidPose(pose_sampled)) {
      return false;
    }
  }
  return true;
}

RRTStar::RRTStar(
  Pose x_start, Pose x_goal, double mu, double collision_check_resolution, bool is_informed,
  CSpace cspace)
: mu_(mu),
  collision_check_resolution_(collision_check_resolution),
  is_informed_(is_informed),
  cspace_(cspace)
{
  node_start_ = Node{x_start, 0.0, inf, 0};
  node_goal_ = Node{x_goal, inf, 0.0};
  nodes_.push_back(node_start_);
}

void RRTStar::extend()
{
  // Determine new node
  Pose x_rand = (isSolutionFound() && is_informed_)
                  ? cspace_.ellipticInformedSampling(
                      node_goal_.cost_from_start, node_start_.pose, node_goal_.pose)
                  : cspace_.uniformSampling();

  const Node node_nearest = nodes_[findNearestIndex(x_rand)];

  // NOTE: no child-parent relation here
  const Pose x_new = cspace_.interpolate_child2parent(node_nearest.pose, x_rand, mu_);

  if (!cspace_.isValidPath_child2parent(x_new, node_nearest.pose, collision_check_resolution_)) {
    return;
  }

  std::vector<size_t> neighbore_indexes = findNeighboreIndexes(x_new);

  const size_t idx_best_parent = getBestParentIndex(x_new, node_nearest, neighbore_indexes);
  Node & node_new = addNewNode(x_new, idx_best_parent);

  // Rewire
  const auto idx_rewire = getRewireTargetIndex(node_new, neighbore_indexes);
  if (idx_rewire != boost::none) {
    rewire(node_new, nodes_[*idx_rewire]);
  }

  // Check if reached
  bool is_reached =
    cspace_.isValidPath_child2parent(node_goal_.pose, node_new.pose, collision_check_resolution_);
  if (is_reached) {
    node_new.cost_to_goal = cspace_.distance(node_new.pose, node_goal_.pose);
    reached_nodes_.push_back(node_new);
  }

  if (isSolutionFound()) {
    // This cannot be inside if(is_reached){...} because we must update this anytime after rewiring
    // takes place
    boost::optional<size_t> idx_min = boost::none;
    double cost_min = inf;
    for (const auto & node : reached_nodes_) {
      const double cost = node.cost_from_start + node.cost_to_goal;
      if (cost < cost_min) {
        cost_min = cost;
        idx_min = *node.idx;
      }
    }
    node_goal_.cost_from_start = cost_min;
    node_goal_.parent_idx = *idx_min;
    node_goal_.cost_to_parent = nodes_[*idx_min].cost_to_goal;
  }
}

std::vector<Pose> RRTStar::sampleSolutionWaypoints(double resolution) const
{
  std::vector<Pose> poses;
  auto node = node_goal_;
  while (node.parent_idx != boost::none) {
    const auto node_parent = nodes_[*node.parent_idx];
    cspace_.sampleWayPoints_child2parent(node.pose, node_parent.pose, resolution, poses);
    node = node_parent;
  }
  poses.push_back(node_start_.pose);
  std::reverse(poses.begin(), poses.end());
  return poses;
}

void RRTStar::dumpState(std::string filename) const
{
  // Dump information of all nodes
  using json = nlohmann::json;

  auto serialize_node = [&](const Node & n) {
    json j;
    j["pose"] = {n.pose.x, n.pose.y, n.pose.yaw};
    if (n.idx == boost::none) {
      j["idx"] = -1;
    } else {
      j["idx"] = *n.idx;
    }
    if (n.parent_idx == boost::none) {
      j["parent_idx"] = -1;
    } else {
      j["parent_idx"] = *n.parent_idx;

      // fill trajectory from parent to this node
      std::vector<Pose> poses;
      cspace_.sampleWayPoints_child2parent(
        n.pose, nodes_[*n.parent_idx].pose, collision_check_resolution_, poses);
      for (const auto & pose : poses) {
        j["traj_piece"].push_back({pose.x, pose.y, pose.yaw});
      }
    }
    return j;
  };

  json j;
  j["radius"] = cspace_.getReedsSheppRadius();
  for (const auto & node : nodes_) {
    j["nodes"].push_back(serialize_node(node));
  }
  j["node_goal"] = serialize_node(node_goal_);
  std::ofstream file;
  file.open(filename);
  file << j;
  file.close();
}

size_t RRTStar::findNearestIndex(const Pose & x_rand) const
{
  double dist_min = inf;
  boost::optional<size_t> idx_min = boost::none;
  for (const auto & node : nodes_) {
    if (cspace_.distanceLowerBound(node.pose, x_rand) < dist_min) {
      const double dist_real = cspace_.distance(node.pose, x_rand);
      if (dist_real < dist_min) {
        dist_min = dist_real;
        idx_min = *node.idx;
      }
    }
  }
  return *idx_min;
}

std::vector<size_t> RRTStar::findNeighboreIndexes(const Pose & x_new) const
{
  // In the original paper of rrtstar, radius is shrinking over time.
  // However, because we use reeds-shepp distance metric instead of Euclidean metric,
  // it is hard to design the shirinking radius update. Usage of informed sampling
  // makes the problem far more complex, as the sampling reagion is shirinking over
  // the time.
  // Due to above difficulty in design of radius update, radius is simply fixed here.
  // In practice, the fixed radius setting works well as long as mu_ value is
  // propery tuned. In car planning scenario, the order or planning world area
  // is similar, and turining radius is also similar through different problems.
  // So, tuning mu_ parameter is not so difficult.

  const double radius_neighbore = mu_;

  std::vector<size_t> indexes;
  for (auto & node : nodes_) {
    if (cspace_.distanceLowerBound(node.pose, x_new) > radius_neighbore) continue;
    const bool is_neighbour = (cspace_.distance(node.pose, x_new) < radius_neighbore);
    if (is_neighbour) {
      indexes.push_back(*node.idx);
    }
  }
  return indexes;
}

RRTStar::Node & RRTStar::addNewNode(const Pose & pose, size_t idx_parent)
{
  const size_t idx_new_node = nodes_.size();
  const double cost_to_parent = cspace_.distance(pose, nodes_[idx_parent].pose);
  const double cost_from_start = nodes_[idx_parent].cost_from_start + cost_to_parent;
  const Node node_new{pose, cost_from_start, inf, idx_new_node, idx_parent, cost_to_parent};
  nodes_.push_back(node_new);
  // TODO(HiroIshida): check ?
  // When replacing the next line to
  // Node & node_parent = nodes_[idx_parent];
  // node_parent.child_indexes.push_back(idx_new_node);
  // It should do the same.
  // But, sometimes, idx_new_node is not pushed and then segmentation fault occurs
  // in erase opeation inside rewire method
  nodes_[idx_parent].child_indexes.push_back(idx_new_node);
  return nodes_[idx_new_node];
}

boost::optional<size_t> RRTStar::getRewireTargetIndex(
  const Node & node_new, const std::vector<size_t> & neighbore_indexes) const
{
  boost::optional<size_t> idx_rewire = boost::none;
  for (const size_t idx_neighbore : neighbore_indexes) {
    const Node & node_neighbore = nodes_[idx_neighbore];
    if (cspace_.isValidPath_child2parent(
          node_neighbore.pose, node_new.pose, collision_check_resolution_)) {
      const double cost_from_start_rewired =
        node_new.cost_from_start + cspace_.distance(node_new.pose, node_neighbore.pose);
      if (cost_from_start_rewired < node_neighbore.cost_from_start) {
        idx_rewire = *node_neighbore.idx;
      }
    }
  }
  return idx_rewire;
}

size_t RRTStar::getBestParentIndex(
  const Pose & pose_new, const Node & node_nearest,
  const std::vector<size_t> & neighbore_indexes) const
{
  size_t idx_cost_min = *node_nearest.idx;
  double cost_min = node_nearest.cost_from_start + cspace_.distance(node_nearest.pose, pose_new);
  for (const size_t idx_neighbore : neighbore_indexes) {
    const Node & node_neighbore = nodes_[idx_neighbore];
    const double cost_start_to_new =
      node_neighbore.cost_from_start + cspace_.distance(node_neighbore.pose, pose_new);
    if (cost_start_to_new < cost_min) {
      if (cspace_.isValidPath_child2parent(
            pose_new, node_neighbore.pose, collision_check_resolution_)) {
        idx_cost_min = *node_neighbore.idx;
        cost_min = cost_start_to_new;
      }
    }
  }
  return idx_cost_min;
}

void RRTStar::rewire(Node & node_a, Node & node_b)
{
  // connect node_a -> node_b
  // Initial state: node_a_parent -> node_a -> #nil; node_b_parent -> node_b -> #nil

  Node & node_b_parent = getParentNode(node_b);
  auto & child_indexes = node_b_parent.child_indexes;
  child_indexes.erase(std::find(child_indexes.begin(), child_indexes.end(), *node_b.idx));
  node_b.parent_idx = boost::none;
  node_b.cost_to_parent = boost::none;

  // Current state: node_a_parent -> node_a -> #nil; node_b_parent -> #nil; node_b -> #nil
  const double cost_a2b = cspace_.distance(node_a.pose, node_b.pose);
  node_a.child_indexes.push_back(*node_b.idx);
  node_b.parent_idx = *node_a.idx;
  node_b.cost_to_parent = cost_a2b;
  node_b.cost_from_start = node_a.cost_from_start + cost_a2b;
  // Current state: node_a_parent -> node_a -> node_b -> #nil; node_b_parent -> #nil;

  // update cost of all descendents of node_b
  std::queue<size_t> bfqueue;
  bfqueue.push(*node_b.idx);
  while (!bfqueue.empty()) {
    const Node node = nodes_[bfqueue.front()];
    bfqueue.pop();
    for (const size_t child_idx : node.child_indexes) {
      auto & node_child = nodes_[child_idx];
      node_child.cost_from_start = node.cost_from_start + *node_child.cost_to_parent;
      bfqueue.push(child_idx);
    }
  }
}

}  // namespace rrtstar_core
