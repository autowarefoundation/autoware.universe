// Copyright 2020 Tier IV, Inc.
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
/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef TIER4_PCL_EXTENSIONS__JOINT_ICP_EXTENDED_HPP_
#define TIER4_PCL_EXTENSIONS__JOINT_ICP_EXTENDED_HPP_

#include <pcl/pcl_config.h>

#if PCL_VERSION < PCL_VERSION_CALC(1, 12, 0)
#include <pcl/filters/boost.h>
#endif

#include <pcl/point_types.h>
#include <pcl/registration/joint_icp.h>

#include <limits>
#include <map>
#include <vector>

namespace pcl
{
template <typename PointSource, typename PointTarget, typename Scalar = float>
class JointIterativeClosestPointExtended
: public JointIterativeClosestPoint<PointSource, PointTarget, Scalar>
{
public:
  using Ptr = pcl::shared_ptr<JointIterativeClosestPointExtended<PointSource, PointTarget, Scalar>>;
  using ConstPtr =
    pcl::shared_ptr<const JointIterativeClosestPointExtended<PointSource, PointTarget, Scalar>>;

  using PointCloudSource =
    typename IterativeClosestPoint<PointSource, PointTarget, Scalar>::PointCloudSource;
  using PointCloudSourcePtr = typename PointCloudSource::Ptr;
  using PointCloudSourceConstPtr = typename PointCloudSource::ConstPtr;

  using PointCloudTarget =
    typename IterativeClosestPoint<PointSource, PointTarget, Scalar>::PointCloudTarget;
  using PointCloudTargetPtr = typename PointCloudTarget::Ptr;
  using PointCloudTargetConstPtr = typename PointCloudTarget::ConstPtr;

  using KdTree = pcl::search::KdTree<PointTarget>;
  using KdTreePtr = typename KdTree::Ptr;

  using KdTreeReciprocal = pcl::search::KdTree<PointSource>;
  using KdTreeReciprocalPtr = typename KdTree::Ptr;

  using PointIndicesPtr = PointIndices::Ptr;
  using PointIndicesConstPtr = PointIndices::ConstPtr;

  using CorrespondenceEstimation =
    pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>;
  using CorrespondenceEstimationPtr = typename CorrespondenceEstimation::Ptr;
  using CorrespondenceEstimationConstPtr = typename CorrespondenceEstimation::ConstPtr;

  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::reg_name_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::getClassName;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::setInputSource;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::input_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::indices_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::target_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::nr_iterations_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::max_iterations_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::previous_transformation_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::final_transformation_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::transformation_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::transformation_epsilon_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::converged_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::corr_dist_threshold_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::inlier_threshold_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::min_number_correspondences_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::update_visualizer_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::euclidean_fitness_epsilon_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::correspondences_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::transformation_estimation_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::correspondence_estimation_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::correspondence_rejectors_;

  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::use_reciprocal_correspondence_;

  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::convergence_criteria_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::source_has_normals_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::target_has_normals_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::need_source_blob_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::need_target_blob_;

  /** \brief XYZ fields offset. */
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::x_idx_offset_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::y_idx_offset_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::z_idx_offset_;

  /** \brief Normal fields offset. */
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::nx_idx_offset_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::ny_idx_offset_;
  using IterativeClosestPoint<PointSource, PointTarget, Scalar>::nz_idx_offset_;

  using JointIterativeClosestPoint<PointSource, PointTarget, Scalar>::sources_;
  using JointIterativeClosestPoint<PointSource, PointTarget, Scalar>::targets_;
  using JointIterativeClosestPoint<PointSource, PointTarget, Scalar>::correspondence_estimations_;

  using Matrix4 = typename IterativeClosestPoint<PointSource, PointTarget, Scalar>::Matrix4;

  /** \brief Empty constructor. */
  JointIterativeClosestPointExtended()
  {
    IterativeClosestPoint<PointSource, PointTarget, Scalar>();
    reg_name_ = "JointIterativeClosestPointExtended";
  };

  /** \brief Empty destructor */
  ~JointIterativeClosestPointExtended() override = default;

  double getFitnessScore(double max_range = std::numeric_limits<double>::max());

protected:
  /** \brief Rigid transformation computation method  with initial guess.
   * \param output the transformed input point cloud dataset using the rigid
   * transformation found \param guess the initial guess of the transformation to
   * compute
   */
  void computeTransformation(PointCloudSource & output, const Matrix4 & guess) override;

  std::vector<KdTreeReciprocalPtr> src_trees_;
  std::vector<KdTreePtr> tgt_trees_;
};
}  // namespace pcl

#ifdef PCL_NO_PRECOMPILE
//  #include <pcl/filters/impl/voxel_grid_covariance.hpp>
#include <does_this_even_exist.hpp>
#endif

#endif  // TIER4_PCL_EXTENSIONS__JOINT_ICP_EXTENDED_HPP_
