/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * $Id$
 *
 */
#ifndef PCL_REGISTRATION_IMPL_GICP_OMP_HPP_
#define PCL_REGISTRATION_IMPL_GICP_OMP_HPP_

#include <atomic>
#include <pcl/registration/boost.h>
#include <pcl/registration/exceptions.h>

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget>
template<typename PointT> void
pclomp::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeCovariances(typename pcl::PointCloud<PointT>::ConstPtr cloud,
                                                                                    const typename pcl::search::KdTree<PointT>::ConstPtr kdtree,
                                                                                    MatricesVector& cloud_covariances)
{
  if (k_correspondences_ > int (cloud->size ()))
  {
    PCL_ERROR ("[pcl::GeneralizedIterativeClosestPoint::computeCovariances] Number of points in cloud (%lu) is less than k_correspondences_ (%lu)!\n", cloud->size (), k_correspondences_);
    return;
  }

  // We should never get there but who knows
  if(cloud_covariances.size () < cloud->size ())
    cloud_covariances.resize (cloud->size ());

  std::vector<std::vector<int>> nn_indices_array(omp_get_max_threads());
  std::vector<std::vector<float>> nn_dist_sq_array(omp_get_max_threads());

  #pragma omp parallel for
  for(std::size_t i=0; i < cloud->size(); i++) {
    auto& nn_indices = nn_indices_array[omp_get_thread_num()];
    auto& nn_dist_sq = nn_dist_sq_array[omp_get_thread_num()];

    const PointT &query_point = cloud->at(i);
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d &cov = cloud_covariances[i];
    // Zero out the cov and mean
    cov.setZero ();

    // Search for the K nearest neighbours
    kdtree->nearestKSearch(query_point, k_correspondences_, nn_indices, nn_dist_sq);

    // Find the covariance matrix
    for(int j = 0; j < k_correspondences_; j++) {
      const PointT &pt = (*cloud)[nn_indices[j]];

      mean[0] += pt.x;
      mean[1] += pt.y;
      mean[2] += pt.z;

      cov(0,0) += pt.x*pt.x;

      cov(1,0) += pt.y*pt.x;
      cov(1,1) += pt.y*pt.y;

      cov(2,0) += pt.z*pt.x;
      cov(2,1) += pt.z*pt.y;
      cov(2,2) += pt.z*pt.z;
    }

    mean /= static_cast<double> (k_correspondences_);
    // Get the actual covariance
    for (int k = 0; k < 3; k++)
      for (int l = 0; l <= k; l++)
      {
        cov(k,l) /= static_cast<double> (k_correspondences_);
        cov(k,l) -= mean[k]*mean[l];
        cov(l,k) = cov(k,l);
      }

    // Compute the SVD (covariance matrix is symmetric so U = V')
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU);
    cov.setZero ();
    Eigen::Matrix3d U = svd.matrixU ();
    // Reconstitute the covariance matrix with modified singular values using the column     // vectors in V.
    for(int k = 0; k < 3; k++) {
      Eigen::Vector3d col = U.col(k);
      double v = 1.; // biggest 2 singular values replaced by 1
      if(k == 2)   // smallest singular value replaced by gicp_epsilon
        v = gicp_epsilon_;
      cov+= v * col * col.transpose();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pclomp::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeRDerivative(const Vector6d &x, const Eigen::Matrix3d &R, Vector6d& g) const
{
  Eigen::Matrix3d dR_dPhi;
  Eigen::Matrix3d dR_dTheta;
  Eigen::Matrix3d dR_dPsi;

  double phi = x[3], theta = x[4], psi = x[5];

  double cphi = std::cos(phi), sphi = sin(phi);
  double ctheta = std::cos(theta), stheta = sin(theta);
  double cpsi = std::cos(psi), spsi = sin(psi);

  dR_dPhi(0,0) = 0.;
  dR_dPhi(1,0) = 0.;
  dR_dPhi(2,0) = 0.;

  dR_dPhi(0,1) = sphi*spsi + cphi*cpsi*stheta;
  dR_dPhi(1,1) = -cpsi*sphi + cphi*spsi*stheta;
  dR_dPhi(2,1) = cphi*ctheta;

  dR_dPhi(0,2) = cphi*spsi - cpsi*sphi*stheta;
  dR_dPhi(1,2) = -cphi*cpsi - sphi*spsi*stheta;
  dR_dPhi(2,2) = -ctheta*sphi;

  dR_dTheta(0,0) = -cpsi*stheta;
  dR_dTheta(1,0) = -spsi*stheta;
  dR_dTheta(2,0) = -ctheta;

  dR_dTheta(0,1) = cpsi*ctheta*sphi;
  dR_dTheta(1,1) = ctheta*sphi*spsi;
  dR_dTheta(2,1) = -sphi*stheta;

  dR_dTheta(0,2) = cphi*cpsi*ctheta;
  dR_dTheta(1,2) = cphi*ctheta*spsi;
  dR_dTheta(2,2) = -cphi*stheta;

  dR_dPsi(0,0) = -ctheta*spsi;
  dR_dPsi(1,0) = cpsi*ctheta;
  dR_dPsi(2,0) = 0.;

  dR_dPsi(0,1) = -cphi*cpsi - sphi*spsi*stheta;
  dR_dPsi(1,1) = -cphi*spsi + cpsi*sphi*stheta;
  dR_dPsi(2,1) = 0.;

  dR_dPsi(0,2) = cpsi*sphi - cphi*spsi*stheta;
  dR_dPsi(1,2) = sphi*spsi + cphi*cpsi*stheta;
  dR_dPsi(2,2) = 0.;

  g[3] = matricesInnerProd(dR_dPhi, R);
  g[4] = matricesInnerProd(dR_dTheta, R);
  g[5] = matricesInnerProd(dR_dPsi, R);
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> void
pclomp::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::estimateRigidTransformationBFGS (const PointCloudSource &cloud_src,
                                                                                                  const std::vector<int> &indices_src,
                                                                                                  const PointCloudTarget &cloud_tgt,
                                                                                                  const std::vector<int> &indices_tgt,
                                                                                                  Eigen::Matrix4f &transformation_matrix)
{
  if (indices_src.size () < 4)     // need at least 4 samples
  {
    PCL_THROW_EXCEPTION (pcl::NotEnoughPointsException,
                         "[pcl::GeneralizedIterativeClosestPoint::estimateRigidTransformationBFGS] Need at least 4 points to estimate a transform! Source and target have " << indices_src.size () << " points!");
    return;
  }
  // Set the initial solution
  Vector6d x = Vector6d::Zero ();
  x[0] = transformation_matrix (0,3);
  x[1] = transformation_matrix (1,3);
  x[2] = transformation_matrix (2,3);
  x[3] = std::atan2 (transformation_matrix (2,1), transformation_matrix (2,2));
  x[4] = asin (-transformation_matrix (2,0));
  x[5] = std::atan2 (transformation_matrix (1,0), transformation_matrix (0,0));

  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;
  tmp_idx_src_ = &indices_src;
  tmp_idx_tgt_ = &indices_tgt;

  // Optimize using forward-difference approximation LM
  const double gradient_tol = 1e-2;
  OptimizationFunctorWithIndices functor(this);
  BFGS<OptimizationFunctorWithIndices> bfgs (functor);
  bfgs.parameters.sigma = 0.01;
  bfgs.parameters.rho = 0.01;
  bfgs.parameters.tau1 = 9;
  bfgs.parameters.tau2 = 0.05;
  bfgs.parameters.tau3 = 0.5;
  bfgs.parameters.order = 3;

  int inner_iterations_ = 0;
  int result = bfgs.minimizeInit (x);
  result = BFGSSpace::Running;
  do
  {
    inner_iterations_++;
    result = bfgs.minimizeOneStep (x);
    if(result)
    {
      break;
    }
    result = bfgs.testGradient(gradient_tol);
  } while(result == BFGSSpace::Running && inner_iterations_ < max_inner_iterations_);
  if(result == BFGSSpace::NoProgress || result == BFGSSpace::Success || inner_iterations_ == max_inner_iterations_)
  {
    PCL_DEBUG ("[pcl::registration::TransformationEstimationBFGS::estimateRigidTransformation]");
    PCL_DEBUG ("BFGS solver finished with exit code %i \n", result);
    transformation_matrix.setIdentity();
    applyState(transformation_matrix, x);
  }
  else
    PCL_THROW_EXCEPTION(pcl::SolverDidntConvergeException,
                        "[pcl::" << getClassName () << "::TransformationEstimationBFGS::estimateRigidTransformation] BFGS solver didn't converge!");
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline double
pclomp::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::OptimizationFunctorWithIndices::operator() (const Vector6d& x)
{
  Eigen::Matrix4f transformation_matrix = gicp_->base_transformation_;
  gicp_->applyState(transformation_matrix, x);
  double f = 0;
  std::vector<double> f_array(omp_get_max_threads(), 0.0);

  int m = static_cast<int> (gicp_->tmp_idx_src_->size ());
  #pragma omp parallel for
  for(int i = 0; i < m; ++i)
  {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    pcl::Vector4fMapConst p_src = gicp_->tmp_src_->points[(*gicp_->tmp_idx_src_)[i]].getVector4fMap();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    pcl::Vector4fMapConst p_tgt = gicp_->tmp_tgt_->points[(*gicp_->tmp_idx_tgt_)[i]].getVector4fMap();
    // Estimate the distance (cost function)
    // The last coordinate is still guaranteed to be set to 1.0
    // Eigen::AlignedVector3<double> res(pp[0] - p_tgt[0], pp[1] - p_tgt[1], pp[2] - p_tgt[2]);
    // Eigen::AlignedVector3<double> temp(gicp_->mahalanobis((*gicp_->tmp_idx_src_)[i]) * res);
	Eigen::Vector4f res = transformation_matrix * p_src - p_tgt;
	Eigen::Matrix4f maha = gicp_->mahalanobis((*gicp_->tmp_idx_src_)[i]);
    // Eigen::Vector4d temp(maha * res);
    // increment= res'*temp/num_matches = temp'*M*temp/num_matches (we postpone 1/num_matches after the loop closes)
    // double ret = double(res.transpose() * temp);
	double ret = res.dot(maha*res);
	f_array[omp_get_thread_num()] += ret;
  }
  f = std::accumulate(f_array.begin(), f_array.end(), 0.0);
  return f/m;
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pclomp::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::OptimizationFunctorWithIndices::df (const Vector6d& x, Vector6d& g)
{
  Eigen::Matrix4f transformation_matrix = gicp_->base_transformation_;
  gicp_->applyState(transformation_matrix, x);
  //Eigen::Vector3d g_t = g.head<3> ();
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> R_array(omp_get_max_threads());
  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> g_array(omp_get_max_threads());

  for (std::size_t i = 0; i < R_array.size(); i++) {
	  R_array[i].setZero();
	  g_array[i].setZero();
  }

  int m = static_cast<int> (gicp_->tmp_idx_src_->size ());
  #pragma omp parallel for
  for(int i = 0; i < m; ++i)
  {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    pcl::Vector4fMapConst p_src = gicp_->tmp_src_->points[(*gicp_->tmp_idx_src_)[i]].getVector4fMap();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    pcl::Vector4fMapConst p_tgt = gicp_->tmp_tgt_->points[(*gicp_->tmp_idx_tgt_)[i]].getVector4fMap();

    Eigen::Vector4f pp(transformation_matrix * p_src);
    // The last coordinate is still guaranteed to be set to 1.0
    Eigen::Vector4d res(pp[0] - p_tgt[0], pp[1] - p_tgt[1], pp[2] - p_tgt[2], 0.0);
    // temp = M*res

	Eigen::Matrix4d maha = gicp_->mahalanobis((*gicp_->tmp_idx_src_)[i]).template cast<double>();

    Eigen::Vector4d temp(maha * res);
    // Increment translation gradient
    // g.head<3> ()+= 2*M*res/num_matches (we postpone 2/num_matches after the loop closes)
    // Increment rotation gradient
    pp = gicp_->base_transformation_ * p_src;

    Eigen::Vector4d p_src3(pp[0], pp[1], pp[2], 0.0);
    g_array[omp_get_thread_num()] += temp;
    R_array[omp_get_thread_num()] += p_src3 * temp.transpose();
  }

  g.setZero();
  Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
  for (std::size_t i = 0; i < R_array.size(); i++) {
	  R += R_array[i];
	  g.head<3>() += g_array[i].head<3>();
  }

  g.head<3>() *= 2.0 / m;
  R *= 2.0 / m;

  gicp_->computeRDerivative(x, R.block<3, 3>(0, 0), g);
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pclomp::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::OptimizationFunctorWithIndices::fdf (const Vector6d& x, double& f, Vector6d& g)
{
  Eigen::Matrix4f transformation_matrix = gicp_->base_transformation_;
  gicp_->applyState(transformation_matrix, x);
  f = 0;
  g.setZero ();
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero ();
  const auto m = static_cast<int> (gicp_->tmp_idx_src_->size ());
  for (int i = 0; i < m; ++i)
  {
    // The last coordinate, p_src[3] is guaranteed to be set to 1.0 in registration.hpp
    pcl::Vector4fMapConst p_src = gicp_->tmp_src_->points[(*gicp_->tmp_idx_src_)[i]].getVector4fMap ();
    // The last coordinate, p_tgt[3] is guaranteed to be set to 1.0 in registration.hpp
    pcl::Vector4fMapConst p_tgt = gicp_->tmp_tgt_->points[(*gicp_->tmp_idx_tgt_)[i]].getVector4fMap ();
    Eigen::Vector4f pp (transformation_matrix * p_src);
    // The last coordinate is still guaranteed to be set to 1.0
    Eigen::Vector3d res (pp[0] - p_tgt[0], pp[1] - p_tgt[1], pp[2] - p_tgt[2]);
    // temp = M*res
    Eigen::Vector3d temp (gicp_->mahalanobis((*gicp_->tmp_idx_src_)[i]).template block<3, 3>(0, 0).template cast<double>() * res);
    // Increment total error
    f+= double(res.transpose() * temp);
    // Increment translation gradient
    // g.head<3> ()+= 2*M*res/num_matches (we postpone 2/num_matches after the loop closes)
    g.head<3> ()+= temp;
    pp = gicp_->base_transformation_ * p_src;
    Eigen::Vector3d p_src3 (pp[0], pp[1], pp[2]);
    // Increment rotation gradient
    R+= p_src3 * temp.transpose();
  }
  f/= double(m);
  g.head<3> ()*= double(2.0/m);
  R*= 2.0/m;
  gicp_->computeRDerivative(x, R, g);
}

////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget> inline void
pclomp::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::computeTransformation (PointCloudSource &output, const Eigen::Matrix4f& guess)
{
  pcl::IterativeClosestPoint<PointSource, PointTarget>::initComputeReciprocal ();
  using namespace std;
  // Difference between consecutive transforms
  double delta = 0;
  // Get the size of the target
  const size_t N = indices_->size ();
  // Set the mahalanobis matrices to identity
  mahalanobis_.resize (N, Eigen::Matrix4f::Identity ());
  // Compute target cloud covariance matrices
  if ((!target_covariances_) || (target_covariances_->empty ()))
  {
    target_covariances_.reset (new MatricesVector);
    computeCovariances<PointTarget> (target_, tree_, *target_covariances_);
  }
  // Compute input cloud covariance matrices
  if ((!input_covariances_) || (input_covariances_->empty ()))
  {
    input_covariances_.reset (new MatricesVector);
    computeCovariances<PointSource> (input_, tree_reciprocal_, *input_covariances_);
  }

  base_transformation_ = Eigen::Matrix4f::Identity();
  nr_iterations_ = 0;
  converged_ = false;
  double dist_threshold = corr_dist_threshold_ * corr_dist_threshold_;
  pcl::transformPointCloud(output, output, guess);

  std::vector<std::vector<int>> nn_indices_array(omp_get_max_threads());
  std::vector<std::vector<float>> nn_dists_array(omp_get_max_threads());
  for (auto& nn_indices : nn_indices_array) { nn_indices.resize(1); }
  for (auto& nn_dists : nn_dists_array) { nn_dists.resize(1); }

  while(!converged_)
  {
    std::atomic<size_t> cnt;
    cnt = 0;
    std::vector<int> source_indices (indices_->size ());
    std::vector<int> target_indices (indices_->size ());

    // guess corresponds to base_t and transformation_ to t
    Eigen::Matrix4d transform_R = Eigen::Matrix4d::Zero ();
    for(size_t i = 0; i < 4; i++)
      for(size_t j = 0; j < 4; j++)
        for(size_t k = 0; k < 4; k++)
          transform_R(i,j)+= double(transformation_(i,k)) * double(guess(k,j));

    const Eigen::Matrix3d R = transform_R.topLeftCorner<3,3> ();

    #pragma omp parallel for
    for (std::size_t i = 0; i < N; i++)
    {
      auto& nn_indices = nn_indices_array[omp_get_thread_num()];
      auto& nn_dists = nn_dists_array[omp_get_thread_num()];

      PointSource query = output[i];
      query.getVector4fMap () = transformation_ * query.getVector4fMap ();

      if (!searchForNeighbors (query, nn_indices, nn_dists))
      {
        PCL_ERROR ("[pcl::%s::computeTransformation] Unable to find a nearest neighbor in the target dataset for point %d in the source!\n", getClassName ().c_str (), (*indices_)[i]);
        continue;
      }

      // Check if the distance to the nearest neighbor is smaller than the user imposed threshold
      if (nn_dists[0] < dist_threshold)
      {
        const Eigen::Matrix3d &C1 = (*input_covariances_)[i];
        const Eigen::Matrix3d &C2 = (*target_covariances_)[nn_indices[0]];
        Eigen::Matrix4f& M_ = mahalanobis_[i];
		M_.setZero();

		Eigen::Matrix3d M = M_.block<3, 3>(0, 0).cast<double>();
        // M = R*C1
        M = R * C1;
        // temp = M*R' + C2 = R*C1*R' + C2
        Eigen::Matrix3d temp = M * R.transpose();
        temp+= C2;
        // M = temp^-1
        M = temp.inverse ();
		M_.block<3, 3>(0, 0) = M.cast<float>();
        int c = cnt++;
        source_indices[c] = static_cast<int> (i);
        target_indices[c] = nn_indices[0];
      }
    }
    // Resize to the actual number of valid correspondences
    source_indices.resize(cnt); target_indices.resize(cnt);

    std::vector<std::pair<int, int>> indices(source_indices.size());
    for(std::size_t i = 0; i<source_indices.size(); i++) {
      indices[i].first = source_indices[i];
      indices[i].second = target_indices[i];
    }

    std::sort(indices.begin(), indices.end(), [=](const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) { return lhs.first < rhs.first; });

    for(std::size_t i = 0; i < source_indices.size(); i++) {
      source_indices[i] = indices[i].first;
      target_indices[i] = indices[i].second;
    }

    /* optimize transformation using the current assignment and Mahalanobis metrics*/
    previous_transformation_ = transformation_;
    //optimization right here
    try
    {
      rigid_transformation_estimation_(output, source_indices, *target_, target_indices, transformation_);
      /* compute the delta from this iteration */
      delta = 0.;
      for(int k = 0; k < 4; k++) {
        for(int l = 0; l < 4; l++) {
          double ratio = 1;
          if(k < 3 && l < 3) // rotation part of the transform
            ratio = 1./rotation_epsilon_;
          else
            ratio = 1./transformation_epsilon_;
          double c_delta = ratio*std::abs(previous_transformation_(k,l) - transformation_(k,l));
          if(c_delta > delta)
            delta = c_delta;
        }
      }
    }
    catch (pcl::PCLException &e)
    {
      PCL_DEBUG ("[pcl::%s::computeTransformation] Optimization issue %s\n", getClassName ().c_str (), e.what ());
      break;
    }
    nr_iterations_++;
    // Check for convergence
    if (nr_iterations_ >= max_iterations_ || delta < 1)
    {
      converged_ = true;
      previous_transformation_ = transformation_;
      PCL_DEBUG ("[pcl::%s::computeTransformation] Convergence reached. Number of iterations: %d out of %d. Transformation difference: %f\n",
                 getClassName ().c_str (), nr_iterations_, max_iterations_, (transformation_ - previous_transformation_).array ().abs ().sum ());
    }
    else
      PCL_DEBUG ("[pcl::%s::computeTransformation] Convergence failed\n", getClassName ().c_str ());
  }
  final_transformation_ = previous_transformation_ * guess;

  // Transform the point cloud
  pcl::transformPointCloud (*input_, output, final_transformation_);
}

template <typename PointSource, typename PointTarget> void
pclomp::GeneralizedIterativeClosestPoint<PointSource, PointTarget>::applyState(Eigen::Matrix4f &t, const Vector6d& x) const
{
  // !!! CAUTION Stanford GICP uses the Z Y X euler angles convention
  Eigen::Matrix3f R;
  R = Eigen::AngleAxisf (static_cast<float> (x[5]), Eigen::Vector3f::UnitZ ())
    * Eigen::AngleAxisf (static_cast<float> (x[4]), Eigen::Vector3f::UnitY ())
    * Eigen::AngleAxisf (static_cast<float> (x[3]), Eigen::Vector3f::UnitX ());
  t.topLeftCorner<3,3> ().matrix () = R * t.topLeftCorner<3,3> ().matrix ();
  Eigen::Vector4f T (static_cast<float> (x[0]), static_cast<float> (x[1]), static_cast<float> (x[2]), 0.0f);
  t.col (3) += T;
}

#endif //PCL_REGISTRATION_IMPL_GICP_HPP_
