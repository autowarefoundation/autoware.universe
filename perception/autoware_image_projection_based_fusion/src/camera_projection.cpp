// Copyright 2024 TIER IV, Inc.
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

#include "autoware/image_projection_based_fusion/camera_projection.hpp"

namespace autoware::image_projection_based_fusion
{
CameraProjection::CameraProjection(
  const sensor_msgs::msg::CameraInfo & camera_info, const float grid_width, const float grid_height,
  const bool unrectify, const bool use_approximation = false)
: camera_info_(camera_info),
  grid_w_size_(grid_width),
  grid_h_size_(grid_height),
  unrectify_(unrectify),
  use_approximation_(use_approximation)
{
  if (grid_w_size_ <= 0.0 || grid_h_size_ <= 0.0) {
    throw std::runtime_error("Both grid_width and grid_height must be > 0.0");
  }

  image_w_ = camera_info.width;
  image_h_ = camera_info.height;

  // prepare camera model
  camera_model_.fromCameraInfo(camera_info);

  // cache settings
  // for shifting to the grid center
  half_grid_w_size_ = grid_w_size_ / 2.0;
  half_grid_h_size_ = grid_h_size_ / 2.0;
  inv_grid_w_size_ = 1 / grid_w_size_;
  inv_grid_h_size_ = 1 / grid_h_size_;
  grid_x_num_ = static_cast<uint32_t>(std::ceil(image_w_ / grid_w_size_));
  grid_y_num_ = static_cast<uint32_t>(std::ceil(image_h_ / grid_h_size_));
  cache_size_ = grid_x_num_ * grid_y_num_;

  // compute 3D rays for the image corners and pixels related to optical center
  // to find the actual FOV size
  // optical centers
  const double ocx = static_cast<double>(camera_info.k.at(2));
  const double ocy = static_cast<double>(camera_info.k.at(5));
  // for checking pincushion shape case
  const cv::Point3d ray_top_left = camera_model_.projectPixelTo3dRay(cv::Point(0, 0));
  const cv::Point3d ray_top_right = camera_model_.projectPixelTo3dRay(cv::Point(image_w_ - 1, 0));
  const cv::Point3d ray_bottom_left = camera_model_.projectPixelTo3dRay(cv::Point(0, image_h_ - 1));
  const cv::Point3d ray_bottom_right =
    camera_model_.projectPixelTo3dRay(cv::Point(image_w_ - 1, image_h_ - 1));
  // for checking barrel shape case
  const cv::Point3d ray_mid_top = camera_model_.projectPixelTo3dRay(cv::Point(ocx, 0));
  const cv::Point3d ray_mid_left = camera_model_.projectPixelTo3dRay(cv::Point(0, ocy));
  const cv::Point3d ray_mid_right = camera_model_.projectPixelTo3dRay(cv::Point(image_w_ - 1, ocy));
  const cv::Point3d ray_mid_bottom =
    camera_model_.projectPixelTo3dRay(cv::Point(ocx, image_h_ - 1));

  cv::Point3d x_left = ray_top_left;
  cv::Point3d x_right = ray_top_right;
  cv::Point3d y_top = ray_top_left;
  cv::Point3d y_bottom = ray_bottom_left;

  // check each side of the view
  if (ray_bottom_left.x < x_left.x) x_left = ray_bottom_left;
  if (ray_mid_left.x < x_left.x) x_left = ray_mid_left;

  if (x_right.x < ray_bottom_right.x) x_right = ray_bottom_right;
  if (x_right.x < ray_mid_right.x) x_right = ray_mid_right;

  if (y_top.y < ray_top_right.y) y_top = ray_top_right;
  if (y_top.y < ray_mid_top.y) y_top = ray_mid_top;

  if (ray_bottom_left.y < y_bottom.y) y_bottom = ray_bottom_left;
  if (ray_mid_bottom.y < y_bottom.y) y_bottom = ray_mid_bottom;

  // set FOV at z = 1.0
  fov_left_ = x_left.x / x_left.z;
  fov_right_ = x_right.x / x_right.z;
  fov_top_ = y_top.y / y_top.z;
  fov_bottom_ = y_bottom.y / y_bottom.z;
}

void CameraProjection::initialize()
{
  if (unrectify_) {
    if (use_approximation_) {
      // create the cache with size of grid center
      // store only xy position in float to reduce memory consumption
      projection_cache_ = std::make_unique<PixelPos[]>(cache_size_);
      initializeCache();

      calcImageProjectedPoint = [this](
                                  const cv::Point3d & point3d, Eigen::Vector2d & projected_point) {
        return this->calcRawImageProjectedPointWithApproximation(point3d, projected_point);
      };
    } else {
      calcImageProjectedPoint = [this](
                                  const cv::Point3d & point3d, Eigen::Vector2d & projected_point) {
        return this->calcRawImageProjectedPoint(point3d, projected_point);
      };
    }
  } else {
    calcImageProjectedPoint = [this](
                                const cv::Point3d & point3d, Eigen::Vector2d & projected_point) {
      return this->calcRectifiedImageProjectedPoint(point3d, projected_point);
    };
  }
}

void CameraProjection::initializeCache()
{
  // sample grid centers till the camera height, width to precompute the projection
  //
  //      grid_size
  //      /
  //     v
  //   |---|          w
  //  0----------------->
  // 0 | . | . | . |
  //   |___|___|___|
  //   | . | . | . |
  //   | ^
  // h | |
  //   v grid center
  //
  // each pixel will be rounded in this grid center
  // edge pixels in the image will be assign to centers that is the outside of the image

  for (float y = half_grid_h_size_; y < image_h_; y += grid_h_size_) {
    for (float x = half_grid_w_size_; x < image_w_; x += grid_w_size_) {
      const float qx =
        std::round((x - half_grid_w_size_) * inv_grid_w_size_) * grid_w_size_ + half_grid_w_size_;
      const float qy =
        std::round((y - half_grid_h_size_) * inv_grid_h_size_) * grid_h_size_ + half_grid_h_size_;

      const int grid_x = static_cast<int>(std::floor(qx / grid_w_size_));
      const int grid_y = static_cast<int>(std::floor(qy / grid_h_size_));
      const int index = (grid_y)*grid_x_num_ + grid_x;

      // precompute projected point
      cv::Point2d raw_image_point = camera_model_.unrectifyPoint(cv::Point2d(qx, qy));
      projection_cache_[index] =
        PixelPos{static_cast<float>(raw_image_point.x), static_cast<float>(raw_image_point.y)};
    }
  }
}

/**
 * @brief Calculate a projection of 3D point to rectified image plane 2D point.
 * @return Return a boolean indicating whether the projected point is on the image plane.
 */
bool CameraProjection::calcRectifiedImageProjectedPoint(
  const cv::Point3d & point3d, Eigen::Vector2d & projected_point)
{
  const cv::Point2d rectified_image_point = camera_model_.project3dToPixel(point3d);

  if (
    rectified_image_point.x < 0.0 || rectified_image_point.x >= image_w_ ||
    rectified_image_point.y < 0.0 || rectified_image_point.y >= image_h_) {
    return false;
  } else {
    projected_point << rectified_image_point.x, rectified_image_point.y;
    return true;
  }
}

/**
 * @brief Calculate a projection of 3D point to raw image plane 2D point.
 * @return Return a boolean indicating whether the projected point is on the image plane.
 */
bool CameraProjection::calcRawImageProjectedPoint(
  const cv::Point3d & point3d, Eigen::Vector2d & projected_point)
{
  const cv::Point2d rectified_image_point = camera_model_.project3dToPixel(point3d);
  const cv::Point2d raw_image_point = camera_model_.unrectifyPoint(rectified_image_point);

  if (
    rectified_image_point.x < 0.0 || rectified_image_point.x >= image_w_ ||
    rectified_image_point.y < 0.0 || rectified_image_point.y >= image_h_) {
    return false;
  } else {
    projected_point << raw_image_point.x, raw_image_point.y;
    return true;
  }
}

/**
 * @brief Calculate a projection of 3D point to raw image plane 2D point with approximation.
 * @return Return a boolean indicating whether the projected point is on the image plane.
 */
bool CameraProjection::calcRawImageProjectedPointWithApproximation(
  const cv::Point3d & point3d, Eigen::Vector2d & projected_point)
{
  const cv::Point2d rectified_image_point = camera_model_.project3dToPixel(point3d);

  // round to a near grid center
  const float qx =
    std::round((rectified_image_point.x - half_grid_w_size_) * inv_grid_w_size_) * grid_w_size_ +
    half_grid_w_size_;
  if (qx < 0.0 || qx >= image_w_) {
    return false;
  }
  const float qy =
    std::round((rectified_image_point.y - half_grid_h_size_) * inv_grid_h_size_) * grid_h_size_ +
    half_grid_h_size_;
  if (qy < 0.0 || qy >= image_h_) {
    return false;
  }

  const int grid_x = static_cast<int>(std::floor(qx / grid_w_size_));
  const int grid_y = static_cast<int>(std::floor(qy / grid_h_size_));
  const int index = grid_y * grid_x_num_ + grid_x;

  projected_point << projection_cache_[index].x, projection_cache_[index].y;

  return true;
}

sensor_msgs::msg::CameraInfo CameraProjection::getCameraInfo()
{
  return camera_info_;
}

bool CameraProjection::isOutsideHorizontalView(const float px, const float pz)
{
  // assuming the points' origin is centered on the camera
  if (pz <= 0.0) return true;
  if (px < fov_left_ * pz) return true;
  if (px > fov_right_ * pz) return true;

  // inside of the horizontal view
  return false;
}

bool CameraProjection::isOutsideVerticalView(const float py, const float pz)
{
  // assuming the points' origin is centered on the camera
  if (pz <= 0.0) return true;
  // up in the camera coordinate is negative
  if (py < fov_top_ * pz) return true;
  if (py > fov_bottom_ * pz) return true;

  // inside of the vertical view
  return false;
}

bool CameraProjection::isOutsideFOV(const float px, const float py, const float pz)
{
  if (isOutsideHorizontalView(px, pz)) return true;
  if (isOutsideVerticalView(py, pz)) return true;

  // inside of the FOV
  return false;
}

}  // namespace autoware::image_projection_based_fusion
