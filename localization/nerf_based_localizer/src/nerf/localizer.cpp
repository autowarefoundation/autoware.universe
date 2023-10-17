// Copyright 2023 Autoware Foundation
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
#include "localizer.hpp"

#include "dataset.hpp"
#include "stop_watch.hpp"

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <random>

using Tensor = torch::Tensor;

Localizer::Localizer(const LocalizerParam & param) : param_(param)
{
  const std::string train_result_dir = param.train_result_dir;

  cv::FileStorage inference_params(
    train_result_dir + "/inference_params.yaml", cv::FileStorage::READ);
  if (!inference_params.isOpened()) {
    throw std::runtime_error("Failed to open " + train_result_dir + "/inference_params.yaml");
  }

  const int n_images = (int)inference_params["n_images"];
  const int train_height = (int)inference_params["height"];
  const int train_width = (int)inference_params["width"];

  std::vector<float> intrinsic_vector;
  inference_params["intrinsic"] >> intrinsic_vector;
  intrinsic_ = torch::tensor(intrinsic_vector, torch::kFloat).view({3, 3}).to(torch::kCUDA);

  std::vector<float> normalizing_center;
  inference_params["normalizing_center"] >> normalizing_center;
  center_ = torch::tensor(normalizing_center, torch::kFloat).to(torch::kCUDA);

  radius_ = (float)inference_params["normalizing_radius"];

  renderer_ = std::make_shared<Renderer>(n_images, param.sample_num_per_ray);

  torch::load(renderer_, train_result_dir + "/checkpoints/latest/renderer.pt");

  // set
  infer_height_ = train_height / param.resize_factor;
  infer_width_ = train_width / param.resize_factor;
  intrinsic_ /= param.resize_factor;
  intrinsic_[2][2] = 1.0;

  /*
  [[+1,  0,  0,  0 ],
  [  0, -1,  0,  0 ],
  [  0,  0, -1,  0 ],
  [  0,  0,  0, +1 ]]
*/
  axis_convert_mat_ = torch::zeros({4, 4});
  axis_convert_mat_[0][0] = +1;
  axis_convert_mat_[1][1] = -1;
  axis_convert_mat_[2][2] = -1;
  axis_convert_mat_[3][3] = +1;
  axis_convert_mat_ = axis_convert_mat_.to(torch::kCUDA);
}

std::vector<Particle> Localizer::optimize_pose_by_random_search(
  Tensor initial_pose, Tensor image_tensor, int64_t particle_num, float noise_coeff)
{
  torch::NoGradGuard no_grad_guard;

  std::mt19937_64 engine(std::random_device{}());

  // Note that the order of the axes is different
  // World coordinates (x: Front, y: Left, z: Up)
  // NeRF coordinates (x: Right, y: Up, z: Back)
  const float pos_noise_x_in_nerf = param_.noise_position_y * noise_coeff / radius_;
  const float pos_noise_y_in_nerf = param_.noise_position_z * noise_coeff / radius_;
  const float pos_noise_z_in_nerf = param_.noise_position_x * noise_coeff / radius_;
  const float theta_x_in_nerf = param_.noise_rotation_y * noise_coeff;
  const float theta_y_in_nerf = param_.noise_rotation_z * noise_coeff;
  const float theta_z_in_nerf = param_.noise_rotation_x * noise_coeff;

  std::normal_distribution<float> dist_position_x(0.0f, pos_noise_x_in_nerf);
  std::normal_distribution<float> dist_position_y(0.0f, pos_noise_y_in_nerf);
  std::normal_distribution<float> dist_position_z(0.0f, pos_noise_z_in_nerf);
  std::normal_distribution<float> dist_rotation_x(0.0f, theta_x_in_nerf);
  std::normal_distribution<float> dist_rotation_y(0.0f, theta_y_in_nerf);
  std::normal_distribution<float> dist_rotation_z(0.0f, theta_z_in_nerf);

  std::vector<Tensor> poses;
  for (int64_t i = 0; i < particle_num; i++) {
    // Sample a random translation
    Tensor curr_pose = initial_pose.clone();
    if (i == 0) {
      poses.push_back(curr_pose);
      continue;
    }
    curr_pose[0][3] += dist_position_x(engine);
    curr_pose[1][3] += dist_position_y(engine);
    curr_pose[2][3] += dist_position_z(engine);

    // orientation
    const float theta_x = dist_rotation_x(engine) * M_PI / 180.0;
    const float theta_y = dist_rotation_y(engine) * M_PI / 180.0;
    const float theta_z = dist_rotation_z(engine) * M_PI / 180.0;
    Eigen::Matrix3f rotation_matrix_x(Eigen::AngleAxisf(theta_x, Eigen::Vector3f::UnitX()));
    Eigen::Matrix3f rotation_matrix_y(Eigen::AngleAxisf(theta_y, Eigen::Vector3f::UnitY()));
    Eigen::Matrix3f rotation_matrix_z(Eigen::AngleAxisf(theta_z, Eigen::Vector3f::UnitZ()));
    const torch::Device dev = initial_pose.device();
    Tensor rotation_tensor_x =
      torch::from_blob(rotation_matrix_x.data(), {3, 3}).to(torch::kFloat32).to(dev);
    Tensor rotation_tensor_y =
      torch::from_blob(rotation_matrix_y.data(), {3, 3}).to(torch::kFloat32).to(dev);
    Tensor rotation_tensor_z =
      torch::from_blob(rotation_matrix_z.data(), {3, 3}).to(torch::kFloat32).to(dev);
    Tensor rotated = rotation_tensor_z.mm(
      rotation_tensor_y.mm(rotation_tensor_x.mm(curr_pose.index({Slc(0, 3), Slc(0, 3)}))));
    curr_pose.index_put_({Slc(0, 3), Slc(0, 3)}, rotated);
    poses.push_back(curr_pose);
  }

  const std::vector<float> weights = evaluate_poses(poses, image_tensor);
  const int pose_num = poses.size();

  std::vector<Particle> result;
  for (int i = 0; i < pose_num; i++) {
    result.push_back({poses[i], weights[i]});
  }
  return result;
}

torch::Tensor gram_schmidt(torch::Tensor A)
{
  A = A.clone();
  for (int i = 0; i < A.size(0); ++i) {
    for (int j = 0; j < i; ++j) {
      A[i] -= torch::dot(A[j], A[i]) * A[j];
    }
    A[i] = A[i] / A[i].norm();
  }
  return A;
}

std::vector<Tensor> Localizer::optimize_pose_by_differential(
  Tensor initial_pose, Tensor image_tensor, int64_t iteration_num, float learning_rate)
{
  std::vector<Tensor> results;
  initial_pose = initial_pose.requires_grad_(true);
  image_tensor = image_tensor.view({infer_height_, infer_width_, 3});
  torch::optim::Adam optimizer({initial_pose}, learning_rate);
  for (int64_t i = 0; i < iteration_num; i++) {
    Tensor pred_img = render_image(initial_pose);
    Tensor loss = torch::nn::functional::mse_loss(pred_img, image_tensor);
    optimizer.zero_grad();
    // For some reason, backward may fail, so check here
    try {
      loss.backward();
    } catch (const std::runtime_error & e) {
      return results;
    }
    optimizer.step();

    Tensor curr_result = initial_pose.clone().detach();
    results.push_back(curr_result);
  }
  return results;
}

Tensor Localizer::render_image(const Tensor & pose)
{
  auto [image, _] =
    renderer_->render_image(pose, intrinsic_, infer_height_, infer_width_, (1 << 16));
  return image;
}

std::vector<float> Localizer::evaluate_poses(
  const std::vector<Tensor> & poses, const Tensor & image)
{
  torch::NoGradGuard no_grad_guard;
  Timer timer;

  const int pixel_num = param_.render_pixel_num;
  const auto CUDALong = torch::TensorOptions().dtype(torch::kLong).device(torch::kCUDA);

  // Pick rays by constant interval
  // const int step = H * W / pixel_num;
  // std::vector<int64_t> i_vec, j_vec;
  // for (int k = 0; k < pixel_num; k++) {
  //   const int v = k * step;
  //   const int64_t i = v / W;
  //   const int64_t j = v % W;
  //   i_vec.push_back(i);
  //   j_vec.push_back(j);
  // }
  // const Tensor i = torch::tensor(i_vec, CUDALong);
  // const Tensor j = torch::tensor(j_vec, CUDALong);

  // Pick rays by random sampling without replacement
  std::vector<int> indices(infer_height_ * infer_width_);
  std::iota(indices.begin(), indices.end(), 0);
  std::mt19937 engine(std::random_device{}());
  std::shuffle(indices.begin(), indices.end(), engine);
  std::vector<int64_t> i_vec, j_vec;
  for (int k = 0; k < pixel_num; k++) {
    const int v = indices[k];
    const int64_t i = v / infer_width_;
    const int64_t j = v % infer_width_;
    i_vec.push_back(i);
    j_vec.push_back(j);
  }
  Tensor i = torch::tensor(i_vec, CUDALong);
  Tensor j = torch::tensor(j_vec, CUDALong);

  // Pick rays by random sampling with replacement
  // const Tensor i = torch::randint(0, H, pixel_num, CUDALong);
  // const Tensor j = torch::randint(0, infer_width_, pixel_num, CUDALong);

  const Tensor ij = torch::stack({i, j}, -1).to(torch::kFloat32);
  std::vector<Tensor> rays_o_vec;
  std::vector<Tensor> rays_d_vec;
  for (const Tensor & pose : poses) {
    auto [rays_o, rays_d] = get_rays_from_pose(pose.unsqueeze(0), intrinsic_.unsqueeze(0), ij);
    rays_o_vec.push_back(rays_o);
    rays_d_vec.push_back(rays_d);
  }

  const int64_t pose_num = poses.size();
  const int64_t numel = pixel_num * pose_num;

  Tensor rays_o = torch::cat(rays_o_vec);  // (numel, 3)
  Tensor rays_d = torch::cat(rays_d_vec);  // (numel, 3)

  timer.start();
  auto [pred_colors, _] = renderer_->render_all_rays(rays_o, rays_d, (1 << 16));

  Tensor pred_pixels = pred_colors.view({pose_num, pixel_num, 3});
  pred_pixels = pred_pixels.clip(0.f, 1.f);
  pred_pixels = pred_pixels.to(image.device());  // (pose_num, pixel_num, 3)

  i = i.to(image.device());
  j = j.to(image.device());

  Tensor gt_pixels = image.index({i, j});              // (pixel_num, 3)
  Tensor diff = pred_pixels - gt_pixels;               // (pose_num, pixel_num, 3)
  Tensor loss = (diff * diff).mean(-1).sum(-1).cpu();  // (pose_num,)
  loss = pixel_num / (loss + 1e-6f);
  loss = torch::pow(loss, 5);
  loss /= loss.sum();

  std::vector<float> result(loss.data_ptr<float>(), loss.data_ptr<float>() + loss.numel());
  return result;
}

Eigen::Matrix3d compute_rotation_average(
  const std::vector<Eigen::Matrix3d> & rotations, const std::vector<double> & weights)
{
  // cf. https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
  std::vector<Eigen::Quaterniond> quaternions;
  for (const Eigen::Matrix3d & rot : rotations) {
    Eigen::Quaterniond quat(rot);
    quaternions.push_back(quat);
  }

  Eigen::Vector4d cumulative(0.0, 0.0, 0.0, 0.0);
  const Eigen::Quaterniond & front = quaternions[0];

  for (Eigen::Quaterniond & q : quaternions) {
    if (q.dot(front) < 0.0) {
      q = Eigen::Quaterniond(-q.coeffs());
    }
    cumulative += q.coeffs();
  }

  cumulative /= quaternions.size();

  Eigen::Quaterniond average_quaternion;
  average_quaternion.coeffs() = cumulative;
  average_quaternion.normalize();

  return average_quaternion.toRotationMatrix();
}

Tensor Localizer::calc_average_pose(const std::vector<Particle> & particles)
{
  torch::Device device = particles.front().pose.device();
  torch::Tensor avg_position_tensor = torch::zeros({3, 1}, device).to(torch::kFloat32);
  std::vector<Eigen::Matrix3d> rotations;
  std::vector<double> weights;

  for (const Particle & particle : particles) {
    torch::Tensor pose = particle.pose;
    torch::Tensor position = pose.index({Slc(0, 3), Slc(3, 4)});
    avg_position_tensor += position * particle.weight;

    // slice to get 3x3 rotation matrix, convert it to Eigen::Matrix3f
    torch::Tensor rotation_tensor = pose.index({Slc(0, 3), Slc(0, 3)}).to(torch::kDouble).cpu();
    Eigen::Matrix3d rotation;
    std::memcpy(
      rotation.data(), rotation_tensor.data_ptr(), sizeof(double) * rotation_tensor.numel());
    rotations.push_back(rotation);
    weights.push_back(particle.weight);
  }

  Eigen::Matrix3d avg_rotation_matrix = compute_rotation_average(rotations, weights);
  torch::Tensor avg_rotation_tensor = torch::from_blob(
    avg_rotation_matrix.data(), {3, 3}, torch::TensorOptions().dtype(torch::kDouble));
  avg_rotation_tensor = avg_rotation_tensor.to(torch::kFloat32);
  avg_rotation_tensor = avg_rotation_tensor.to(device);

  // combine average position and rotation to form average pose
  torch::Tensor avg_pose = torch::zeros_like(particles.front().pose);
  avg_pose.index_put_({Slc(0, 3), Slc(3, 4)}, avg_position_tensor);
  avg_pose.index_put_({Slc(0, 3), Slc(0, 3)}, avg_rotation_tensor);

  return avg_pose;
}

torch::Tensor Localizer::camera2nerf(const torch::Tensor & pose_in_world)
{
  torch::Tensor x = pose_in_world;
  x = torch::mm(x, axis_convert_mat_);
  x = torch::mm(axis_convert_mat_.t(), x);

  // normalize t
  Tensor t = x.index({Slc(0, 3), 3}).clone();
  t = (t - center_.unsqueeze(0)) / radius_;
  x.index_put_({Slc(0, 3), 3}, t);

  x = x.index({Slc(0, 3), Slc(0, 4)});
  return x;
}

torch::Tensor Localizer::nerf2camera(const torch::Tensor & pose_in_camera)
{
  torch::Tensor x = pose_in_camera;
  x = torch::cat({x, torch::tensor({0, 0, 0, 1}).view({1, 4}).to(torch::kCUDA)});

  // denormalize t
  Tensor t = x.index({Slc(0, 3), 3}).clone();
  t = t * radius_ + center_.unsqueeze(0);
  x.index_put_({Slc(0, 3), 3}, t);

  x = torch::mm(x, axis_convert_mat_.t());
  x = torch::mm(axis_convert_mat_, x);
  return x;
}
