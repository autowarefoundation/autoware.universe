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
#include "../../src/nerf/localizer.hpp"
#include "../../src/nerf/utils.hpp"
#include "main_functions.hpp"

#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// Disable checking because of too many give errors
// cspell:disable

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

torch::Tensor calc_rotation_tensor(float degree, Eigen::Vector3f axis)
{
  const float theta = degree * M_PI / 180.0;
  Eigen::AngleAxisf a(theta, axis);
  Eigen::Matrix3f rotation_matrix(a);
  torch::Tensor result =
    torch::from_blob(rotation_matrix.data(), {3, 3}).to(torch::kFloat32).cuda();
  return result;
}

void render(Localizer & localizer, torch::Tensor pose)
{
  std::cout << "pose:\n" << pose << std::endl;
  torch::Tensor pose_camera = localizer.camera2nerf(pose);
  torch::Tensor image = localizer.render_image(pose_camera);
  utils::write_image_tensor("image.png", image);
  std::cout << "Move by WASD, E:Up, Q:Down, J:YAW+, K:PITCH+, L:YAW-, I:PITCH-, O:ROll+, U:ROll-"
            << std::endl;
}

void walk(const std::string & train_result_dir)
{
  torch::NoGradGuard no_grad_guard;
  LocalizerParam param;
  param.train_result_dir = train_result_dir;
  param.resize_factor = 8;
  Localizer localizer(param);
  torch::Tensor pose = torch::eye(4).cuda();
  float step = 0.2;
  constexpr float degree = 10.0;

  render(localizer, pose);

  while (1) {
    if (kbhit()) {
      const char pushed_key = getchar();
      printf("pushed '%c'\n", pushed_key);
      torch::Tensor orientation = pose.index({Slc(0, 3), Slc(0, 3)});
      if (pushed_key == 'w') {
        torch::Tensor tmp = torch::tensor({step, 0.0f, 0.0f}, torch::kFloat32).view({3, 1}).cuda();
        pose.index({Slc(0, 3), Slc(3, 4)}) += orientation.matmul(tmp);
      } else if (pushed_key == 'a') {
        torch::Tensor tmp = torch::tensor({0.0f, step, 0.0f}, torch::kFloat32).view({3, 1}).cuda();
        pose.index({Slc(0, 3), Slc(3, 4)}) += orientation.matmul(tmp);
      } else if (pushed_key == 'd') {
        torch::Tensor tmp = torch::tensor({0.0f, -step, 0.0f}, torch::kFloat32).view({3, 1}).cuda();
        pose.index({Slc(0, 3), Slc(3, 4)}) += orientation.matmul(tmp);
      } else if (pushed_key == 's') {
        torch::Tensor tmp = torch::tensor({-step, 0.0f, 0.0f}, torch::kFloat32).view({3, 1}).cuda();
        pose.index({Slc(0, 3), Slc(3, 4)}) += orientation.matmul(tmp);
      } else if (pushed_key == 'e') {
        torch::Tensor tmp = torch::tensor({0.0f, 0.0f, step}, torch::kFloat32).view({3, 1}).cuda();
        pose.index({Slc(0, 3), Slc(3, 4)}) += orientation.matmul(tmp);
      } else if (pushed_key == 'q') {
        torch::Tensor tmp = torch::tensor({0.0f, 0.0f, -step}, torch::kFloat32).view({3, 1}).cuda();
        pose.index({Slc(0, 3), Slc(3, 4)}) += orientation.matmul(tmp);
      } else if (pushed_key == 'j') {
        torch::Tensor rotation_matrix = calc_rotation_tensor(-degree, Eigen::Vector3f::UnitZ());
        orientation = torch::matmul(orientation, rotation_matrix);
        pose.index({Slc(0, 3), Slc(0, 3)}) = orientation;
      } else if (pushed_key == 'k') {
        torch::Tensor rotation_matrix = calc_rotation_tensor(-degree, Eigen::Vector3f::UnitY());
        orientation = torch::matmul(orientation, rotation_matrix);
        pose.index({Slc(0, 3), Slc(0, 3)}) = orientation;
      } else if (pushed_key == 'l') {
        torch::Tensor rotation_matrix = calc_rotation_tensor(+degree, Eigen::Vector3f::UnitZ());
        orientation = torch::matmul(orientation, rotation_matrix);
        pose.index({Slc(0, 3), Slc(0, 3)}) = orientation;
      } else if (pushed_key == 'i') {
        torch::Tensor rotation_matrix = calc_rotation_tensor(+degree, Eigen::Vector3f::UnitY());
        orientation = torch::matmul(orientation, rotation_matrix);
        pose.index({Slc(0, 3), Slc(0, 3)}) = orientation;
      } else if (pushed_key == 'o') {
        torch::Tensor rotation_matrix = calc_rotation_tensor(+degree, Eigen::Vector3f::UnitX());
        orientation = torch::matmul(orientation, rotation_matrix);
        pose.index({Slc(0, 3), Slc(0, 3)}) = orientation;
      } else if (pushed_key == 'u') {
        torch::Tensor rotation_matrix = calc_rotation_tensor(-degree, Eigen::Vector3f::UnitX());
        orientation = torch::matmul(orientation, rotation_matrix);
        pose.index({Slc(0, 3), Slc(0, 3)}) = orientation;
      } else if (pushed_key == 'x') {
        step += 0.1;
        std::cout << "step = " << step << std::endl;
      } else if (pushed_key == 'z') {
        step -= 0.1;
        std::cout << "step = " << step << std::endl;
      } else {
        std::cout << "Unknown kye: " << pushed_key << std::endl;
        continue;
      }
      render(localizer, pose);
    }
  }
}
