/*
 * Copyright 2020 TierIV. All rights reserved.
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

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include <boost/filesystem.hpp>

#include <rclcpp/rclcpp.hpp>

#include <map_loader/pointcloud_map_loader_node.h>

namespace fs = boost::filesystem;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::vector<std::string> pcd_paths;
  for (int i = 1; i < argc; ++i) {
    const fs::path arg(argv[i]);

    if (!fs::exists(arg)) {
      const std::string msg = "invalid path: " + arg.string();
      std::cerr << msg;
    }

    if (fs::is_regular_file(arg)) {
      pcd_paths.push_back(argv[i]);
    }

    if (fs::is_directory(arg)) {
      for (const auto & f : fs::directory_iterator(arg)) {
        const auto & p = f.path();

        if (!fs::is_regular_file(p)) {
          continue;
        }

        if (p.extension() != ".pcd" && p.extension() != ".PCD") {
          continue;
        }

        pcd_paths.push_back(p.string());
      }
    }
  }

  if(pcd_paths.empty())
  {
    const std::string msg = "no valid_path";
    throw std::runtime_error(msg);
  }

  const auto pointcloud_map_loader_node = std::make_shared<PointCloudMapLoaderNode>(pcd_paths);
  rclcpp::spin(pointcloud_map_loader_node);
  rclcpp::shutdown();

  return 0;
}
