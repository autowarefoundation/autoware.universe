// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <vector>

#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "rclcpp_components/node_factory_template.hpp"

#define NODE_MAIN_LOGGER_NAME "pointcloud_accumulator_node"

int main(int argc, char * argv[])
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  rclcpp::Logger logger = rclcpp::get_logger(NODE_MAIN_LOGGER_NAME);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.arguments(args);
  std::vector<class_loader::ClassLoader * > loaders;
  std::vector<rclcpp_components::NodeInstanceWrapper> node_wrappers;

  std::string library_name = "libpointcloud_preprocessor_filter.so";
  std::string class_name = "rclcpp_components::NodeFactoryTemplate<pointcloud_preprocessor::PointcloudAccumulatorComponent>";

  RCLCPP_DEBUG(logger, "Load library %s", library_name.c_str());
  auto loader = new class_loader::ClassLoader(library_name);
  auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
  for (auto clazz : classes) {
    std::string name = clazz.c_str();
    if (!(name.compare(class_name))) {
      RCLCPP_DEBUG(logger, "Instantiate class %s", clazz.c_str());
      auto node_factory = loader->createInstance<rclcpp_components::NodeFactory>(clazz);
      auto wrapper = node_factory->create_node_instance(options);
      auto node = wrapper.get_node_base_interface();
      node_wrappers.push_back(wrapper);
      exec.add_node(node);
    }
  }
  loaders.push_back(loader);


  exec.spin();

  for (auto wrapper : node_wrappers) {
    exec.remove_node(wrapper.get_node_base_interface());
  }
  node_wrappers.clear();

  rclcpp::shutdown();

  return 0;
}
