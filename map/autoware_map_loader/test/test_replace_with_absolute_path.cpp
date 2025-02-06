// Copyright 2022 The Autoware Contributors
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

#include "../src/pointcloud_map_loader/utils.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <map>
#include <set>
#include <string>
#include <vector>

using autoware::map_loader::PCDFileMetadata;
using autoware::map_loader::replace_with_absolute_path;
using ::testing::ContainerEq;

TEST(ReplaceWithAbsolutePathTest, BasicFunctionality)
{
  std::map<std::string, PCDFileMetadata> pcd_metadata_path = {
    {"file1.pcd", {{1, 2, 3}, {4, 5, 6}}},
    {"file2.pcd", {{-1, -2, -3}, {-4, -5, -6}}},
  };

  std::vector<std::string> pcd_paths = {
    "/home/user/pcd/file1.pcd",
    "/home/user/pcd/file2.pcd",
  };

  std::map<std::string, PCDFileMetadata> expected = {
    {"/home/user/pcd/file1.pcd", {{1, 2, 3}, {4, 5, 6}}},
    {"/home/user/pcd/file2.pcd", {{-1, -2, -3}, {-4, -5, -6}}},
  };

  std::set<std::string> missing_pcd_names;
  auto result = replace_with_absolute_path(pcd_metadata_path, pcd_paths, missing_pcd_names);
  ASSERT_THAT(result, ContainerEq(expected));
}

TEST(ReplaceWithAbsolutePathTest, NoMatchingFiles)
{
  std::map<std::string, PCDFileMetadata> pcd_metadata_path = {
    {"file1.pcd", {{1, 2, 3}, {4, 5, 6}}},
    {"file2.pcd", {{-1, -2, -3}, {-4, -5, -6}}},
  };

  std::vector<std::string> pcd_paths = {
    "/home/user/pcd/non_matching_file1.pcd",
    "/home/user/pcd/non_matching_file2.pcd",
  };

  std::map<std::string, PCDFileMetadata> expected = {};
  std::set<std::string> missing_pcd_names;

  auto result = replace_with_absolute_path(pcd_metadata_path, pcd_paths, missing_pcd_names);
  ASSERT_THAT(result, ContainerEq(expected));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
