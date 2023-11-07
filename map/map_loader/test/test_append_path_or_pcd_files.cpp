// Copyright 2023 The Autoware Contributors
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

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

void createDummyPcdFile(const std::string & path)
{
  std::ofstream file(path);
  file << "dummy content";
  file.close();
}

class AppendPathOrPcdFilesTest : public ::testing::Test
{
protected:
  std::string test_directory;

  void SetUp() override
  {
    // Create a temporary directory for testing
    test_directory = (std::filesystem::temp_directory_path() / "appendPathOrPcdFilesTest").string();
    std::filesystem::create_directory(test_directory);

    // Setup dummy files
    createDummyPcdFile(test_directory + "/test1.pcd");
    createDummyPcdFile(test_directory + "/test2.PCD");
    std::ofstream(test_directory + "/test.txt").close();  // Non-PCD file
  }

  void TearDown() override
  {
    // Remove the temporary directory
    std::filesystem::remove_all(test_directory);
  }
};

TEST_F(AppendPathOrPcdFilesTest, AppendSinglePcdFile)
{
  std::vector<std::string> path_list;
  std::string pcd_file_path = test_directory + "/test1";

  appendPathOrPcdFiles(pcd_file_path, path_list);

  ASSERT_EQ(path_list.size(), 1u);
  EXPECT_EQ(path_list[0], pcd_file_path + ".pcd");
}

TEST_F(AppendPathOrPcdFilesTest, AppendSinglePCDFileCaseInsensitive)
{
  std::vector<std::string> path_list;
  std::string pcd_file_path = test_directory + "/test2";

  appendPathOrPcdFiles(pcd_file_path, path_list);

  ASSERT_EQ(path_list.size(), 1u);
  EXPECT_EQ(path_list[0], pcd_file_path + ".PCD");
}

TEST_F(AppendPathOrPcdFilesTest, AppendPcdFilesFromDirectory)
{
  std::vector<std::string> path_list;

  appendPathOrPcdFiles(test_directory, path_list);

  ASSERT_EQ(path_list.size(), 2u);
  EXPECT_NE(
    std::find(path_list.begin(), path_list.end(), test_directory + "/test1.pcd"), path_list.end());
  EXPECT_NE(
    std::find(path_list.begin(), path_list.end(), test_directory + "/test2.PCD"), path_list.end());
}

TEST_F(AppendPathOrPcdFilesTest, ThrowsExceptionForInvalidPath)
{
  std::vector<std::string> path_list;
  std::string invalid_path = test_directory + "/non_existent";

  EXPECT_THROW(appendPathOrPcdFiles(invalid_path, path_list), std::runtime_error);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
