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

#ifndef NDT_SCAN_MATCHER__METADATA_HPP_
#define NDT_SCAN_MATCHER__METADATA_HPP_

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

namespace loc
{

struct SegmentIndex
{
  int x, y;

  SegmentIndex() {}
  SegmentIndex(int in_x, int in_y) : x(in_x), y(in_y) {}
  SegmentIndex(const SegmentIndex & other) : x(other.x), y(other.y) {}
  SegmentIndex(SegmentIndex && other) : x(other.x), y(other.y) {}

  SegmentIndex & operator=(const SegmentIndex & other);
  SegmentIndex & operator=(SegmentIndex && other);

  bool operator<(const SegmentIndex & other) const;

  std::string to_string() const;
};

struct PCDMetadata
{
  // Segmentation resolution
  float res_x_, res_y_;
  // Lower bound of the segmentation
  float lower_x_, lower_y_;
  // Prefix of the segment files
  std::string tag_name_;
  // Key: segment index, val: absolute path to segment PCD
  std::map<SegmentIndex, std::string> segment_list_;

  PCDMetadata();

  // Import data from a metadata file
  void import(const std::string & pcd_metadata_path, const std::string & pcd_dir);

  // Utility function to quickly convert coordinate to segment index
  // Notice that the SegmentIndex is the BOUNDARY coordinate in integer of the segment
  // It is NOT the segmentation index (i.e. coordinate / resolution)
  SegmentIndex coorToSegmentIndex(float x, float y) const;

  void segmentIndexToBoundary(
    const SegmentIndex & sid, float & min_x, float & min_y, float & max_x, float & max_y);

  std::map<SegmentIndex, std::string>::iterator find(const SegmentIndex & key);

  // Relay endpoints of segment list to outside
  auto begin() -> decltype(segment_list_.begin());
  auto end() -> decltype(segment_list_.end());

  size_t size() { return segment_list_.size(); }

private:
  // Safely find a key from a YAML node, and save its value to val
  template <typename T>
  bool safeLoad(const YAML::Node & node, const std::string & key, T & val)
  {
    bool retval = true;

    try {
      auto val_node = node[key];
      val = val_node.as<T>();
    } catch (YAML::Exception & e) {
      std::cerr << "Failed to retrieve a value at key " << key << ". Error: " << e.what()
                << std::endl;
      retval = false;
    }

    return retval;
  }
};

}  // namespace loc

#endif  // NDT_SCAN_MATCHER__METADATA_HPP_
