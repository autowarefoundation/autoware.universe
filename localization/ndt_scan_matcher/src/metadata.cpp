#include <ndt_scan_matcher/metadata.hpp>

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

namespace loc
{
SegmentIndex & SegmentIndex::operator=(const SegmentIndex & other)
{
  x = other.x;
  y = other.y;

  return *this;
}

SegmentIndex & SegmentIndex::operator=(SegmentIndex && other)
{
  x = other.x;
  y = other.y;

  return *this;
}

bool SegmentIndex::operator<(const SegmentIndex & other) const
{
  return ((x < other.x) || (x == other.x && y < other.y));
}

std::string SegmentIndex::to_string() const
{
  std::ostringstream out;

  out << x << "_" << y;

  return out.str();
}

PCDMetadata::PCDMetadata()
{
  res_x_ = res_y_ = 0;
  lower_x_ = lower_y_ = 0;
}

// Import data from a metadata file
void PCDMetadata::import(const std::string & pcd_metadata_path, const std::string & pcd_dir)
{
  YAML::Node config = YAML::LoadFile(pcd_metadata_path);
  bool extract_tag_from_key = false;

  safeLoad<float>(config, "x_resolution", res_x_);
  safeLoad<float>(config, "y_resolution", res_y_);

  if (!safeLoad<float>(config, "lower_x", lower_x_)) {
    lower_x_ = 0;
  }

  if (!safeLoad<float>(config, "lower_y", lower_y_)) {
    lower_y_ = 0;
  }

  if (!safeLoad<std::string>(config, "tag_name", tag_name_)) {
    // Extract tag name from the string id of segments
    extract_tag_from_key = true;
  }

  for (const auto & node : config) {
    // Skip metadata info
    if (
      node.first.as<std::string>() == "x_resolution" ||
      node.first.as<std::string>() == "y_resolution" || node.first.as<std::string>() == "lower_x" ||
      node.first.as<std::string>() == "lower_y" || node.first.as<std::string>() == "tag_name") {
      continue;
    }

    // Extract tag name from string id of segments
    // Just for backward compatibility, when the new format of metadata is applied, remove this
    if (extract_tag_from_key) {
      auto str_id = node.first.as<std::string>();
      auto first_ubar_loc = str_id.find("_"), last_ubar_loc = str_id.rfind("_");

      // If these two are the same, just leave tag name as empty
      // Otherwise, cut the prefix of the str_id and make it as tag name
      if (first_ubar_loc != last_ubar_loc) {
        tag_name_ = str_id.substr(0, first_ubar_loc);
      }

      extract_tag_from_key = false;
    }

    // Append PCD segment info
    std::vector<int> values = node.second.as<std::vector<int>>();
    SegmentIndex seg_id(values[0], values[1]);
    // Construct the absolute path of the corresponding segment PCD
    std::ostringstream abs_path_pcd;

    abs_path_pcd << pcd_dir << "/" << tag_name_ << "_" << seg_id.to_string() << ".pcd";

    // If the file exists, insert its information to the list of segments
    if (
      std::filesystem::exists(abs_path_pcd.str()) &&
      std::filesystem::is_regular_file(abs_path_pcd.str())) {
      segment_list_[seg_id] = abs_path_pcd.str();
    } else {
      // Otherwise, file a warning
    }
  }
}

// Utility function to quickly convert coordinate to segment index
// Notice that the SegmentIndex is the BOUNDARY coordinate in integer of the segment
// It is NOT the segmentation index (i.e. coordinate / resolution)
SegmentIndex PCDMetadata::coordinateToSegmentIndex(float x, float y) const
{
  int idx = static_cast<int>(floor((x - lower_x_) / res_x_) * res_x_);
  int idy = static_cast<int>(floor((y - lower_y_) / res_y_) * res_y_);

  return SegmentIndex(idx, idy);
}

void PCDMetadata::segmentIndexToBoundary(
  const SegmentIndex & sid, float & min_x, float & min_y, float & max_x, float & max_y)
{
  min_x = static_cast<float>(sid.x) * res_x_ + lower_x_;
  min_y = static_cast<float>(sid.y) * res_y_ + lower_y_;
  max_x = min_x + res_x_;
  max_y = max_y + res_y_;
}

std::map<SegmentIndex, std::string>::iterator PCDMetadata::find(const SegmentIndex & key)
{
  return segment_list_.find(key);
}

// Relay endpoints of segment list to outside
auto PCDMetadata::begin() -> decltype(segment_list_.begin())
{
  return segment_list_.begin();
}

auto PCDMetadata::end() -> decltype(segment_list_.end())
{
  return segment_list_.end();
}

}  // namespace loc
