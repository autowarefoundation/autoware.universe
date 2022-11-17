#include "ll2_decomposer/from_bin_msg.hpp"

#include <lanelet2_extension/regulatory_elements/detection_area.hpp>

#include <boost/archive/binary_iarchive.hpp>

#include <lanelet2_io/io_handlers/Serialize.h>

namespace map
{
lanelet::LaneletMapPtr fromBinMsg(const autoware_auto_mapping_msgs::msg::HADMapBin & msg)
{
  lanelet::LaneletMapPtr map = std::make_shared<lanelet::LaneletMap>();

  std::string data_str;
  data_str.assign(msg.data.begin(), msg.data.end());

  std::stringstream ss;
  ss << data_str;
  boost::archive::binary_iarchive oa(ss);
  oa >> *map;
  lanelet::Id id_counter;
  oa >> id_counter;
  lanelet::utils::registerId(id_counter);
  return map;
}

}  // namespace map