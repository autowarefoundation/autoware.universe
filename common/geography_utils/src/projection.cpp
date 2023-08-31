// Copyright 2023 TIER IV, Inc.
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

#include <GeographicLib/Geoid.hpp>
#include <geography_utils/projection.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_projection/UTM.h>

#include <map>
#include <stdexcept>
#include <string>
#include <utility>

namespace geography_utils
{

Eigen::Vector3d to_basic_point_3d_pt(const LocalPoint src)
{
  Eigen::Vector3d dst;
  dst.x() = src.x;
  dst.y() = src.y;
  dst.z() = src.z;
  return dst;
}

LocalPoint project_forward(const GeoPoint geo_point, const MapProjectorInfo & projector_info)
{
  LocalPoint local_point;
  if (projector_info.projector_type == MapProjectorInfo::LOCAL_CARTESIAN_UTM) {
    lanelet::GPSPoint position{geo_point.latitude, geo_point.longitude, geo_point.altitude};
    lanelet::Origin origin{position};
    lanelet::projection::UtmProjector projector{origin};
    lanelet::BasicPoint3d projected_local_point = projector.forward(position);
    local_point.x = projected_local_point.x();
    local_point.y = projected_local_point.y();
    local_point.z = projected_local_point.z();
  } else if (projector_info.projector_type == MapProjectorInfo::MGRS) {
    const int mgrs_precision = 9;  // set precision as 100 micro meter
    lanelet::projection::MGRSProjector projector{};
    lanelet::GPSPoint position{geo_point.latitude, geo_point.longitude, geo_point.altitude};
    lanelet::BasicPoint3d projected_local_point = projector.forward(position, mgrs_precision);
    local_point.x = projected_local_point.x();
    local_point.y = projected_local_point.y();
    local_point.z = projected_local_point.z();
  } else {
    const std::string error_msg = "Invalid map projector type: " + projector_info.projector_type +
                                  ". Currently supported types: MGRS, and LocalCartesianUTM";
    throw std::runtime_error(error_msg);
  }
  return local_point;
}

GeoPoint project_reverse(const LocalPoint local_point, const MapProjectorInfo & projector_info)
{
  GeoPoint geo_point;
  if (projector_info.projector_type == MapProjectorInfo::LOCAL_CARTESIAN_UTM) {
    lanelet::GPSPoint position{
      projector_info.map_origin.latitude, projector_info.map_origin.longitude,
      projector_info.map_origin.altitude};
    lanelet::Origin origin{position};
    lanelet::projection::UtmProjector projector{origin};
    lanelet::GPSPoint projected_gps_point = projector.reverse(to_basic_point_3d_pt(local_point));
    geo_point.latitude = projected_gps_point.lat;
    geo_point.longitude = projected_gps_point.lon;
    geo_point.altitude = projected_gps_point.ele;
  } else if (projector_info.projector_type == MapProjectorInfo::MGRS) {
    lanelet::GPSPoint projected_gps_point = lanelet::projection::MGRSProjector::reverse(
      to_basic_point_3d_pt(local_point), projector_info.mgrs_grid);
    geo_point.latitude = projected_gps_point.lat;
    geo_point.longitude = projected_gps_point.lon;
    geo_point.altitude = projected_gps_point.ele;
  } else {
    const std::string error_msg = "Invalid map projector type: " + projector_info.projector_type +
                                  ". Currently supported types: MGRS, and LocalCartesianUTM";
    throw std::runtime_error(error_msg);
  }
  return geo_point;
}

}  // namespace geography_utils
