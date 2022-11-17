// Copyright 2022 Macnica, Inc.
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

#ifndef ROUTE_HANDLER__LANELET_POINT_HPP_
#define ROUTE_HANDLER__LANELET_POINT_HPP_

#include "route_handler/forward.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

namespace route_handler
{

//! @brief A point on a lanelet centerline (2d only).
class LaneletPoint
{
public:
  //! @brief Uninitialized lanelet point (invalid).
  LaneletPoint() = default;
  //! @brief Initialize point on given lanelet centerline, at given arc length offset.
  explicit LaneletPoint(const lanelet::ConstLanelet & lanelet, double arc_length);

  //! @note Invalid lanelet point are never equal.
  inline bool operator==(const LaneletPoint & other) const;
  //! @note Invalid lanelet point are never equal.
  inline bool operator!=(const LaneletPoint & other) const;

  //! @brief Get the first point on the lanelet centerline (arc_length=0).
  [[nodiscard]] static LaneletPoint startOf(const lanelet::ConstLanelet & lanelet);
  //! @brief Get the last point on the lanelet centerline.
  [[nodiscard]] static LaneletPoint endOf(const lanelet::ConstLanelet & lanelet);

  //! @brief Project the point on the lanelet centerline.
  //! @return projection result as LaneletPoint.
  [[nodiscard]] static LaneletPoint fromProjection(
    const lanelet::ConstLanelet & lanelet, const lanelet::BasicPoint2d & point);
  //! @brief Project the point on the lanelet centerline.
  //! @return projection result as LaneletPoint.
  [[nodiscard]] static LaneletPoint fromProjection(
    const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & point);
  //! @brief Project the point on the lanelet centerline.
  //! @return projection result as LaneletPoint.
  [[nodiscard]] static LaneletPoint fromProjection(
    const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Pose & pose);
  //! @brief Project the point on the lanelet centerline.
  //! @return projection result as LaneletPoint.
  
  //! @brief Conversion to lanelet::BasicPoint2d.
  [[nodiscard]] lanelet::BasicPoint2d toBasicPoint2d() const;
  //! @brief Conversion to geometry_msgs::msg::Point.
  [[nodiscard]] geometry_msgs::msg::Point toPointMsg() const;

  //! @brief Whether the point is initialized correctly (has valid lanelet).
  [[nodiscard]] bool isValid() const;

  //! @brief Get section from the point to the end of the lanelet.
  [[nodiscard]] LaneletSection forwardSection() const;
  //! @brief Get section from the beginning of the lanelet to the point.
  [[nodiscard]] LaneletSection backwardSection() const;

  // Getters

  [[nodiscard]] const lanelet::ConstLanelet & lanelet() const { return lanelet_; }
  [[nodiscard]] double lanelet_length() const { return lanelet_length_; }
  [[nodiscard]] double arc_length() const { return arc_length_; }

private:
  lanelet::ConstLanelet lanelet_{lanelet::InvalId};  //!< target lanelet
  double lanelet_length_{0.};  //!< length of the lanelet centerline (internal use)
  double arc_length_{0.};      //!< arc length offset of point on lanelet 2d centerline
};

bool LaneletPoint::operator==(const LaneletPoint & other) const
{
  return isValid() && lanelet_ == other.lanelet_ && arc_length_ == other.arc_length_;
}

bool LaneletPoint::operator!=(const LaneletPoint & other) const { return !operator==(other); }

}  // namespace route_handler

#endif  // ROUTE_HANDLER__LANELET_POINT_HPP_
