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

#ifndef ROUTE_HANDLER__LANELET_SECTION_HPP_
#define ROUTE_HANDLER__LANELET_SECTION_HPP_

#include "route_handler/forward.hpp"

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

#include <optional>

namespace route_handler
{

//! @brief A section of a single lanelet, with start and end point.
class LaneletSection
{  
public:
  //! @brief Uninitialized lanelet section (invalid).
  LaneletSection() = default;
  //! @brief Initialize section of length 0 (start=end).
  explicit LaneletSection(const LaneletPoint &point);
  //! @brief Initialize lanelet section.
  //! @param lanelet target lanelet.
  //! @param optional_start_arc_length arc length of start point, or the beginning of the lanelet if none.
  //! @param optional_end_arc_length arc length of end point, or the end of the lanelet if none.
  //! Start and End points are clamped within lanelet boundaries.
  //! The section will be left uninitialized if the start point is after the end point.
  explicit LaneletSection(
    const lanelet::ConstLanelet & lanelet, 
    const std::optional<double> optional_start_arc_length = {}, 
    const std::optional<double> optional_end_arc_length = {});

  //! @brief Whether the section is initialized correctly
  [[nodiscard]] bool isValid() const;

  //! @brief Whether the section is a point (length = 0)
  [[nodiscard]] bool isPoint() const; 

  //! @brief Whether section covers the whole lanelet
  [[nodiscard]] bool coversWholeLanelet() const;

  //! @brief Lenght of the centerline from start to end point
  [[nodiscard]] double length() const;

  // point query

  //! @brief Whether the point is inside the section
  [[nodiscard]] bool contains(const LaneletPoint& point) const;

  //! @brief Get section start point.
  //! @return start point if section is valid, an invalid point otherwise.
  [[nodiscard]] LaneletPoint getStartPoint() const;

  //! @brief Get section end point.
  //! @return end point if section is valid, an invalid point otherwise.
  [[nodiscard]] LaneletPoint getEndPoint() const;
  
  //! @brief Get point at given arc length within the section.
  //! If the arc length points outside the section, the start/end point of the section is returned instead (whichever is closest).
  //! @note arc length is relative to the section, not the lanelet
  //! @return the point at given arc length, an invalid point if the section is not initialized.
  [[nodiscard]] LaneletPoint getPointAt(const double arc_length) const;
  
  // line query

  //! @brief Get center line of section
  [[nodiscard]] std::vector<lanelet::BasicPoint3d> getCenterline() const;
  //! @brief Get left bound of section
  [[nodiscard]] std::vector<lanelet::BasicPoint3d> getLeftBound() const;
  //! @brief Get right bound of section
  [[nodiscard]] std::vector<lanelet::BasicPoint3d> getRightBound() const;

  // editing utils
  
  //! @brief Split the section in two at given point
  //! Splitting will fail if the point is not in the section.
  //! @return whether split was successful
  [[nodiscard]] bool split(
    const LaneletPoint & split_point,
    LaneletSection * section_before,
    LaneletSection * section_after) const;

  //! @brief Concatenate two sections
  //! The two sections must belong to the same lanelet, and must be connected. Section order also matters.
  //! @return the concatenated section if successful, an invalid section otherwise.
  [[nodiscard]] static LaneletSection concatenate(
    const LaneletSection & first_section,
    const LaneletSection & second_section);

  //! @brief Get intersection between 2 sections
  //! @note Sections from different lanelets are considered disjoint, even if they overlap geometrically.
  //! @return the intersection section if it exists, an invalid section otherwise
  [[nodiscard]] static LaneletSection intersect(
    const LaneletSection & section_A,
    const LaneletSection & section_B);

  // getters

  [[nodiscard]] const lanelet::ConstLanelet & lanelet() const { return lanelet_; }
  [[nodiscard]] double lanelet_length() const { return lanelet_length_; }
  [[nodiscard]] double start_arc_length() const { return start_arc_length_; }
  [[nodiscard]] double end_arc_length() const { return end_arc_length_; }

private:
  lanelet::ConstLanelet lanelet_; //!< target lanelet
  double lanelet_length_;         //!< lanelet centerline length
  double start_arc_length_{0.};   //!< arc length offset of the section start point on the lanelet centerline
  double end_arc_length_{0.};     //!< arc length offset of the section end point on the lanelet centerliner
};

}  // namespace route_handler

#endif  // ROUTE_HANDLER__LANELET_SECTION_HPP_
