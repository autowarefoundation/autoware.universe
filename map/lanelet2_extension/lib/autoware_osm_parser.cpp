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
 *
 * Authors: Ryohsuke Mitsudome
 */

#include "lanelet2_extension/io/autoware_osm_parser.hpp"

#include "lanelet2_core/geometry/LineString.h"
#include "lanelet2_io/io_handlers/Factory.h"
#include "lanelet2_io/io_handlers/OsmFile.h"
#include "lanelet2_io/io_handlers/OsmHandler.h"

#include <string>

namespace lanelet
{
namespace io_handlers
{

// non-const converter for const lanelet primitive types
struct NonConstConverter {
  auto operator()(const ConstPoint3d& c) {
    return Point3d(c.id(), c.basicPoint(), c.attributes());
  }

  auto operator()(const ConstLineString3d& c) {
    return LineString3d(c.id(), std::vector<Point3d>(c.begin(), c.end()), c.attributes());
  }

  auto operator()(const ConstLineStrings3d& cs) {
    auto ls = LineStrings3d{};
    ls.reserve(cs.size());
    for (auto&& c: cs) {
      ls.emplace_back(NonConstConverter()(c));
    }
    return ls;
  }

  auto operator()(const ConstInnerBounds& cs) {
    auto ib = InnerBounds{};
    ib.reserve(cs.size());
    for (auto&& c: cs) {
      ib.emplace_back(NonConstConverter()(c));
    }
    return ib;
  }

  auto operator()(const ConstPolygon3d& c) {
    return Polygon3d(c.id(), std::vector<Point3d>(c.begin(), c.end()), c.attributes());
  }

  auto operator()(const ConstWeakArea& e) {
    auto c = e.lock();
    return WeakArea(Area(std::const_pointer_cast<AreaData>(c.constData())));
  }

  auto operator()(const ConstWeakLanelet& e) {
    auto c = e.lock();
    return WeakLanelet(Lanelet(std::const_pointer_cast<LaneletData>(c.constData())));
  }
};

std::unique_ptr<LaneletMap> AutowareOsmParser::parse(
  const std::string & filename, ErrorMessages & errors) const
{
  auto map = OsmParser::parse(filename, errors);

  // overwrite x and y values if there are local_x, local_y tags
  for (Point3d point : map->pointLayer) {
    if (point.hasAttribute("local_x")) {
      point.x() = point.attribute("local_x").asDouble().value();
    }
    if (point.hasAttribute("local_y")) {
      point.y() = point.attribute("local_y").asDouble().value();
    }
  }

  // re-construct LaneletMapLayer
  PointLayer::Map points;
  LaneletLayer::Map lanelets;
  AreaLayer::Map areas;
  RegulatoryElementLayer::Map regulatory_elements;
  PolygonLayer::Map polygons;
  LineStringLayer::Map line_strings;

  for(auto&& elem: map->pointLayer) {
    points.emplace(elem.id(), Point3d(elem.id(), elem.basicPoint(), elem.attributes()));
  }

  for(const auto& elem: map->lineStringLayer) {
    line_strings.emplace(elem.id(), LineString3d(elem.id(), std::vector<Point3d>(elem.begin(), elem.end()), elem.attributes()));
  }

  for(const auto& elem: map->polygonLayer) {
    polygons.emplace(elem.id(), Polygon3d(elem.id(), std::vector<Point3d>(elem.begin(), elem.end()), elem.attributes()));
  }

  for(auto&& elem: map->laneletLayer) {
    lanelets.emplace(elem.id(), Lanelet(elem.id(), elem.leftBound(), elem.rightBound(), elem.attributes(), elem.regulatoryElements()));
  }

  for(auto&& elem: map->areaLayer) {
    areas.emplace(elem.id(), Area(elem.id(), elem.outerBound(), elem.innerBounds(), elem.attributes(), elem.regulatoryElements()));
  }

  for(auto&& elem: map->regulatoryElementLayer) {
    auto parameterMap = RuleParameterMap{};
    for(const auto& param: elem->getParameters()) {
      auto rules = RuleParameters{};
      rules.reserve(param.second.size());
      for (auto&& p: param.second) {
        auto v = boost::apply_visitor([](const auto& v){ return RuleParameter(NonConstConverter{}(v)); }, p);
        rules.emplace_back(v);
      }
      parameterMap[param.first] = rules;
    }
    auto rule = elem->attributes().at(AttributeNamesString::Subtype).value();
    regulatory_elements.emplace(elem->id(), RegulatoryElementFactory::create(rule, elem->id(), parameterMap, elem->attributes()));
  }

  map = std::make_unique<LaneletMap>(lanelets, areas, regulatory_elements, polygons, line_strings, points);

  // rerun align function in just in case
  for (Lanelet & lanelet : map->laneletLayer) {
    LineString3d new_left, new_right;
    std::tie(new_left, new_right) = geometry::align(lanelet.leftBound(), lanelet.rightBound());
    lanelet.setLeftBound(new_left);
    lanelet.setRightBound(new_right);
  }

  return map;
}

namespace
{
RegisterParser<AutowareOsmParser> regParser;
}

void AutowareOsmParser::parseVersions(
  const std::string & filename, std::string * format_version, std::string * map_version)
{
  if (format_version == nullptr || map_version == nullptr) {
    std::cerr << __FUNCTION__ << ": either format_version or map_version is null pointer!";
    return;
  }

  pugi::xml_document doc;
  auto result = doc.load_file(filename.c_str());
  if (!result) {
    throw lanelet::ParseError(
      std::string("Errors occurred while parsing osm file: ") + result.description());
  }

  auto osmNode = doc.child("osm");
  auto metainfo = osmNode.child("MetaInfo");
  if (metainfo.attribute("format_version")) {
    *format_version = metainfo.attribute("format_version").value();
  }
  if (metainfo.attribute("map_version")) {*map_version = metainfo.attribute("map_version").value();}
}

}  // namespace io_handlers
}  // namespace lanelet
