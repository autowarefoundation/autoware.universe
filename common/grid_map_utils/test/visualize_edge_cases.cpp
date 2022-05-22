// Copyright 2022 Tier IV, Inc.
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

#include "grid_map_utils/polygon_iterator.hpp"

#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <random>
#include <string>
#include <vector>

int main()
{
  grid_map::Polygon base_polygon;
  base_polygon.addVertex(grid_map::Position(-5.0, 5.0));
  base_polygon.addVertex(grid_map::Position(0.0, 5.0));
  base_polygon.addVertex(grid_map::Position(5.0, 5.0));
  base_polygon.addVertex(grid_map::Position(5.0, 0.0));
  base_polygon.addVertex(grid_map::Position(5.0, -5.0));
  base_polygon.addVertex(grid_map::Position(0.0, -5.0));
  base_polygon.addVertex(grid_map::Position(-5.0, -5.0));
  base_polygon.addVertex(grid_map::Position(-5.0, 0.0));

  for (auto seed = 0; seed < 10000; ++seed) {
    std::printf("SEED = %d", seed);
    std::random_device r;
    std::default_random_engine engine(seed);
    std::uniform_real_distribution poly_offset(-2.50, 2.50);
    std::uniform_real_distribution res_dist(0.01, 1.00);
    std::uniform_real_distribution move_dist(-2.0, 2.0);
    grid_map::Polygon polygon;
    grid_map::GridMap map({"layer"});
    grid_map::GridMap gridmap({"layer"});
    const auto resolution = res_dist(engine);
    std::printf(" Resolution = %f\n", resolution);
    map.setGeometry(grid_map::Length(10.0, 10.0), resolution, grid_map::Position(0.0, 0.0));
    gridmap.setGeometry(grid_map::Length(10.0, 10.0), resolution, grid_map::Position(0.0, 0.0));
    const auto move = grid_map::Position(move_dist(engine), move_dist(engine));
    // const auto move = grid_map::Position(0.0, 0.0);
    map.move(move);
    gridmap.move(move);
    for (const auto & vertex : base_polygon.getVertices()) {
      polygon.addVertex(
        vertex + grid_map::Position(poly_offset(engine), poly_offset(engine)) + move);
    }
    grid_map_utils::PolygonIterator iterator(map, polygon);
    grid_map::PolygonIterator iterator_gridmap(gridmap, polygon);
    while (!iterator.isPastEnd() && !iterator_gridmap.isPastEnd()) {
      if (
        (*iterator).x() != (*iterator_gridmap).x() || (*iterator).y() != (*iterator_gridmap).y()) {
        grid_map_utils::PolygonIterator iterator(map, polygon);
        while (!iterator.isPastEnd()) {
          map.at("layer", *iterator) = 100;
          ++iterator;
        }
        grid_map::PolygonIterator iterator_gridmap(gridmap, polygon);
        while (!iterator_gridmap.isPastEnd()) {
          gridmap.at("layer", *iterator_gridmap) = 100;
          ++iterator_gridmap;
        }

        cv::Mat img;
        cv::Mat img2;
        cv::Mat custom_img;
        cv::Mat gm_img;
        cv::Mat diff_img;

        grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
          map, "layer", CV_8UC1, 0.0, 100, img);
        cv::resize(img, custom_img, cv::Size(500, 500), cv::INTER_LINEAR);
        grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
          gridmap, "layer", CV_8UC1, 0.0, 100, img2);
        cv::resize(img2, gm_img, cv::Size(500, 500), cv::INTER_LINEAR);
        cv::compare(custom_img, gm_img, diff_img, cv::CMP_EQ);
        if (cv::countNonZero(diff_img) > 0) {
          cv::imshow("custom", custom_img);
          cv::imshow("grid_map", gm_img);
          cv::imshow("diff", diff_img);
          cv::waitKey(0);
          cv::destroyAllWindows();
        }
        break;
      }
      ++iterator;
      ++iterator_gridmap;
    }
  }
}
