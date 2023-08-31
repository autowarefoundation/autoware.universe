#include <boost/geometry/geometry.hpp>

#include <gtest/gtest.h>
using BoostPoint = boost::geometry::model::d2::point_xy<double>;
using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;

TEST(MapBasedRule, eagleyMap)
{
  const BoostPoint boost_point(88735, 43149);
  std::vector<BoostPolygon> bounding_boxes;
  // 88898.4,42673.4
  // 88861.1,42637.8
  // 88679.3,42957.2
  // 88707.8,42975.6

  BoostPolygon box;
  box.outer().push_back(BoostPoint(88898.4, 42673.4));
  box.outer().push_back(BoostPoint(88861.1, 42637.8));
  box.outer().push_back(BoostPoint(88679.3, 42957.2));
  box.outer().push_back(BoostPoint(88707.8, 42975.6));

  // NOTE: this check is failed due to not-closed BoostPolygon
  // EXPECT_FALSE(boost::geometry::within(boost_point, box));

  box.outer().push_back(BoostPoint(88898.4, 42673.4));
  EXPECT_FALSE(boost::geometry::within(boost_point, box));
}