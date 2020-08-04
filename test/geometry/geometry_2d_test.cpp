#include "pathviz/geometry/geometry_2d.hpp"

#include "gtest/gtest.h"

using namespace pathviz;

TEST(Geometry2dTest, Point) {
  geometry_2d::Point p1(3, 0), p2(0, 4), p1_copy(3, 0);
  EXPECT_TRUE(p1 == p1_copy);
  EXPECT_FALSE(p1 == p2);
  EXPECT_DOUBLE_EQ(5, geometry_2d::distance_2d(p1, p2));
}
