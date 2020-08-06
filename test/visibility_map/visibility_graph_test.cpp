#include "pathviz/visibility_map/visibility_graph.hpp"

#include <set>

#include "gtest/gtest.h"

using namespace pathviz;

TEST(VisibilityMap, Terrain) {
  visibility_map::Terrain terrain;
  terrain.AddObstacle(geometry_2d::Polygon({{0, 0}, {3, 0}, {0, 4}}));
  terrain.AddObstacle(
      geometry_2d::Polygon({{-10, -10}, {-9, -10}, {-9, -9}, {-10, -9}}));

  std::set<geometry_2d::Point> vertices{
      {0, 0}, {3, 0}, {0, 4}, {-10, -10}, {-9, -10}, {-9, -9}, {-10, -9}};
  ASSERT_EQ(vertices, terrain.GetAllVertices());
}
