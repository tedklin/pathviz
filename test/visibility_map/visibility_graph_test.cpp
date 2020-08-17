#include "pathviz/visibility_map/visibility_graph.hpp"

#include <set>

#include "gtest/gtest.h"

using namespace pathviz;

TEST(VisibilityMap, VisibilitySweep) {
  // Example in Choset p.115, reflected about the y-axis (our sweep order is
  // slightly different).
  geometry_2d::Terrain terrain;
  terrain.AddObstacle(
      geometry_2d::Polygon({{-2, 1}, {-2, -2}, {-3, -2}, {-3, 1}}));
  terrain.AddObstacle(
      geometry_2d::Polygon({{-4, 2}, {-4, -3}, {-6, -3}, {-6, 2}}));

  std::cout << "visibility sweep verbose\n\n";
  std::set<geometry_2d::Point> vertices = visibility_map::get_visible_vertices(
      terrain, geometry_2d::Point{0, 0}, true);
  EXPECT_EQ(vertices,
            std::set<geometry_2d::Point>(
                {geometry_2d::Point{-2, 1}, geometry_2d::Point{-2, -2}}));
}
