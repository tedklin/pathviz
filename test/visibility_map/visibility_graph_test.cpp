#include "pathviz/visibility_map/visibility_graph.hpp"

#include <set>

#include "gtest/gtest.h"

using namespace pathviz;

TEST(VisibilityMap, TerrainTest) {
  visibility_map::Terrain terrain;
  terrain.AddObstacle(geometry_2d::Polygon({{0, 0}, {3, 0}, {0, 4}}));
  terrain.AddObstacle(
      geometry_2d::Polygon({{-10, -10}, {-9, -10}, {-9, -9}, {-10, -9}}));

  std::set<geometry_2d::Point> vertices{
      {0, 0}, {3, 0}, {0, 4}, {-10, -10}, {-9, -10}, {-9, -9}, {-10, -9}};
  EXPECT_EQ(vertices, terrain.AllVertices());
}

TEST(VisibilityMap, VisibleVerticesSweep) {
  // Example in Choset p.115, reflected about the y-axis (our sweep order is
  // slightly different).
  visibility_map::Terrain terrain;
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
