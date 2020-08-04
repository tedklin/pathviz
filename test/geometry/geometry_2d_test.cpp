#include "pathviz/geometry/geometry_2d.hpp"

#include "gtest/gtest.h"

using namespace pathviz;

TEST(Geometry2d, Point) {
  geometry_2d::Point p1{3, 0}, p2{0, 4}, p1_copy{3, 0};
  EXPECT_TRUE(p1 == p1_copy);
  EXPECT_FALSE(p1 == p2);
  EXPECT_DOUBLE_EQ(5, geometry_2d::distance_2d(p1, p2));
}

TEST(Geometry2d, IncidentEdges) {
  EXPECT_THROW(geometry_2d::Polygon line({{0, 0}, {3, 0}}), std::runtime_error);

  geometry_2d::Polygon triangle({{0, 0}, {3, 0}, {0, 4}});

  EXPECT_THROW(auto incident_edges = triangle.IncidentEdges({4, 0}),
               std::runtime_error);

  auto incident_edges = triangle.IncidentEdges({0, 0});
  EXPECT_TRUE((incident_edges.first == geometry_2d::Edge{{0, 0}, {3, 0}} ||
               incident_edges.first == geometry_2d::Edge{{0, 0}, {0, 4}}));
  EXPECT_TRUE((incident_edges.second == geometry_2d::Edge{{0, 0}, {3, 0}} ||
               incident_edges.second == geometry_2d::Edge{{0, 0}, {0, 4}}));
  EXPECT_FALSE(incident_edges.first == incident_edges.second);

  incident_edges = triangle.IncidentEdges({3, 0});
  EXPECT_TRUE((incident_edges.first == geometry_2d::Edge{{3, 0}, {0, 0}} ||
               incident_edges.first == geometry_2d::Edge{{3, 0}, {0, 4}}));
  EXPECT_TRUE((incident_edges.second == geometry_2d::Edge{{3, 0}, {0, 0}} ||
               incident_edges.second == geometry_2d::Edge{{3, 0}, {0, 4}}));
  EXPECT_FALSE(incident_edges.first == incident_edges.second);

  incident_edges = triangle.IncidentEdges({0, 4});
  EXPECT_TRUE((incident_edges.first == geometry_2d::Edge{{0, 4}, {0, 0}} ||
               incident_edges.first == geometry_2d::Edge{{0, 4}, {3, 0}}));
  EXPECT_TRUE((incident_edges.second == geometry_2d::Edge{{0, 4}, {0, 0}} ||
               incident_edges.second == geometry_2d::Edge{{0, 4}, {3, 0}}));
  EXPECT_FALSE(incident_edges.first == incident_edges.second);

  geometry_2d::Polygon square({{-1, -1}, {1, -1}, {1, 1}, {-1, 1}});
  incident_edges = square.IncidentEdges({-1, -1});
  EXPECT_TRUE((incident_edges.first == geometry_2d::Edge{{-1, -1}, {1, -1}} ||
               incident_edges.first == geometry_2d::Edge{{-1, -1}, {-1, 1}}));
  EXPECT_TRUE((incident_edges.second == geometry_2d::Edge{{-1, -1}, {1, -1}} ||
               incident_edges.second == geometry_2d::Edge{{-1, -1}, {-1, 1}}));
  EXPECT_FALSE(incident_edges.first == incident_edges.second);

  incident_edges = square.IncidentEdges({1, 1});
  EXPECT_TRUE((incident_edges.first == geometry_2d::Edge{{1, 1}, {1, -1}} ||
               incident_edges.first == geometry_2d::Edge{{1, 1}, {-1, 1}}));
  EXPECT_TRUE((incident_edges.second == geometry_2d::Edge{{1, 1}, {1, -1}} ||
               incident_edges.second == geometry_2d::Edge{{1, 1}, {-1, 1}}));
  EXPECT_FALSE(incident_edges.first == incident_edges.second);
}
