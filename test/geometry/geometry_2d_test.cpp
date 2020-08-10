#include "pathviz/geometry/geometry_2d.hpp"

#include "gtest/gtest.h"

using namespace pathviz;

using geometry_2d::LineSegment;
using geometry_2d::MinBoundingBox;
using geometry_2d::Point;
using geometry_2d::Polygon;

TEST(Geometry2d, PointTest) {
  Point p1{3, 0}, p2{0, 4}, p1_copy{3, 0};
  EXPECT_TRUE(p1 == p1_copy);
  EXPECT_FALSE(p1 == p2);
}

TEST(Geometry2d, LineSegmentTest) {
  LineSegment e{{3, 0}, {0, 4}};
  EXPECT_TRUE((Point{3, 0} == e.Other({0, 4})));
  EXPECT_TRUE((Point{0, 4} == e.Other({3, 0})));
  EXPECT_THROW(e.Other({0, 0}), std::runtime_error);

  LineSegment e_reverse = geometry_2d::reverse(e);
  EXPECT_TRUE((e_reverse == LineSegment{{0, 4}, {3, 0}}));
  EXPECT_FALSE((e_reverse == LineSegment{{3, 0}, {0, 4}}));
  EXPECT_TRUE((geometry_2d::reverse(e_reverse) == LineSegment{{3, 0}, {0, 4}}));
}

TEST(Geometry2d, NumericalTest) {
  EXPECT_DOUBLE_EQ(5, geometry_2d::distance(Point{3, 0}, Point{0, 4}));
  EXPECT_DOUBLE_EQ(5, geometry_2d::length(LineSegment{{3, 0}, {0, 4}}));

  EXPECT_DOUBLE_EQ((-4.0 / 3.0),
                   geometry_2d::slope(LineSegment{{3, 0}, {0, 4}}));
  EXPECT_DOUBLE_EQ(geometry_2d::slope(LineSegment{{3, 0}, {0, 4}}),
                   geometry_2d::slope(LineSegment{{0, 4}, {3, 0}}));

  EXPECT_DOUBLE_EQ(1, geometry_2d::y_intercept(LineSegment{{1, 2}, {2, 3}}));
  EXPECT_DOUBLE_EQ(1, geometry_2d::y_intercept(LineSegment{{1, 1}, {4, 1}}));
  EXPECT_THROW(geometry_2d::y_intercept(LineSegment{{1, 2}, {1, 1}}),
               std::runtime_error);

  EXPECT_DOUBLE_EQ(-1, geometry_2d::x_intercept(LineSegment{{1, 2}, {2, 3}}));
  EXPECT_DOUBLE_EQ(1, geometry_2d::x_intercept(LineSegment{{1, 1}, {1, 4}}));
  EXPECT_THROW(geometry_2d::x_intercept(LineSegment{{1, 1}, {2, 1}}),
               std::runtime_error);
}

TEST(Geometry2d, IntersectionTest) {
  // standard cases
  EXPECT_TRUE(geometry_2d::is_intersecting(LineSegment{{0, 0}, {3, 3}},
                                           LineSegment{{0, 3}, {3, 0}}));
  EXPECT_FALSE(geometry_2d::is_intersecting(LineSegment{{0, 0}, {3, 3}},
                                            LineSegment{{0, 7}, {7, 0}}));

  // vertical line
  EXPECT_TRUE(geometry_2d::is_intersecting(LineSegment{{0, -1}, {0, 1}},
                                           LineSegment{{1, 1}, {-1, -1}}));
  EXPECT_FALSE(geometry_2d::is_intersecting(LineSegment{{0, -1}, {0, 1}},
                                            LineSegment{{1, 1}, {2, 2}}));

  // horizontal line
  EXPECT_TRUE(geometry_2d::is_intersecting(LineSegment{{1, 0}, {-1, 0}},
                                           LineSegment{{1, 1}, {-1, -1}}));
  EXPECT_FALSE(geometry_2d::is_intersecting(LineSegment{{1, 0}, {-1, 0}},
                                            LineSegment{{1, 1}, {2, 2}}));

  // overlapping colinear
  EXPECT_FALSE(geometry_2d::is_intersecting(LineSegment{{0, 0}, {1, 1}},
                                            LineSegment{{0, 0}, {2, 2}}));

  // common endpoints
  EXPECT_FALSE(geometry_2d::is_intersecting(LineSegment{{0, 0}, {1, 1}},
                                            LineSegment{{1, 1}, {2, 0}}));

  // barely intersecting
  EXPECT_TRUE(
      geometry_2d::is_intersecting(LineSegment{{0, 0}, {1.0000001, 1.0000001}},
                                   LineSegment{{0, 2}, {2, 0}}));
}

TEST(Geometry2d, BoundingBoxTest) {
  MinBoundingBox box;
  box.AddPoint({1, 1});
  EXPECT_EQ(1, box.left_bound);
  EXPECT_EQ(1, box.right_bound);
  EXPECT_EQ(1, box.bottom_bound);
  EXPECT_EQ(1, box.top_bound);

  box.AddPoint({3, 1});
  EXPECT_EQ(1, box.left_bound);
  EXPECT_EQ(3, box.right_bound);
  EXPECT_EQ(1, box.bottom_bound);
  EXPECT_EQ(1, box.top_bound);

  box.AddPoint({-3, 2});
  EXPECT_EQ(-3, box.left_bound);
  EXPECT_EQ(3, box.right_bound);
  EXPECT_EQ(1, box.bottom_bound);
  EXPECT_EQ(2, box.top_bound);

  MinBoundingBox box2;
  box2.AddPoint({0, 0});
  box2.AddPoint({-5, -5});
  EXPECT_EQ(-5, box2.left_bound);
  EXPECT_EQ(0, box2.right_bound);
  EXPECT_EQ(-5, box2.bottom_bound);
  EXPECT_EQ(0, box2.top_bound);

  box.AddBoundingBox(box2);
  EXPECT_EQ(-5, box.left_bound);
  EXPECT_EQ(3, box.right_bound);
  EXPECT_EQ(-5, box.bottom_bound);
  EXPECT_EQ(2, box.top_bound);
}

TEST(Geometry2d, PolygonTest) {
  EXPECT_THROW(Polygon line({{0, 0}, {3, 0}}), std::runtime_error);

  Polygon triangle({{0, 0}, {3, 0}, {0, 4}});

  std::cout << "triangle AllEdges ad-hoc test:\n";
  for (const auto& edge : triangle.AllEdges()) {
    std::cout << geometry_2d::to_string(edge) << '\n';
  }

  EXPECT_THROW(auto incident_edges = triangle.IncidentEdges({4, 0}),
               std::runtime_error);

  auto incident_edges = triangle.IncidentEdges({0, 0});
  EXPECT_TRUE((incident_edges.first == LineSegment{{0, 0}, {0, 4}}));
  EXPECT_TRUE((incident_edges.second == LineSegment{{0, 0}, {3, 0}}));
  EXPECT_FALSE(incident_edges.first == incident_edges.second);

  incident_edges = triangle.IncidentEdges({3, 0});
  EXPECT_TRUE((incident_edges.first == LineSegment{{3, 0}, {0, 0}}));
  EXPECT_TRUE((incident_edges.second == LineSegment{{3, 0}, {0, 4}}));
  EXPECT_FALSE(incident_edges.first == incident_edges.second);

  incident_edges = triangle.IncidentEdges({0, 4});
  EXPECT_TRUE((incident_edges.first == LineSegment{{0, 4}, {3, 0}}));
  EXPECT_TRUE((incident_edges.second == LineSegment{{0, 4}, {0, 0}}));
  EXPECT_FALSE(incident_edges.first == incident_edges.second);

  geometry_2d::Polygon square({{-1, -1}, {1, -1}, {1, 1}, {-1, 1}});
  incident_edges = square.IncidentEdges({-1, -1});
  EXPECT_TRUE((incident_edges.first == LineSegment{{-1, -1}, {-1, 1}}));
  EXPECT_TRUE((incident_edges.second == LineSegment{{-1, -1}, {1, -1}}));
  EXPECT_FALSE(incident_edges.first == incident_edges.second);

  incident_edges = square.IncidentEdges({1, 1});
  EXPECT_TRUE((incident_edges.first == LineSegment{{1, 1}, {1, -1}}));
  EXPECT_TRUE((incident_edges.second == LineSegment{{1, 1}, {-1, 1}}));
  EXPECT_FALSE(incident_edges.first == incident_edges.second);
}
