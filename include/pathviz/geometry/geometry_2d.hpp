#pragma once

#include <string>
#include <utility>
#include <vector>

#include "graphlib/geometry/graph_2d.hpp"

namespace pathviz {
namespace geometry_2d {

struct Point {
  double x, y;
};

inline bool operator==(const Point& lhs, const Point& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

inline bool operator!=(const Point& lhs, const Point& rhs) {
  return !operator==(lhs, rhs);
}

// Ordering with x-coordinate precedence.
inline bool operator<(const Point& lhs, const Point& rhs) {
  if (lhs.x != rhs.x) {
    return lhs.x < rhs.x;
  } else {
    return lhs.y < rhs.y;
  }
}

inline std::string to_string(const Point& p) {
  return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
}

struct LineSegment {
  Point from, to;
  Point Other(Point p);
};

inline bool operator==(const LineSegment& lhs, const LineSegment& rhs) {
  return lhs.to == rhs.to && lhs.from == rhs.from;
}

inline bool operator!=(const LineSegment& lhs, const LineSegment& rhs) {
  return !operator==(lhs, rhs);
}

inline std::string to_string(const LineSegment& p) {
  return "[" + to_string(p.from) + "->" + to_string(p.to) + "]";
}

LineSegment reverse(const LineSegment& e);

double distance(const Point& p1, const Point& p2);
double length(const LineSegment& e);

// Angle (-pi, pi] from positive horizontal axis w.r.t. a source Point.
double angle_from_horizontal(const Point& source, const Point& target);

bool is_horizontal(const LineSegment& e);
bool is_vertical(const LineSegment& e);

double slope(const LineSegment& e);
double y_intercept(const LineSegment& e);
double x_intercept(const LineSegment& e);

// Does not count common endpoints as intersections. Treats any form of
// colinearity as NOT an intersection. Note that this function may be subject to
// floating point arithmetic error.
bool is_intersecting(const LineSegment& e1, const LineSegment& e2);

// Adjacent pairs of vertices represent edges of the polygon; the vertices on
// each end of the vector close the polygon.
struct Polygon {
  Polygon(std::vector<Point> polygon);

  std::vector<LineSegment> AllEdges() const;

  // Incident edges pointing outwards from the given point.
  std::pair<LineSegment, LineSegment> IncidentEdges(Point from) const;

  const std::vector<Point> polygon_;
};

// TODO: overload equality operator for Polygon, taking into account possible
// offset and wrap-around

}  // namespace geometry_2d
}  // namespace pathviz
