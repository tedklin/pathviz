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

struct Edge {
  Point from, to;
  Point Other(Point p);
};

inline bool operator==(const Edge& lhs, const Edge& rhs) {
  return lhs.to == rhs.to && lhs.from == rhs.from;
}

inline bool operator!=(const Edge& lhs, const Edge& rhs) {
  return !operator==(lhs, rhs);
}

inline std::string to_string(const Edge& p) {
  return "[" + to_string(p.from) + "->" + to_string(p.to) + "]";
}

double distance(const Point& p1, const Point& p2);
double length(const Edge& e);

double angle_from_horizontal(const Point& source, const Point& target);
double angle_from_horizontal(const Edge& e);

double slope(const Edge& e);
double y_intercept(const Edge& e);

// Does not count common endpoints or any form of colinearity as an
// intersection.
bool is_intersecting(const Edge& e1, const Edge& e2);

// Adjacent pairs of vertices represent edges of the polygon; the vertices on
// each end of the vector close the polygon.
struct Polygon {
  Polygon(std::vector<Point> polygon);

  std::pair<Edge, Edge> IncidentEdges(Point from) const;

  const std::vector<Point> polygon_;
};

// TODO: overload equality operator for Polygon, taking into account possible
// offset and wrap-around

}  // namespace geometry_2d
}  // namespace pathviz
