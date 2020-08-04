#pragma once

#include <algorithm>
#include <exception>
#include <string>
#include <utility>
#include <vector>

#include "graphlib/geometry/graph_2d.hpp"

namespace pathviz {
namespace geometry_2d {

struct Point {
  double x, y;
};

struct Edge {
  Point from, to;
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

inline bool operator==(const Edge& lhs, const Edge& rhs) {
  return lhs.to == rhs.to && lhs.from == rhs.from;
}

inline bool operator!=(const Edge& lhs, const Edge& rhs) {
  return !operator==(lhs, rhs);
}

inline std::string to_string(const Point& p) {
  return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
}

inline std::string to_string(const Edge& p) {
  return "[" + to_string(p.from) + "->" + to_string(p.to) + "]";
}

inline double distance_2d(const Point& p1, const Point& p2) {
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

inline double distance_2d(const Edge& e) {
  return std::sqrt(std::pow(e.from.x - e.to.x, 2) +
                   std::pow(e.from.y - e.to.y, 2));
}

struct Polygon {
  Polygon(std::vector<Point> polygon);

  std::pair<Edge, Edge> IncidentEdges(Point from) const;

  // Adjacent vertices represent edges of the polygon; the vertices on the
  // ends of the vector close the polygon.
  const std::vector<Point> polygon_;
};

// TODO: overload equality operator for Polygon, taking into account possible
// offset and wrap-around

}  // namespace geometry_2d
}  // namespace pathviz
