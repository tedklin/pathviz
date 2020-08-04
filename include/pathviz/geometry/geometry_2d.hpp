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
  Point(double x, double y) : x_(x), y_(y) {}
  const double x_, y_;
};

inline bool operator==(const Point& lhs, const Point& rhs) {
  return lhs.x_ == rhs.x_ && lhs.y_ == rhs.y_;
}

inline bool operator!=(const Point& lhs, const Point& rhs) {
  return !operator==(lhs, rhs);
}

inline std::string to_string(const Point& p) {
  return "(" + std::to_string(p.x_) + ", " + std::to_string(p.y_) + ")";
}

inline double distance_2d(const Point& p1, const Point& p2) {
  return std::sqrt(std::pow(p1.x_ - p2.x_, 2) + std::pow(p1.y_ - p2.y_, 2));
}

struct Edge {
  Edge(Point to, Point from) : to_(to), from_(from) {}
  const Point to_, from_;
};

struct Polygon {
  Polygon(std::vector<Point> polygon) : polygon_(polygon) {
    if (polygon_.size() < 3) {
      throw std::runtime_error(
          "Polygon error: polygon must have at least 3 points");
    }
  }

  std::pair<Edge, Edge> IncidentEdges(Point from) const {
    auto iter = std::find(polygon_.cbegin(), polygon_.cend(), from);
    if (iter == polygon_.cend()) {
      throw std::runtime_error(
          "Polygon error: tried to access a point that doesn't exist");
    }

    // All three cases here are mutually exclusive because a polygon has at
    // least 3 points.
    if (from == polygon_.front()) {
      Edge e1(from, *(polygon_.cend() - 1));
      Edge e2(from, *(iter + 1));
      return std::pair<Edge, Edge>(e1, e2);
    } else if (from == polygon_.back()) {
      Edge e1(from, *(iter - 1));
      Edge e2(from, *(polygon_.cbegin()));
      return std::pair<Edge, Edge>(e1, e2);
    } else {
      Edge e1(from, *(iter - 1));
      Edge e2(from, *(iter + 1));
      return std::pair<Edge, Edge>(e1, e2);
    }
  }

  // Adjacent vertices represent edges of the polygon; the vertices on the
  // ends of the vector close the polygon.
  const std::vector<Point> polygon_;
};  // namespace geometry_2d

}  // namespace geometry_2d
}  // namespace pathviz
