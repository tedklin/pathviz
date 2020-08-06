#include "pathviz/geometry/geometry_2d.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>

namespace pathviz {
namespace geometry_2d {

Point Edge::Other(Point p) {
  if (p == from) {
    return to;
  } else if (p == to) {
    return from;
  }
  throw std::runtime_error("Edge error: input Point p doesn't exist");
}

double distance(const Point& p1, const Point& p2) {
  return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

double length(const Edge& e) { return distance(e.from, e.to); }

double angle_from_horizontal(const Point& source, const Point& target) {
  double dx = target.x - source.x, dy = target.y - source.y;
  return std::atan2(dy, dx);
}

double angle_from_horizontal(const Edge& e) {
  return angle_from_horizontal(e.from, e.to);
}

double slope(const Edge& e) {
  return (e.to.y - e.from.y) / (e.to.x - e.from.x);
}

double y_intercept(const Edge& e) { return e.to.y - slope(e) * e.to.x; }

// Does not count common endpoints or any form of colinearity as an
// intersection.
bool is_intersecting(const Edge& e1, const Edge& e2) {
  bool e1_is_vertical = e1.from.x == e1.to.x;
  bool e2_is_vertical = e2.from.x == e2.to.x;

  if (e1_is_vertical && e2_is_vertical) {
    return false;  // colinear
  } else if (e1_is_vertical) {
    double y_intersect = slope(e2) * e1.from.x + y_intercept(e2);
    return std::min(e1.from.y, e1.to.y) < y_intersect &&
           y_intersect < std::max(e1.from.y, e1.to.y);
  } else if (e2_is_vertical) {
    double y_intersect = slope(e1) * e2.from.x + y_intercept(e1);
    return std::min(e2.from.y, e2.to.y) < y_intersect &&
           y_intersect < std::max(e2.from.y, e2.to.y);
  }

  if (slope(e1) == slope(e2)) {
    return false;  // colinear
  } else {
    double x_intersect =
        (y_intercept(e2) - y_intercept(e1)) / (slope(e1) - slope(e2));

    std::pair<double, double> overlapping_x_interval = std::make_pair(
        std::max(std::min(e1.from.x, e1.to.x), std::min(e2.from.x, e2.to.x)),
        std::min(std::max(e1.from.x, e1.to.x), std::max(e2.from.x, e2.to.x)));

    return overlapping_x_interval.first < x_intersect &&
           x_intersect < overlapping_x_interval.second;
  }
}

Polygon::Polygon(std::vector<Point> polygon) : polygon_(polygon) {
  if (polygon_.size() < 3) {
    throw std::runtime_error(
        "Polygon error: polygon must have at least 3 points");
  }
}

std::pair<Edge, Edge> Polygon::IncidentEdges(Point from) const {
  auto iter = std::find(polygon_.cbegin(), polygon_.cend(), from);
  if (iter == polygon_.cend()) {
    throw std::runtime_error(
        "Polygon error: tried to access a point that doesn't exist");
  }

  // All three cases here are mutually exclusive because a polygon has at
  // least 3 points.
  if (from == polygon_.front()) {
    Edge e1{from, *(polygon_.cend() - 1)};
    Edge e2{from, *(iter + 1)};
    return std::pair<Edge, Edge>(e1, e2);
  } else if (from == polygon_.back()) {
    Edge e1{from, *(iter - 1)};
    Edge e2{from, *(polygon_.cbegin())};
    return std::pair<Edge, Edge>(e1, e2);
  } else {
    Edge e1{from, *(iter - 1)};
    Edge e2{from, *(iter + 1)};
    return std::pair<Edge, Edge>(e1, e2);
  }
}

}  // namespace geometry_2d
}  // namespace pathviz