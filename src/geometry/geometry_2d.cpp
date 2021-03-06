#include "pathviz/geometry/geometry_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace pathviz {
namespace geometry_2d {

const Point& LineSegment::Other(const Point& p) {
  if (p == from) {
    return to;
  } else if (p == to) {
    return from;
  }
  throw std::runtime_error(
      "geometry_2d::LineSegment error: input Point p doesn't exist");
}

LineSegment reverse(const LineSegment& e) { return LineSegment{e.to, e.from}; }

double distance(const Point& p1, const Point& p2) {
  return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

double length(const LineSegment& e) { return distance(e.from, e.to); }

double angle_from_horizontal(const Point& source, const Point& target) {
  double dx = target.x - source.x, dy = target.y - source.y;
  return std::atan2(dy, dx);
}

double angle_from_horizontal(const LineSegment& e) {
  return angle_from_horizontal(e.from, e.to);
}

bool is_horizontal(const LineSegment& e) { return e.from.y == e.to.y; }

bool is_vertical(const LineSegment& e) { return e.from.x == e.to.x; }

double slope(const LineSegment& e) {
  if (is_vertical(e)) {
    throw std::runtime_error(
        "geometry_2d::slope error: input LineSegment is vertical");
  }
  return (e.to.y - e.from.y) / (e.to.x - e.from.x);
}

double y_intercept(const LineSegment& e) {
  if (is_vertical(e)) {
    throw std::runtime_error(
        "geometry_2d::y_intercept error: input LineSegment is vertical");
  }
  if (e.from.y == e.to.y) {
    return e.from.y;
  }
  return e.to.y - slope(e) * e.to.x;
}

double x_intercept(const LineSegment& e) {
  if (is_horizontal(e)) {
    throw std::runtime_error(
        "geometry_2d::x_intercept error: input LineSegment is horizontal");
  }
  if (e.from.x == e.to.x) {
    return e.from.x;
  }
  return -y_intercept(e) / slope(e);
}

// TODO: find ways to make this more numerically robust!
bool is_intersecting(const LineSegment& e1, const LineSegment& e2) {
  if (e1.from == e2.from || e1.from == e2.to || e1.to == e2.from ||
      e1.to == e2.to) {
    return false;
  }

  if (is_vertical(e1) && is_vertical(e2)) {
    return false;  // colinear
  } else if (is_vertical(e1)) {
    double y_intersect = slope(e2) * e1.from.x + y_intercept(e2);
    return std::min(e2.from.y, e2.to.y) < y_intersect &&
           y_intersect < std::max(e2.from.y, e2.to.y);
  } else if (is_vertical(e2)) {
    double y_intersect = slope(e1) * e2.from.x + y_intercept(e1);
    return std::min(e1.from.y, e1.to.y) < y_intersect &&
           y_intersect < std::max(e1.from.y, e1.to.y);
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

void MinBoundingBox::AddPoint(const Point& p) {
  if (p.x < left_bound) {
    left_bound = p.x;
  }
  if (p.x > right_bound) {
    right_bound = p.x;
  }
  if (p.y < bottom_bound) {
    bottom_bound = p.y;
  }
  if (p.y > top_bound) {
    top_bound = p.y;
  }
}

void MinBoundingBox::AddBoundingBox(const MinBoundingBox& other) {
  if (other.left_bound < left_bound) {
    left_bound = other.left_bound;
  }
  if (other.right_bound > right_bound) {
    right_bound = other.right_bound;
  }
  if (other.bottom_bound < bottom_bound) {
    bottom_bound = other.bottom_bound;
  }
  if (other.top_bound > top_bound) {
    top_bound = other.top_bound;
  }
}

// The current implementation assumes that the order of points inputted is
// counter-clockwise.
// TODO: automatically reverse order of points if detected clockwise.
Polygon::Polygon(std::vector<Point> polygon) {
  for (const Point& p : polygon) {
    polygon_.push_back(p);
    bounding_box_.AddPoint(p);
  }

  for (auto iter = polygon_.cbegin() + 1; iter != polygon_.cend(); ++iter) {
    all_edges_.push_back(LineSegment{*(iter - 1), *iter});
  }
  all_edges_.push_back(
      LineSegment{polygon_.front(), polygon_.back()});  // closing edge

  if (polygon_.size() < 3) {
    throw std::runtime_error(
        "geometry_2d::Polygon error: polygon must have at least 3 points");
  }
}

std::pair<LineSegment, LineSegment> Polygon::IncidentEdges(Point from) const {
  auto iter = std::find(polygon_.cbegin(), polygon_.cend(), from);
  if (iter == polygon_.cend()) {
    throw std::runtime_error(
        "geometry_2d::Polygon error: tried to access a point that doesn't "
        "exist");
  }

  // All three cases here are mutually exclusive because a polygon has at
  // least 3 points.
  if (iter == polygon_.cbegin()) {
    LineSegment e1{from, *(polygon_.cend() - 1)};
    LineSegment e2{from, *(iter + 1)};
    return std::pair<LineSegment, LineSegment>(e1, e2);
  } else if (iter == polygon_.cend() - 1) {
    LineSegment e1{from, *(iter - 1)};
    LineSegment e2{from, *(polygon_.cbegin())};
    return std::pair<LineSegment, LineSegment>(e1, e2);
  } else {
    LineSegment e1{from, *(iter - 1)};
    LineSegment e2{from, *(iter + 1)};
    return std::pair<LineSegment, LineSegment>(e1, e2);
  }
}

Terrain::Terrain(std::vector<Polygon> obstacles) {
  for (const auto& obstacle : obstacles) {
    AddObstacle(obstacle);
  }
}

void Terrain::AddObstacle(const Polygon& obstacle) {
  for (const Point& p : obstacle.GetPolygon()) {
    obstacle_index_.insert(std::make_pair(p, obstacles_.size()));
  }
  obstacles_.push_back(obstacle);
}

const std::vector<Polygon>& Terrain::AllObstacles() const { return obstacles_; }

std::set<Point> Terrain::AllVertices() const {
  std::set<Point> vertices;
  for (const auto& obstacle : obstacles_) {
    vertices.insert(obstacle.GetPolygon().cbegin(),
                    obstacle.GetPolygon().cend());
  }
  return vertices;
}

int Terrain::GetObstacleIndex(const Point& vertex) const {
  if (obstacle_index_.find(vertex) == obstacle_index_.end()) {
    throw std::runtime_error(
        "visibility_graph::Terrain error: tried to access a vertex that "
        "doesn't exist");
  }
  return obstacle_index_.at(vertex);
}

const Polygon& Terrain::GetObstacle(const Point& vertex) const {
  return obstacles_.at(GetObstacleIndex(vertex));
}

}  // namespace geometry_2d
}  // namespace pathviz
