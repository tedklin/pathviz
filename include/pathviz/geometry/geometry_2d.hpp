#pragma once

#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

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
  const Point& Other(const Point& p);
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

constexpr double pi = 3.14159265358979323846;

// Angle (-pi, pi] from positive horizontal axis
// w.r.t. an input source Point.
double angle_from_horizontal(const Point& source, const Point& target);
// w.r.t. an input LineSegment's "from" Point.
double angle_from_horizontal(const LineSegment& e);

bool is_horizontal(const LineSegment& e);
bool is_vertical(const LineSegment& e);

double slope(const LineSegment& e);
double y_intercept(const LineSegment& e);
double x_intercept(const LineSegment& e);

// Does not count common endpoints as intersections. Treats any form of
// colinearity as NOT an intersection. Note that this function may be subject to
// floating point arithmetic error, especially near endpoints.
// TODO: find ways to make this more numerically robust!
bool is_intersecting(const LineSegment& e1, const LineSegment& e2);

struct MinBoundingBox {
  double left_bound = std::numeric_limits<double>::infinity(),
         right_bound = -std::numeric_limits<double>::infinity(),
         bottom_bound = std::numeric_limits<double>::infinity(),
         top_bound = -std::numeric_limits<double>::infinity();

  void AddPoint(const Point& p);
  void AddBoundingBox(const MinBoundingBox& other);
};

// Adjacent pairs of vertices represent edges of the polygon; the vertices on
// each end of the vector close the polygon.
//
// The current implementation assumes that the order of points inputted is
// counterclockwise.
class Polygon {
 public:
  Polygon(std::vector<Point> polygon);

  const std::vector<Point>& GetPolygon() const { return polygon_; }
  const std::vector<LineSegment>& AllEdges() const { return all_edges_; };
  const MinBoundingBox& BoundingBox() const { return bounding_box_; };

  // Incident edges pointing outwards from the given point.
  //
  // Maintains counterclockwise polygon ordering (if we rotate about the input
  // point such that a ray drawn between the incident edges from the input point
  // towards the inside of the polygon aligns with the positive y-axis, then the
  // incident edge to the left of the y-axis would be the first in the pair
  // returned by this function).
  std::pair<LineSegment, LineSegment> IncidentEdges(Point from) const;

 private:
  std::vector<Point> polygon_;

  std::vector<LineSegment> all_edges_;
  MinBoundingBox bounding_box_;
};
// TODO: overload equality operator for Polygon, taking into account possible
// offset and wrap-around

// The current implementation of Terrain assumes no obstacles overlap or share
// common vertices.
class Terrain {
 public:
  Terrain() = default;
  Terrain(std::vector<Polygon> obstacles);

  void AddObstacle(const Polygon& obstacle);

  const std::vector<Polygon>& AllObstacles() const;
  std::set<geometry_2d::Point> AllVertices() const;

  int GetObstacleIndex(const Point& vertex) const;
  const Polygon& GetObstacle(const Point& vertex) const;

 private:
  std::vector<Polygon> obstacles_;

  // Zero-indexed map from vertices to the index in which its associated
  // obstacle is stored in the obstacles_ vector.
  std::map<Point, int> obstacle_index_;
};

}  // namespace geometry_2d
}  // namespace pathviz
