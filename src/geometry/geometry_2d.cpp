#include "pathviz/geometry/geometry_2d.hpp"

namespace pathviz {
namespace geometry_2d {

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