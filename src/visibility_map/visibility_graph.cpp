#include "pathviz/visibility_map/visibility_graph.hpp"

namespace pathviz {
namespace visibility_map {

// TODO: support handling overlapping obstacles

Terrain::Terrain(std::vector<geometry_2d::Polygon> obstacles)
    : obstacles_(obstacles) {}

void Terrain::AddObstacle(const geometry_2d::Polygon& obstacle) {
  obstacles_.push_back(obstacle);
}

const std::vector<geometry_2d::Polygon>& Terrain::GetObstacles() {
  return obstacles_;
}

std::set<geometry_2d::Point> Terrain::GetAllVertices() {
  std::set<geometry_2d::Point> vertices;
  for (const auto& obstacle : obstacles_) {
    vertices.insert(obstacle.polygon_.cbegin(), obstacle.polygon_.cend());
  }
  return vertices;
}

// bool is_visible(const geometry_2d::Point& source,
//                 const geometry_2d::Point& target,
//                 const std::vector<geometry_2d::Edge>& active_edges) {
//   // stuff
// }

// std::set<geometry_2d::Point> get_visible_vertices(
//     const Terrain& terrain, const geometry_2d::Point& source) {
//   // stuff
// }

// graphlib::Graph2d get_visibility_graph(const Terrain& terrain) {
//   // stuff
// }

}  // namespace visibility_map
}  // namespace pathviz
