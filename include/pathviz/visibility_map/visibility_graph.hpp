#pragma once

#include <map>
#include <set>
#include <vector>

#include "graphlib/geometry/graph_2d.hpp"
#include "pathviz/geometry/geometry_2d.hpp"

namespace pathviz {
namespace visibility_map {

// Current implementation assumes no obstacles overlap or share common
// vertices.
class Terrain {
 public:
  Terrain() = default;
  Terrain(std::vector<geometry_2d::Polygon> obstacles);

  void AddObstacle(const geometry_2d::Polygon& obstacle);

  const std::vector<geometry_2d::Polygon>& AllObstacles() const;
  std::set<geometry_2d::Point> AllVertices() const;

  // Get obstacle Polygon associated with given vertex.
  const geometry_2d::Polygon& GetObstacle(
      const geometry_2d::Point& vertex) const;

 private:
  std::vector<geometry_2d::Polygon> obstacles_;

  // Zero-indexed map from vertices to the index in which its associated
  // obstacle is stored in the obstacles_ vector.
  std::map<geometry_2d::Point, int> obstacle_index_;
};

std::set<geometry_2d::Point> get_visible_vertices(
    const Terrain& terrain, const geometry_2d::Point& source,
    bool verbose = false);

// graphlib::Graph2d get_visibility_graph(const Terrain& terrain,
//                                        bool verbose = false);

}  // namespace visibility_map
}  // namespace pathviz
