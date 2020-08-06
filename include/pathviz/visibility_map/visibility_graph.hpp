#pragma once

#include <set>
#include <vector>

#include "graphlib/geometry/graph_2d.hpp"
#include "pathviz/geometry/geometry_2d.hpp"

namespace pathviz {
namespace visibility_map {

class Terrain {
 public:
  Terrain() = default;
  Terrain(std::vector<geometry_2d::Polygon> obstacles);

  void AddObstacle(const geometry_2d::Polygon& obstacle);

  const std::vector<geometry_2d::Polygon>& GetObstacles();

  std::set<geometry_2d::Point> GetAllVertices();

 private:
  std::vector<geometry_2d::Polygon> obstacles_;
};

// bool is_visible(const geometry_2d::Point& source,
//                 const geometry_2d::Point& target,
//                 const std::vector<geometry_2d::Edge>& active_edges);

// std::set<geometry_2d::Point> get_visible_vertices(
//     const Terrain& terrain, const geometry_2d::Point& source);

// graphlib::Graph2d get_visibility_graph(const Terrain& terrain);

}  // namespace visibility_map
}  // namespace pathviz
