#pragma once

#include <set>
#include <vector>

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

}  // namespace visibility_map
}  // namespace pathviz
