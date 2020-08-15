#pragma once

#include <map>
#include <memory>
#include <set>
#include <vector>

#include "graphlib/geometry/graph_2d.hpp"
#include "pathviz/geometry/geometry_2d.hpp"
#include "pathviz/visualization/interface.hpp"

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

  int GetObstacleIndex(const geometry_2d::Point& vertex) const;

  const geometry_2d::Polygon& GetObstacle(
      const geometry_2d::Point& vertex) const;

 private:
  std::vector<geometry_2d::Polygon> obstacles_;

  // Zero-indexed map from vertices to the index in which its associated
  // obstacle is stored in the obstacles_ vector.
  std::map<geometry_2d::Point, int> obstacle_index_;
};

struct AnimationManager {
  AnimationManager(
      ros::Publisher* marker_pub, double update_rate_ms,
      const visualization::PointListDescriptor& current_source_descriptor,
      const visualization::PointListDescriptor& current_target_descriptor,
      const visualization::LineListDescriptor& current_edge_descriptor,
      const visualization::LineListDescriptor& active_edges_descriptor,
      const visualization::LineListDescriptor& valid_edges_descriptor,
      const visualization::LineListDescriptor& invalid_edges_descriptor);

  ros::Publisher* marker_pub_;
  double update_rate_ms_;
  std::unique_ptr<visualization::PointListManager> current_source_vertex_,
      current_target_vertex_;
  std::unique_ptr<visualization::LineListManager> current_edge_, active_edges_,
      valid_edges_, invalid_edges_, total_valid_edges_;
};

std::set<geometry_2d::Point> get_visible_vertices(
    const Terrain& terrain, const geometry_2d::Point& source,
    bool verbose = false, AnimationManager* animation_manager = nullptr);

graphlib::Graph2d get_visibility_graph(
    const Terrain& terrain, bool verbose = false,
    AnimationManager* animation_manager = nullptr);

}  // namespace visibility_map
}  // namespace pathviz
