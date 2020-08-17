#pragma once

#include <memory>
#include <set>
#include <vector>

#include "graphlib/geometry/graph_2d.hpp"
#include "pathviz/geometry/geometry_2d.hpp"
#include "pathviz/visualization/interface.hpp"

namespace pathviz {
namespace visibility_map {

struct AnimationManager {
  AnimationManager(
      ros::Publisher* marker_pub, double update_rate_ms,
      const visualization::PointDescriptor& current_source_descriptor,
      const visualization::PointDescriptor& current_target_descriptor,
      const visualization::LineDescriptor& current_edge_descriptor,
      const visualization::LineDescriptor& active_edges_descriptor,
      const visualization::LineDescriptor& valid_edges_descriptor,
      const visualization::LineDescriptor& invalid_edges_descriptor);

  ros::Publisher* marker_pub_;
  double update_rate_ms_;
  std::unique_ptr<visualization::PointListManager> current_source_vertex_,
      current_target_vertex_;
  std::unique_ptr<visualization::LineListManager> current_edge_, active_edges_,
      valid_edges_, invalid_edges_, total_valid_edges_;
};

std::set<geometry_2d::Point> get_visible_vertices(
    const geometry_2d::Terrain& terrain, const geometry_2d::Point& source,
    bool verbose = false, AnimationManager* animation_manager = nullptr);

graphlib::Graph2d get_visibility_graph(
    const geometry_2d::Terrain& terrain, const geometry_2d::Point& start,
    const geometry_2d::Point& goal, bool verbose = false,
    AnimationManager* animation_manager = nullptr);

}  // namespace visibility_map
}  // namespace pathviz
