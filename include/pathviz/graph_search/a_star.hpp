#pragma once

#include <stack>

#include "graphlib/geometry/graph_2d.hpp"

#include "pathviz/visualization/interface.hpp"

namespace pathviz {
namespace graph_search {

struct AnimationManager {
  AnimationManager(
      ros::Publisher* marker_pub, double update_rate_ms,
      const visualization::PointDescriptor& fringe_descriptor,
      const visualization::PointDescriptor& current_vertex_descriptor,
      const visualization::LineDescriptor& relaxed_edges_descriptor);

  ros::Publisher* marker_pub_;
  double update_rate_ms_;
  std::unique_ptr<visualization::PointListManager> fringe_, current_vertex_,
      relaxed_vertices_;
  std::unique_ptr<visualization::LineListManager> relaxed_edges_;
};

// A* algorithm for single-source shortest non-negative weighted paths. Uses
// Euclidean distance from destination as a heuristic.
//
// The default heuristic multiplier of 1 guarantees admissibility (the algorithm
// will always return the optimal path). Higher heuristic multipliers make the
// algorithm greedier, which means it will probably find a path to the goal
// faster, but the path might be suboptimal. A heuristic multiplier of 0 results
// in Dijkstra's algorithm.
std::stack<const graphlib::Vertex2d*> a_star(
    graphlib::Graph2d* graph, const graphlib::Vertex2d* search_root,
    const graphlib::Vertex2d* destination, double heuristic_multiplier = 1,
    AnimationManager* animation_manager = nullptr);

// Dijkstra's algorithm for single-source shortest non-negative weighted paths.
// This is actually an alias that runs the a_star function with a heuristic
// multiplier of 0.
std::stack<const graphlib::Vertex2d*> dijkstra(
    graphlib::Graph2d* graph, const graphlib::Vertex2d* search_root,
    const graphlib::Vertex2d* destination,
    AnimationManager* animation_manager = nullptr);

}  // namespace graph_search
}  // namespace pathviz
