// Adapted from the implementation of Dijkstra's algorithm in
// graphlib/algo/weighted_paths.cpp.

#include "pathviz/graph_search/a_star.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

namespace pathviz {
namespace graph_search {

AnimationManager::AnimationManager(
    ros::Publisher* marker_pub, double update_rate_ms,
    const visualization::PointDescriptor& fringe_descriptor,
    const visualization::PointDescriptor& current_vertex_descriptor,
    const visualization::PointDescriptor& relaxed_vertices_descriptor,
    const visualization::LineDescriptor& relaxed_edges_descriptor)
    : marker_pub_(marker_pub), update_rate_ms_(update_rate_ms) {
  fringe_ = std::make_unique<visualization::PointListManager>(
      marker_pub, "fringe", fringe_descriptor);
  current_vertex_ = std::make_unique<visualization::PointListManager>(
      marker_pub, "current_vertex", current_vertex_descriptor);
  relaxed_vertices_ = std::make_unique<visualization::PointListManager>(
      marker_pub, "relaxed_vertices", relaxed_vertices_descriptor);
  relaxed_edges_ = std::make_unique<visualization::LineListManager>(
      marker_pub, "relaxed_edges", relaxed_edges_descriptor);
}

using graphlib::Graph2d;
using graphlib::Vertex2d;

// Returns a stack for the path currently encoded in the parent members of each
// Vertex.
std::stack<const Vertex2d*> get_path(Graph2d* graph,
                                     const Vertex2d* search_root,
                                     const Vertex2d* destination) {
  const Vertex2d* v = destination;
  std::stack<const Vertex2d*> s;
  s.push(v);

  while (v != search_root) {
    if (v) {
      v = dynamic_cast<const Vertex2d*>(v->parent_);
      s.push(v);
    } else {
      std::cerr << "No path between " << search_root->name_ << " and "
                << destination->name_ << "\n\n";
      return std::stack<const Vertex2d*>();
    }
  }
  return s;
}

// Maps each vertex to its currently-known shortest distance to the search
// root.
std::map<const Vertex2d*, double> dist_to_root;

void setup_dist_to_root(Graph2d* graph, const Vertex2d* search_root) {
  dist_to_root.clear();
  for (const auto& v : graph->GetAdjacencyMap()) {
    if (v.first == search_root) {
      dist_to_root[dynamic_cast<const Vertex2d*>(v.first)] = 0;
    } else {
      dist_to_root[dynamic_cast<const Vertex2d*>(v.first)] =
          std::numeric_limits<double>::infinity();
    }
  }
}

std::stack<const graphlib::Vertex2d*> a_star(
    Graph2d* graph, const Vertex2d* search_root, const Vertex2d* destination,
    double heuristic_multiplier, AnimationManager* animation_manager) {
  setup_dist_to_root(graph, search_root);

  auto heuristic = [&destination](const Vertex2d* v) -> double {
    // Euclidean distance heuristic.
    return graphlib::distance_2d(*destination, *v);
  };
  auto pq_ordering = [&search_root, &heuristic, &heuristic_multiplier](
                         const Vertex2d* v1, const Vertex2d* v2) -> bool {
    if (dist_to_root.at(v1) == dist_to_root.at(v2)) {
      // Handle case where both dist_to_root values are infinity.
      return (heuristic(v1) * heuristic_multiplier) >
             (heuristic(v2) * heuristic_multiplier);
    }
    return (heuristic(v1) * heuristic_multiplier + dist_to_root.at(v1)) >
           (heuristic(v2) * heuristic_multiplier + dist_to_root.at(v2));
  };

  std::vector<const Vertex2d*> min_heap;
  min_heap.push_back(search_root);
  if (animation_manager) {
    animation_manager->fringe_->AddPoint({search_root->x_, search_root->y_});
    animation_manager->fringe_->Publish();
    visualization::sleep_ms(animation_manager->update_rate_ms_ * 2);
  }

  while (!min_heap.empty()) {
    std::pop_heap(min_heap.begin(), min_heap.end(), pq_ordering);
    const Vertex2d* v1 = min_heap.back();
    min_heap.pop_back();
    if (animation_manager) {
      animation_manager->fringe_->RemovePoint({v1->x_, v1->y_});
      animation_manager->fringe_->Publish();

      animation_manager->relaxed_vertices_->AddPoint({v1->x_, v1->y_});
      animation_manager->relaxed_vertices_->Publish();

      animation_manager->current_vertex_->Clear();
      animation_manager->current_vertex_->AddPoint({v1->x_, v1->y_});
      animation_manager->current_vertex_->Publish();
      visualization::sleep_ms(animation_manager->update_rate_ms_ * 2);
    }

    if (v1 == destination) break;

    for (auto& adj : graph->GetAdjacentSet(v1)) {
      const Vertex2d* v2 = dynamic_cast<const Vertex2d*>(adj.first);
      double weight = adj.second;

      if (dist_to_root.at(v2) > dist_to_root.at(v1) + weight) {
        if (animation_manager) {
          if (v2->parent_) {
            const Vertex2d* old_parent =
                dynamic_cast<const Vertex2d*>(v2->parent_);
            animation_manager->relaxed_edges_->RemoveLine(
                {{old_parent->x_, old_parent->y_}, {v2->x_, v2->y_}});
          }
        }
        dist_to_root.at(v2) = dist_to_root.at(v1) + weight;
        v2->parent_ = v1;
        if (std::find(min_heap.begin(), min_heap.end(), v2) != min_heap.end()) {
          // If v2 is already in the min-heap, do a complete reheapify of the
          // underlying vector with v2's updated "g_dist_to_root" value.
          std::make_heap(min_heap.begin(), min_heap.end(), pq_ordering);
        } else {
          // If v2 is not yet in the min-heap, push it to the back of the
          // underlying vector, then bubble it up to its proper heap placement.
          min_heap.push_back(v2);
          std::push_heap(min_heap.begin(), min_heap.end(), pq_ordering);
        }
        if (animation_manager) {
          animation_manager->fringe_->AddPoint({v2->x_, v2->y_});
          animation_manager->relaxed_edges_->AddLine(
              {{v1->x_, v1->y_}, {v2->x_, v2->y_}});
        }
      }
    }
    if (animation_manager) {
      animation_manager->fringe_->Publish();
      animation_manager->relaxed_edges_->Publish();

      animation_manager->current_vertex_->Clear();
      animation_manager->current_vertex_->Publish();
      visualization::sleep_ms(animation_manager->update_rate_ms_);
    }
  }

  if (animation_manager) {
    animation_manager->fringe_->Clear();
    animation_manager->fringe_->Publish();

    animation_manager->current_vertex_->Clear();
    animation_manager->current_vertex_->Publish();

    visualization::sleep_ms(1000);

    // animation_manager->relaxed_edges_->Clear();
    // animation_manager->relaxed_edges_->Publish();

    // animation_manager->relaxed_vertices_->Clear();
    // animation_manager->relaxed_vertices_->Publish();
  }

  return get_path(graph, search_root, destination);
}

std::stack<const graphlib::Vertex2d*> dijkstra(
    graphlib::Graph2d* graph, const graphlib::Vertex2d* search_root,
    const graphlib::Vertex2d* destination,
    AnimationManager* animation_manager) {
  return a_star(graph, search_root, destination, 0, animation_manager);
}

}  // namespace graph_search
}  // namespace pathviz
