#include "graphlib/algo/weighted_paths.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>

#include "graphlib/algo/dfs.hpp"

namespace graphlib {

// Maps each vertex to its currently-known shortest distance to the search
// root.
std::map<std::shared_ptr<const Vertex>, double> g_dist_to_root;

// Several algorithms implemented here start by setting the known distance to
// the search root to 0 and the known distance to all other (yet undiscovered)
// vertices to infinity.
void setup_dist_to_root(Graph* graph,
                        std::shared_ptr<const Vertex> search_root) {
  g_dist_to_root.clear();
  for (const auto& v : graph->GetVertexMap()) {
    if (v.first == search_root) {
      g_dist_to_root[v.first] = 0;
    } else {
      g_dist_to_root[v.first] = std::numeric_limits<double>::infinity();
    }
  }
}

// Comparator to emulate Dijkstra's min-heap with an underlying std::vector.
struct GreaterDistToRoot {
  bool operator()(std::shared_ptr<const Vertex> lhs,
                  std::shared_ptr<const Vertex> rhs) const {
    return g_dist_to_root.at(lhs) > g_dist_to_root.at(rhs);
  }
};

// Dijkstra's algorithm uses a min-heap to keep track of the next non-tree
// Vertex closest to the search root. std::priority_queue doesn't work here
// because we need the ability to change the "value" (distance to search root)
// of a Vertex already in the heap and reheapify on the go. To support this
// functionality, we use std::make_heap, std::push_heap, and std::pop_heap on an
// underlying std::vector.
void dijkstra(Graph* graph, std::shared_ptr<const Vertex> search_root,
              std::shared_ptr<const Vertex> destination) {
  setup_dist_to_root(graph, search_root);

  std::vector<std::shared_ptr<const Vertex>> min_heap;
  min_heap.push_back(search_root);

  while (!min_heap.empty()) {
    // Bubble the smallest element (top of min-heap) down to the end of the
    // underlying vector, store it in v1, then pop it.
    std::pop_heap(min_heap.begin(), min_heap.end(), GreaterDistToRoot());
    std::shared_ptr<const Vertex> v1 = min_heap.back();
    min_heap.pop_back();

    if (v1 == destination) return;

    for (auto& adj : graph->GetAdjacentSet(*v1)) {
      std::shared_ptr<const Vertex> v2 = adj.first;
      double weight = adj.second;

      if (g_dist_to_root.at(v2) > g_dist_to_root.at(v1) + weight) {
        g_dist_to_root.at(v2) = g_dist_to_root.at(v1) + weight;
        v2->parent_ = v1;

        if (std::find(min_heap.begin(), min_heap.end(), v2) != min_heap.end()) {
          // If v2 is already in the min-heap, do a complete reheapify of the
          // underlying vector with v2's updated "g_dist_to_root" value.
          std::make_heap(min_heap.begin(), min_heap.end(), GreaterDistToRoot());
        } else {
          // If v2 is not yet in the min-heap, push it to the back of the
          // underlying vector, then bubble it up to its proper heap placement.
          min_heap.push_back(v2);
          std::push_heap(min_heap.begin(), min_heap.end(), GreaterDistToRoot());
        }
      }
    }
  }
}

// UNTESTED!
// This is analogous to Dijkstra's algorithm, but instead of a min-heap keeping
// track of which vertex to process next, we simply take vertices in topological
// order. (Sedgewick)
//
// By processing vertices in topological order, we ensure that each edge is only
// relaxed once. When a vertex v is considered, all edges going into v will have
// already been relaxed (by the nature of topological sort). This means
// dist_to_root[v] can't change after v is processed. Therefore, all edges
// coming out of v will only be relaxed when v is being processed (and never
// after).
void dag_paths(Graph* graph, std::shared_ptr<const Vertex> search_root,
               std::shared_ptr<const Vertex> destination) {
  setup_dist_to_root(graph, search_root);

  std::stack<std::shared_ptr<const Vertex>> s = topological_sort(graph);
  while (!s.empty()) {
    std::shared_ptr<const Vertex> v1 = s.top();
    s.pop();

    if (v1 == destination) return;

    for (auto& adj : graph->GetAdjacentSet(*v1)) {
      std::shared_ptr<const Vertex> v2 = adj.first;
      double weight = adj.second;

      if (g_dist_to_root.at(v2) > g_dist_to_root.at(v1) + weight) {
        g_dist_to_root.at(v2) = g_dist_to_root.at(v1) + weight;
        v2->parent_ = v1;
      }
    }
  }
}

void bellman_ford(Graph* graph, std::shared_ptr<const Vertex> search_root) {
  setup_dist_to_root(graph, search_root);
  std::queue<std::shared_ptr<const Vertex>> q;
  std::map<std::shared_ptr<const Vertex>, bool>
      on_q;  // quick lookup for if vertex is on queue
  for (const auto& v : graph->GetVertexMap()) {
    on_q[v.first] = false;
  }

  q.push(search_root);
  on_q.at(search_root) = true;

  while (!q.empty()) {
    std::shared_ptr<const Vertex> v1 = q.front();
    q.pop();
    on_q.at(v1) = false;

    for (auto& adj : graph->GetAdjacentSet(*v1)) {
      std::shared_ptr<const Vertex> v2 = adj.first;
      double weight = adj.second;

      if (g_dist_to_root.at(v2) > g_dist_to_root.at(v1) + weight) {
        g_dist_to_root.at(v2) = g_dist_to_root.at(v1) + weight;
        v2->parent_ = v1;

        if (!on_q.at(v2)) {
          q.push(v2);
          on_q.at(v2) = true;
        }
      }
    }
  }
}

// Returns a stack for the path currently encoded in the parent members of each
// Vertex. Analogous to shortest_unweighted_path generation in bfs.cpp
std::stack<std::shared_ptr<const Vertex>> get_path_helper(
    Graph* graph, std::shared_ptr<const Vertex> search_root,
    std::shared_ptr<const Vertex> destination) {
  std::shared_ptr<const Vertex> v = destination;
  std::stack<std::shared_ptr<const Vertex>> s;
  s.push(v);

  while (v != search_root) {
    if (v) {
      v = v->parent_;
      s.push(v);
    } else {
      std::cerr << "No path between " << search_root->name_ << " and "
                << destination->name_ << "\n\n";
      return std::stack<std::shared_ptr<const Vertex>>();
    }
  }
  return s;
}

std::stack<std::shared_ptr<const Vertex>> shortest_pos_weight_path(
    Graph* graph, std::shared_ptr<const Vertex> search_root,
    std::shared_ptr<const Vertex> destination) {
  dijkstra(graph, search_root, destination);
  return get_path_helper(graph, search_root, destination);
}

std::stack<std::shared_ptr<const Vertex>> shortest_weighted_path(
    Graph* graph, std::shared_ptr<const Vertex> search_root,
    std::shared_ptr<const Vertex> destination) {
  bellman_ford(graph, search_root);
  return get_path_helper(graph, search_root, destination);
}

DistanceMatrix floyd_warshall(Graph* graph) {
  DistanceMatrix dist_matrix;
  for (const auto& v1 : graph->GetVertexMap()) {
    for (const auto& v2 : graph->GetVertexMap()) {
      if (v1 == v2) {
        dist_matrix[v1.first][v2.first] = 0;
      } else if (graph->EdgeExists(*(v1.first), *(v2.first))) {
        dist_matrix[v1.first][v2.first] =
            graph->EdgeWeight(*(v1.first), *(v2.first));
      } else {
        dist_matrix[v1.first][v2.first] =
            std::numeric_limits<double>::infinity();
      }
    }
  }

  for (const auto& n : dist_matrix) {
    std::shared_ptr<const Vertex> k = n.first;
    for (auto& i : dist_matrix) {
      std::shared_ptr<const Vertex> v1 = i.first;
      for (auto& j : i.second) {
        std::shared_ptr<const Vertex> v2 = j.first;
        double& known_dist = j.second;
        double dist_through_k =
            dist_matrix.at(v1).at(k) + dist_matrix.at(k).at(v2);

        if (dist_through_k < known_dist) {
          known_dist = dist_through_k;
        }
      }
    }
  }

  return dist_matrix;
}

}  // namespace graphlib
