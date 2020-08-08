#include "graphlib/algo/bfs.hpp"

#include <iostream>
#include <queue>

namespace graphlib {

void bfs(Graph* graph, std::shared_ptr<const Vertex> search_root,
         void (*process_vertex_early)(std::shared_ptr<const Vertex> v),
         void (*process_edge)(std::shared_ptr<const Vertex> v1,
                              std::shared_ptr<const Vertex> v2, double weight),
         void (*process_vertex_late)(std::shared_ptr<const Vertex> v)) {
  std::queue<std::shared_ptr<const Vertex>> q;
  q.push(search_root);

  while (!q.empty()) {
    std::shared_ptr<const Vertex> v1 = q.front();
    q.pop();

    v1->state_ = Vertex::State::DISCOVERED;
    if (process_vertex_early) {
      process_vertex_early(v1);
    }

    for (auto& adj : graph->GetAdjacentSet(*v1)) {
      std::shared_ptr<const Vertex> v2 = adj.first;
      double weight = adj.second;
      if (process_edge) {
        process_edge(v1, v2, weight);
      }
      if (v2->state_ == Vertex::State::UNDISCOVERED) {
        v2->state_ = Vertex::State::DISCOVERED;
        v2->parent_ = v1;
        q.push(v2);
      }
    }

    if (process_vertex_late) {
      process_vertex_late(v1);
    }
    v1->state_ = Vertex::State::PROCESSED;
  }
}

std::stack<std::shared_ptr<const Vertex>> shortest_unweighted_path(
    Graph* graph, std::shared_ptr<const Vertex> search_root,
    std::shared_ptr<const Vertex> destination) {
  bfs(graph, search_root);
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

std::set<std::shared_ptr<const Vertex>> g_component;

std::vector<std::set<std::shared_ptr<const Vertex>>> connected_components(
    Graph* graph) {
  g_component.clear();

  std::vector<std::set<std::shared_ptr<const Vertex>>> components;
  for (auto& p : graph->GetVertexMap()) {
    if (p.first->state_ == Vertex::State::UNDISCOVERED) {
      bfs(graph, p.first,
          [](std::shared_ptr<const Vertex> v) { g_component.insert(v); });
      components.push_back(g_component);
      g_component.clear();
    }
  }
  return components;
}

bool g_is_bipartite = true;
std::map<std::shared_ptr<const Vertex>, int>
    g_color;  // "color" represented by 1 or -1

bool is_bipartite(Graph* graph) {
  g_is_bipartite = true;
  g_color.clear();
  for (const auto& p : graph->GetVertexMap()) {
    // We need to use a non-1 initial value to differentiate between tree edges
    // and nondiscovery edges.
    g_color[p.first] = 0;
  }

  for (auto& p : graph->GetVertexMap()) {
    if (p.first->state_ == Vertex::State::UNDISCOVERED) {
      g_color.at(p.first) = 1;
      bfs(graph, p.first, nullptr,
          [](std::shared_ptr<const Vertex> v1, std::shared_ptr<const Vertex> v2,
             double weight) {
            // Check for any nondiscovery edges that violate two-coloring.
            if (g_color.at(v1) == g_color.at(v2)) {
              std::cout << v1->name_ << " (color=" << g_color.at(v1) << ") and "
                        << v2->name_ << " (color=" << g_color.at(v2)
                        << ") violate bipartiteness\n";
              g_is_bipartite = false;
            }

            // Color any newly discovered Vertex to be the complement of its
            // parent.
            g_color.at(v2) = -(g_color.at(v1));
          });
    }
  }
  return g_is_bipartite;
}

void print_vertex(std::shared_ptr<const Vertex> v) {
  std::cout << "processing vertex: " << v->name_ << "\n";
}

void print_edge(std::shared_ptr<const Vertex> v1,
                std::shared_ptr<const Vertex> v2, double weight) {
  std::cout << "processing edge: " << v1->name_ << " -> " << v2->name_ << "\n";
}

}  // namespace graphlib
