#include "graphlib/algo/dfs.hpp"

#include <iostream>
#include <memory>
#include <stack>

namespace graphlib {

// Time intervals can give us valuable information about the structure of the
// DFS search tree (see Skiena).
int g_time = 0;

// Allow for early search termination.
bool g_finished = false;

void dfs_helper(Graph* graph, const Vertex* v1,
                void (*process_vertex_early)(const Vertex* v),
                void (*process_edge)(const Vertex* v1, const Vertex* v2,
                                     double weight),
                void (*process_vertex_late)(const Vertex* v)) {
  if (g_finished) return;

  v1->state_ = Vertex::State::DISCOVERED;
  v1->entry_time_ = ++g_time;
  if (process_vertex_early) {
    process_vertex_early(v1);
  }

  for (auto& adj : graph->GetAdjacentSet(v1)) {
    const Vertex* v2 = adj.first;
    double weight = adj.second;

    if (v2->state_ == Vertex::State::UNDISCOVERED) {
      // Tree edge.
      v2->parent_ = v1;
      if (process_edge) {
        process_edge(v1, v2, weight);
      }
      dfs_helper(graph, v2, process_vertex_early, process_edge,
                 process_vertex_late);
    } else if ((v2->state_ == Vertex::State::DISCOVERED && v1->parent_ != v2) ||
               graph->IsDirected()) {
      // If undirected, we ignore this edge if it's merely the reverse of an
      // already processed edge. Reverse edges also be thought of as "cycles" of
      // only two vertices and hold no meaning for undirected graphs. If this is
      // not a reverse edge, this *must* be a (newly discovered) back edge.

      // If directed, this can be a back, forward, or cross edge.

      if (process_edge) {
        process_edge(v1, v2, weight);
      }
    }

    if (g_finished) return;
  }

  if (process_vertex_late) {
    process_vertex_late(v1);
  }
  v1->exit_time_ = ++g_time;
  v1->state_ = Vertex::State::PROCESSED;
}

void dfs(Graph* graph, const Vertex* search_root,
         void (*process_vertex_early)(const Vertex* v),
         void (*process_edge)(const Vertex* v1, const Vertex* v2,
                              double weight),
         void (*process_vertex_late)(const Vertex* v)) {
  g_time = 0;
  g_finished = false;

  dfs_helper(graph, search_root, process_vertex_early, process_edge,
             process_vertex_late);
}

void dfs_graph(Graph* graph, void (*process_vertex_early)(const Vertex* v),
               void (*process_edge)(const Vertex* v1, const Vertex* v2,
                                    double weight),
               void (*process_vertex_late)(const Vertex* v)) {
  g_time = 0;
  g_finished = false;

  for (auto& p : graph->GetAdjacencyMap()) {
    if (p.first->state_ == Vertex::State::UNDISCOVERED) {
      dfs_helper(graph, p.first, process_vertex_early, process_edge,
                 process_vertex_late);
    }
  }
}

// The four basic edge types as seen in Skiena.
enum class EdgeType { TREE, BACK, FORWARD, CROSS, UNCLASSIFIED };

EdgeType classify_edge(const Vertex* v1, const Vertex* v2) {
  // Note that undirected graphs can only have tree or back edges.
  if (v2->parent_ == v1) return EdgeType::TREE;
  if (v2->state_ == Vertex::State::DISCOVERED) return EdgeType::BACK;

  // Additional possible edge types for directed graphs.
  if (v2->state_ == Vertex::State::PROCESSED &&
      v2->entry_time_ > v1->entry_time_) {
    return EdgeType::FORWARD;
  }
  if (v2->state_ == Vertex::State::PROCESSED &&
      v2->entry_time_ < v1->entry_time_) {
    return EdgeType::CROSS;
  }

  std::cerr << "Warning: unclassified edge " << v1->name_ << " -> " << v2->name_
            << "!\n\n";
  return EdgeType::UNCLASSIFIED;
}

bool g_cyclic = false;

bool is_cyclic(Graph* graph) {
  g_cyclic = false;

  for (auto& p : graph->GetAdjacencyMap()) {
    if (p.first->state_ == Vertex::State::UNDISCOVERED) {
      dfs(graph, p.first, nullptr,
          [](const Vertex* v1, const Vertex* v2, double weight) {
            if (v2->state_ == Vertex::State::DISCOVERED && v1->parent_ != v2) {
              // Found back edge, forming a cycle.
              g_cyclic = true;
              g_finished = true;
            }
          });
    }
    if (g_cyclic) break;
  }

  return g_cyclic;
}

std::stack<const Vertex*> g_topo_stack;

std::stack<const Vertex*>& topological_sort(Graph* graph) {
  if (!graph->IsDirected()) {
    std::cerr << "Warning: tried to perform topological sort on a non-DAG!\n\n";
  }

  while (!g_topo_stack.empty()) {
    g_topo_stack.pop();
  }

  dfs_graph(graph, nullptr,
            [](const Vertex* v1, const Vertex* v2, double weight) {
              if (classify_edge(v1, v2) == EdgeType::BACK) {
                std::cerr << "Warning: tried to perform topological sort on a "
                             "non-DAG!\n\n";
              }
            },
            [](const Vertex* v) { g_topo_stack.push(v); });

  return g_topo_stack;
}

std::stack<Vertex> g_kosaraju_stack;
std::vector<const Vertex*> g_strong_component;

// Kosaraju's algorithm.
std::vector<std::vector<const Vertex*>> strong_components(Graph* graph) {
  std::vector<std::vector<const Vertex*>> components;

  std::unique_ptr<Graph> reverse = graph->GetReverseGraph();
  dfs_graph(reverse.get(), nullptr, nullptr,
            [](const Vertex* v) { g_kosaraju_stack.push(*v); });

  while (!g_kosaraju_stack.empty()) {
    const Vertex* v = graph->GetVertexPtr(g_kosaraju_stack.top());
    g_kosaraju_stack.pop();
    if (v->state_ == Vertex::State::UNDISCOVERED) {
      dfs_helper(graph, v,
                 [](const Vertex* v) { g_strong_component.push_back(v); },
                 nullptr, nullptr);
    }
    if (!g_strong_component.empty()) {
      components.push_back(g_strong_component);
    }
    g_strong_component.clear();
  }

  return components;
}

}  // namespace graphlib
