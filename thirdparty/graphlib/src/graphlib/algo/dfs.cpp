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

void dfs_helper(Graph* graph, std::shared_ptr<const Vertex> v1,
                void (*process_vertex_early)(std::shared_ptr<const Vertex> v),
                void (*process_edge)(std::shared_ptr<const Vertex> v1,
                                     std::shared_ptr<const Vertex> v2,
                                     double weight),
                void (*process_vertex_late)(std::shared_ptr<const Vertex> v)) {
  if (g_finished) return;

  v1->state_ = Vertex::State::DISCOVERED;
  v1->entry_time_ = ++g_time;
  if (process_vertex_early) {
    process_vertex_early(v1);
  }

  for (auto& adj : graph->GetAdjacentSet(*v1)) {
    std::shared_ptr<const Vertex> v2 = adj.first;
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

void dfs(Graph* graph, std::shared_ptr<const Vertex> search_root,
         void (*process_vertex_early)(std::shared_ptr<const Vertex> v),
         void (*process_edge)(std::shared_ptr<const Vertex> v1,
                              std::shared_ptr<const Vertex> v2, double weight),
         void (*process_vertex_late)(std::shared_ptr<const Vertex> v)) {
  g_time = 0;
  g_finished = false;

  dfs_helper(graph, search_root, process_vertex_early, process_edge,
             process_vertex_late);
}

void dfs_graph(Graph* graph,
               void (*process_vertex_early)(std::shared_ptr<const Vertex> v),
               void (*process_edge)(std::shared_ptr<const Vertex> v1,
                                    std::shared_ptr<const Vertex> v2,
                                    double weight),
               void (*process_vertex_late)(std::shared_ptr<const Vertex> v)) {
  g_time = 0;
  g_finished = false;

  for (auto& p : graph->GetVertexMap()) {
    if (p.first->state_ == Vertex::State::UNDISCOVERED) {
      dfs_helper(graph, p.first, process_vertex_early, process_edge,
                 process_vertex_late);
    }
  }
}

// The four basic edge types as seen in Skiena.
enum class EdgeType { TREE, BACK, FORWARD, CROSS, UNCLASSIFIED };

EdgeType classify_edge(std::shared_ptr<const Vertex> v1,
                       std::shared_ptr<const Vertex> v2) {
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

  for (auto& p : graph->GetVertexMap()) {
    if (p.first->state_ == Vertex::State::UNDISCOVERED) {
      dfs(graph, p.first, nullptr,
          [](std::shared_ptr<const Vertex> v1, std::shared_ptr<const Vertex> v2,
             double weight) {
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

std::stack<std::shared_ptr<const Vertex>> g_topo_stack;

std::stack<std::shared_ptr<const Vertex>>& topological_sort(Graph* graph) {
  if (!graph->IsDirected()) {
    std::cerr << "Warning: tried to perform topological sort on a non-DAG!\n\n";
  }

  while (!g_topo_stack.empty()) {
    g_topo_stack.pop();
  }

  dfs_graph(graph, nullptr,
            [](std::shared_ptr<const Vertex> v1,
               std::shared_ptr<const Vertex> v2, double weight) {
              if (classify_edge(v1, v2) == EdgeType::BACK) {
                std::cerr << "Warning: tried to perform topological sort on a "
                             "non-DAG!\n\n";
              }
            },
            [](std::shared_ptr<const Vertex> v) { g_topo_stack.push(v); });

  return g_topo_stack;
}

std::stack<Vertex> g_kosaraju_stack;
std::set<std::shared_ptr<const Vertex>> g_strong_component;

// Kosaraju's algorithm.
std::vector<std::set<std::shared_ptr<const Vertex>>> strong_components(
    Graph* graph) {
  std::vector<std::set<std::shared_ptr<const Vertex>>> components;

  std::shared_ptr<Graph> reverse = graph->GetReverseGraph();
  dfs_graph(reverse.get(), nullptr, nullptr,
            [](std::shared_ptr<const Vertex> v) { g_kosaraju_stack.push(*v); });

  while (!g_kosaraju_stack.empty()) {
    std::shared_ptr<const Vertex> v =
        graph->GetVertexPtr(g_kosaraju_stack.top());
    g_kosaraju_stack.pop();
    if (v->state_ == Vertex::State::UNDISCOVERED) {
      dfs_helper(
          graph, v,
          [](std::shared_ptr<const Vertex> v) { g_strong_component.insert(v); },
          nullptr, nullptr);
    }
    if (!g_strong_component.empty()) {
      components.push_back(g_strong_component);
    }
    g_strong_component.clear();
  }

  return components;
}

std::set<std::shared_ptr<const Vertex>> g_artic_vertices;
std::map<std::shared_ptr<const Vertex>, std::shared_ptr<const Vertex>>
    g_reachable_ancestors;
std::map<std::shared_ptr<const Vertex>, int> g_tree_out_degree;

// UNTESTED!
std::set<std::shared_ptr<const Vertex>>& articulation_vertices(Graph* graph) {
  g_artic_vertices.clear();
  g_reachable_ancestors.clear();
  g_tree_out_degree.clear();
  for (const auto& p : graph->GetVertexMap()) {
    g_reachable_ancestors[p.first] = p.first;
    g_tree_out_degree[p.first] = 0;
  }

  dfs_graph(
      graph, nullptr,
      [](std::shared_ptr<const Vertex> v1, std::shared_ptr<const Vertex> v2,
         double weight) {
        if (classify_edge(v1, v2) == EdgeType::TREE) {
          ++(g_tree_out_degree.at(v1));
        } else if (classify_edge(v1, v2) == EdgeType::BACK &&
                   v1->parent_ != v2) {
          if (v2->entry_time_ < g_reachable_ancestors.at(v1)->entry_time_) {
            g_reachable_ancestors.at(v1) = v2;
          }
        }
      },
      [](std::shared_ptr<const Vertex> v) {
        // Edge case for search tree root vertex
        if (!(v->parent_)) {
          if (g_tree_out_degree.at(v) > 1) {
            std::cout << "Root articulation vertex found: " << v->name_ << '\n';
            g_artic_vertices.insert(v);
          }
          return;
        }

        // If v's search tree parent is not the root vertex
        if (!(v->parent_->parent_)) {
          if (g_reachable_ancestors.at(v) == v->parent_) {
            std::cout << "Parent articulation vertex found: " << v->name_
                      << '\n';
            g_artic_vertices.insert(v);
          }
          if (g_reachable_ancestors.at(v) == v) {
            std::cout << "Bridge articulation vertex found: "
                      << v->parent_->name_ << '\n';
            g_artic_vertices.insert(v->parent_);

            // If v is not a leaf
            if (g_tree_out_degree.at(v) > 0) {
              std::cout << "Bridge articulation vertex found: " << v->name_
                        << '\n';
              g_artic_vertices.insert(v);
            }
          }
        }

        // A Vertex's earliest reachable ancestor is also its search tree
        // parent's earliest reachable ancestor.
        if (g_reachable_ancestors.at(v)->entry_time_ <
            g_reachable_ancestors.at(v->parent_)->entry_time_) {
          g_reachable_ancestors.at(v->parent_) = g_reachable_ancestors.at(v);
        }
      });

  return g_artic_vertices;
}

// UNTESTED!
bool is_biconnected(Graph* graph) {
  articulation_vertices(graph);
  return g_artic_vertices.empty();
}

}  // namespace graphlib
