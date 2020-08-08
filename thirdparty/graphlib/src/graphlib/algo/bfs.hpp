#pragma once

#include "graphlib/graph.hpp"

#include <set>
#include <stack>
#include <vector>

namespace graphlib {

// Traditional BFS algorithm. Results in a complete BFS search tree encoded in
// the parent members of each Vertex.
void bfs(
    Graph* graph, std::shared_ptr<const Vertex> search_root,
    void (*process_vertex_early)(std::shared_ptr<const Vertex> v) = nullptr,
    void (*process_edge)(std::shared_ptr<const Vertex> v1,
                         std::shared_ptr<const Vertex> v2,
                         double weight) = nullptr,
    void (*process_vertex_late)(std::shared_ptr<const Vertex> v) = nullptr);

// Repeatedly pop the stack returned by this function to obtain shortest
// unweighted path.
std::stack<std::shared_ptr<const Vertex>> shortest_unweighted_path(
    Graph* graph, std::shared_ptr<const Vertex> search_root,
    std::shared_ptr<const Vertex> destination);

// Connected components (not strong).
std::vector<std::set<std::shared_ptr<const Vertex>>> connected_components(
    Graph* graph);

// Check if a graph is two-colorable.
bool is_bipartite(Graph* graph);

// Misc common vertex and edge processing functions
void print_vertex(std::shared_ptr<const Vertex> v);
void print_edge(std::shared_ptr<const Vertex> v1,
                std::shared_ptr<const Vertex> v2, double weight);

}  // namespace graphlib
