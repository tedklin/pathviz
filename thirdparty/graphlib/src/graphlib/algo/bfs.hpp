#pragma once

#include "graphlib/graph.hpp"

#include <stack>
#include <vector>

namespace graphlib {

// Traditional BFS algorithm. Results in a complete BFS search tree encoded in
// the parent members of each Vertex.
void bfs(Graph* graph, const Vertex* search_root,
         void (*process_vertex_early)(const Vertex* v) = nullptr,
         void (*process_edge)(const Vertex* v1, const Vertex* v2,
                              double weight) = nullptr,
         void (*process_vertex_late)(const Vertex* v) = nullptr);

// Repeatedly pop the stack returned by this function to obtain shortest
// unweighted path.
std::stack<const Vertex*> shortest_unweighted_path(Graph* graph,
                                                   const Vertex* search_root,
                                                   const Vertex* destination);

// Connected components (not strong).
std::vector<std::vector<const Vertex*>> connected_components(Graph* graph);

// Check if a graph is two-colorable.
bool is_bipartite(Graph* graph);

// Misc common vertex and edge processing functions
void print_vertex(const Vertex* v);
void print_edge(const Vertex* v1, const Vertex* v2, double weight);

}  // namespace graphlib
