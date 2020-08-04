// The current DFS implementation (see dfs.cpp) is recursive with several global
// variables storing the information of interest. That is why several of the
// functions here return by reference.

#pragma once

#include "graphlib/algo/bfs.hpp"
#include "graphlib/graph.hpp"

#include <set>
#include <stack>

namespace graphlib {

// Traditional DFS algorithm. Results in a complete DFS search tree encoded in
// the parent members of each Vertex.
void dfs(Graph* graph, const Vertex* search_root,
         void (*process_vertex_early)(const Vertex* v) = nullptr,
         void (*process_edge)(const Vertex* v1, const Vertex* v2,
                              double weight) = nullptr,
         void (*process_vertex_late)(const Vertex* v) = nullptr);

// DFS that traverses entire Graph (possibly disconnected, so no specified
// search root).
void dfs_graph(Graph* graph,
               void (*process_vertex_early)(const Vertex* v) = nullptr,
               void (*process_edge)(const Vertex* v1, const Vertex* v2,
                                    double weight) = nullptr,
               void (*process_vertex_late)(const Vertex* v) = nullptr);

bool is_cyclic(Graph* graph);

// Repeatedly pop the stack returned by this function to obtain topological
// sort of a DAG. Useful for scheduling problems and optimizations of other
// algorithms on DAGs.
std::stack<const Vertex*>& topological_sort(Graph* graph);

// For directed graphs, a strongly connected component is one where every Vertex
// can reach every other Vertex and vice versa. Strongly connected components
// highlight significant groupings in a network of relationships.
std::vector<std::set<const Vertex*>> strong_components(Graph* graph);

// UNTESTED!
// Articulation vertices are vertices that, when cut, disconnect the graph.
// No articulation vertices == biconnected.
std::set<const Vertex*>& articulation_vertices(Graph* graph);
bool is_biconnected(Graph* graph);

// Forward declarations for functions common to BFS.
void print_vertex(const Vertex* v);
void print_edge(const Vertex* v1, const Vertex* v2, double weight);

}  // namespace graphlib
