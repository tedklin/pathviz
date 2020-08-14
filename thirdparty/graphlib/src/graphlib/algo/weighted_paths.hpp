#pragma once

#include "graphlib/graph.hpp"

#include <map>
#include <stack>

namespace graphlib {

// For algorithms below that take a destination as an argument:
// If not given a destination, results in a complete shortest-paths tree encoded
// in each Vertex's parent member. If given a destination, terminates execution
// once destination is processed.

// Dijkstra's algorithm for single-source shortest non-negative weighted paths.
void dijkstra(Graph* graph, const Vertex* search_root,
              const Vertex* destination = nullptr);

// UNTESTED!
// A faster method for computing single-source shortest paths for edge-weighted
// DAGs, using a topological sort.
void dag_paths(Graph* graph, const Vertex* search_root,
               const Vertex* destination = nullptr);

// Queue-based Bellman-Ford algorithm for single-source shortest weighted paths
// in a graph without negative cycles.
void bellman_ford(Graph* graph, const Vertex* search_root);

// Repeatedly pop the stacks returned by these functions to obtain the
// corresponding paths.
std::stack<const Vertex*> shortest_pos_weight_path(Graph* graph,
                                                   const Vertex* search_root,
                                                   const Vertex* destination);
std::stack<const Vertex*> shortest_weighted_path(Graph* graph,
                                                 const Vertex* search_root,
                                                 const Vertex* destination);

// To improve the readability of our Floyd-Warshall output matrix type, we
// enforce an ordering based on the underlying Vertices stored by the matrix
// rather than the pointer values themselves. (see graph.hpp)
using DistanceMatrix =
    std::map<const Vertex*,
             std::map<const Vertex*, double, UnderlyingVertexOrder>,
             UnderlyingVertexOrder>;

// Floyd-Warshall algorithm for all-pairs distance matrix. Doubles as
// representation for transitive closure.
DistanceMatrix floyd_warshall(Graph* graph);

}  // namespace graphlib
