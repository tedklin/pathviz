// A minimum spanning tree can be thought of as the subgraph with the minimum
// amount of "pipe" needed to connect the set of Vertices of a given graph.
//
// All input graphs are assumed to be undirected.

#pragma once

#include "graphlib/graph.hpp"

#include <vector>

namespace graphlib {

// Prim's algorithm. Assumes given graph is connected.
std::vector<Edge> prim_mst(Graph* graph);

// Kruskal's algorithm.
std::vector<Edge> kruskal_mst(Graph* graph);

}  // namespace graphlib
