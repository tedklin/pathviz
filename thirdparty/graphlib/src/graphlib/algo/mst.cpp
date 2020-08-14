#include "graphlib/algo/mst.hpp"

#include <iostream>
#include <queue>

namespace graphlib {

// Min-heap keeps track of the shortest edge between a non-tree vertex and the
// growing spanning tree.
std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>>
    g_crossing_edges;

// Debugging utility to check contents of an Edge priority queue.
std::string to_string(
    std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> pq) {
  std::string s("Edge priority queue contents:\n");
  while (!pq.empty()) {
    s += to_string(pq.top());
    pq.pop();
  }
  s += '\n';
  return s;
}

void prim_visit(Graph* graph, const Vertex* v) {
  v->state_ = Vertex::State::DISCOVERED;
  for (auto& adj : graph->GetAdjacentSet(v)) {
    const Vertex* v2 = adj.first;
    double weight = adj.second;
    if (v2->state_ == Vertex::State::UNDISCOVERED) {
      g_crossing_edges.emplace(v, v2, weight);
    }
  }
}

// "Lazy" implementation of Prim's algorithm as seen in Sedgewick.
std::vector<Edge> prim_mst(Graph* graph) {
  if (graph->IsDirected()) {
    std::cerr << "Error: Tried to run Prim's algorithm on directed graph!\n";
    return std::vector<Edge>();
  }

  std::vector<Edge> mst;
  while (!g_crossing_edges.empty()) {
    g_crossing_edges.pop();
  }

  // Assuming the given graph is connected, this successfully adds at least one
  // crossing edge to the priority queue to kick off the algorithm.
  prim_visit(graph, graph->GetAdjacencyMap().cbegin()->first);

  while (!g_crossing_edges.empty()) {
    Edge e = g_crossing_edges.top();
    g_crossing_edges.pop();

    if (e.v1_->state_ == Vertex::State::DISCOVERED &&
        e.v2_->state_ == Vertex::State::DISCOVERED) {
      continue;
    }
    mst.push_back(e);
    if (e.v1_->state_ == Vertex::State::UNDISCOVERED) {
      prim_visit(graph, e.v1_);
    }
    if (e.v2_->state_ == Vertex::State::UNDISCOVERED) {
      prim_visit(graph, e.v2_);
    }
  }
  return mst;
}

// This is a Graph adaptation of the weighted Union-Find data structure to help
// with Kruskal's algorithm. The universal set is the set of all Vertices in a
// given Graph, and the disjoint subsets maintained by the weighted Union-Find
// represent connected components.
//
// Each disjoint subset can be represented as a tree, with the tree root
// "naming" the subset. The underlying implementation here is composed of two
// maps, one of which binds each Vertex to its parent in its tree. Vertices
// bound to a nullptr represent tree roots. The other map keeps track of each
// tree's size, which is used to keep trees balanced when they are merged.
class VertexUnionFind {
 public:
  VertexUnionFind(Graph* graph) {
    // Initially, each vertex is its own subset / connected component.
    for (const auto& v : graph->GetAdjacencyMap()) {
      parents_[v.first] = nullptr;
      sizes_[v.first] = 1;
    }
  }

  // Returns the "name" of the subset / connected component containing a given
  // Vertex (i.e. the root of the given Vertex's tree.)
  const Vertex* Find(const Vertex* v) const {
    while (parents_.at(v)) {
      v = parents_.at(v);
    }
    return v;
  }

  bool IsConnected(const Vertex* v1, const Vertex* v2) const {
    return Find(v1) == Find(v2);
  }

  // Merge the subsets containing the given Vertices.
  void Union(const Vertex* v1, const Vertex* v2) {
    if (IsConnected(v1, v2)) {
      return;
    }

    // Merge the smaller tree into the larger tree to maintain balance.
    const Vertex *set1 = Find(v1), *set2 = Find(v2);
    if (sizes_.at(set1) < sizes_.at(set2)) {
      parents_.at(set1) = set2;
      sizes_.at(set2) += sizes_.at(set1);
    } else {
      parents_.at(set2) = set1;
      sizes_.at(set1) += sizes_.at(set2);
    }
  }

 private:
  std::map<const Vertex*, const Vertex*> parents_;
  std::map<const Vertex*, int> sizes_;
};

std::vector<Edge> kruskal_mst(Graph* graph) {
  if (graph->IsDirected()) {
    std::cerr << "Error: Tried to run Kruskal's algorithm on directed graph!\n";
    return std::vector<Edge>();
  }

  std::vector<Edge> mst;
  while (!g_crossing_edges.empty()) {
    g_crossing_edges.pop();
  }

  // Add all edges in given graph to priority queue.
  for (const auto& p : graph->GetAdjacencyMap()) {
    for (const auto& adj : p.second) {
      g_crossing_edges.emplace(p.first, adj.first, adj.second);
    }
  }

  VertexUnionFind uf(graph);
  while (!g_crossing_edges.empty() &&
         mst.size() < graph->GetAdjacencyMap().size()) {
    Edge e = g_crossing_edges.top();
    g_crossing_edges.pop();

    if (!uf.IsConnected(e.v1_, e.v2_)) {
      uf.Union(e.v1_, e.v2_);
      mst.push_back(e);
    }
  }
  return mst;
}

}  // namespace graphlib
