/*
Let the following be an undirected, unweighted graph with vertices
{A, B, C, D, E}:

      A       B
      |\
      | \
      |  \
      D---E---C

The corresponding adjacency sets would be:

      A -> {D, E}
      B -> {}
      C -> {E}
      D -> {A, E}
      E -> {A, C, D}

===============================================================================

The "Vertex" struct represents a vertex with a string name. It encodes commonly
used values associated with graph algorithms, such as search state. Inheritance
can be leveraged to create Vertex types that store more information.

The "Graph" class defines three typenames, "VertexSet", "AdjacentSet", and
"AdjacencyMap", for internal use as the underlying data structure.

The "VertexSet" typename (set<unique_ptr<Vertex>>) represents the only set of
Vertex instances allocated and maintained by the Graph. All uses of a Vertex in
the Graph must use a pointer to the Vertices contained in the VertexSet data
member.

The "AdjacencyMap" typename (map<const Vertex*, AdjacentSet>), maps each
Vertex instance of a graph to a corresponding "AdjacentSet" type.

The "AdjacentSet" typename (set<pair<const Vertex*, double>>) represents
the set of neighboring vertices from an arbitrary "source" vertex (the "source"
vertex is defined by the AdjacencyMap key to which an AdjacentSet is bound).
Each neighboring vertex is paired with a floating point edge weight.

Altogether, an AdjacencyMap key and a pair contained by the AdjacentSet bound
to that key represent the concept of one directed weighted edge in a Graph.

There exists an auxiliary "Edge" struct, which also represents the concept of an
edge in a graph, but this is only used in specific algorithms (like finding
MSTs) and are NOT for defining Graphs themselves.

===============================================================================

To support intuitive usage, the typenames "InputUnweightedAL" and
"InputWeightedAL" are defined for users to pass into Graph constructors.

The adjacency list in the example above could be thought of as the following
literal (of type "InputUnweightedAL"):

  rep1 = {
      {A, {D, E}},
      {B, {}},
      {C, {E}},
      {D, {A, E}},
      {E, {A, C, D}}
  }

With explicit edge weights, the adjacency list could also be thought of as the
following literal (of type "InputWeightedAL"):

  rep2 = {
      {A, {{D, 1}, {E, 1}}},
      {B, {}},
      {C, {{E, 1}}},
      {D, {{A, 1}, {E, 1}}},
      {E, {{A, 1}, {C, 1}, {D, 1}}}
  }

Note that the example is an undirected graph (i.e. existence of edge {A, D}
implies edge {D, A}). The "Graph" class automatically adds reverse edges in
undirected graphs. Thus, the following literal of type "InputUnweightedAL" would
also result in the same Graph as the above.

  rep3 = {
      {A, {D, E}},
      {B, {}},
      {D, {E}},
      {E, {C}}
  }

However, if the Graph instance's is_directed member is true, then rep3 would
naturally result in a different Graph than rep1 and rep2.

See examples/core_test.cpp for actual examples of this behavior.
*/

#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>

namespace graphlib {

struct Vertex {
  enum class State { UNDISCOVERED, DISCOVERED, PROCESSED };

  Vertex(const std::string& name) : name_(name) {}

  const std::string name_;

  // The Vertex data members below store special states used by graph algorithm
  // implementations. Making these mutable gives us the flexibility to add other
  // nonconst Vertex data that should not be modified during the execution of
  // graph algorithms, but can otherwise be modified for external uses (I like
  // to think of those members as the "payload" stored by a Vertex).
  mutable State state_ = State::UNDISCOVERED;   // search state
  mutable const Vertex* parent_;                // search tree parent
  mutable int entry_time_ = 0, exit_time_ = 0;  // dfs time intervals

  void Reset() const {
    state_ = State::UNDISCOVERED;
    parent_ = nullptr;
    entry_time_ = 0;
    exit_time_ = 0;
  }

  virtual ~Vertex() = default;
};

inline bool operator<(const Vertex& lhs, const Vertex& rhs) {
  return (lhs.name_ < rhs.name_);
}

inline bool operator==(const Vertex& lhs, const Vertex& rhs) {
  return (lhs.name_ == rhs.name_);
}

inline bool operator!=(const Vertex& lhs, const Vertex& rhs) {
  return !operator==(lhs, rhs);
}

std::string to_string(const Vertex::State& state);

struct UnderlyingVertexOrder {
  bool operator()(const Vertex* lhs, const Vertex* rhs) const {
    return *lhs < *rhs;
  }
  bool operator()(const std::unique_ptr<Vertex>& lhs,
                  const std::unique_ptr<Vertex>& rhs) const {
    return *lhs < *rhs;
  }
  bool operator()(const std::pair<const Vertex*, double>& lhs,
                  const std::pair<const Vertex*, double>& rhs) const {
    return *(lhs.first) < *(rhs.first);
  }
};

class Graph {
 protected:
  // Underlying data structure types.
  using VertexSet = std::set<std::unique_ptr<Vertex>, UnderlyingVertexOrder>;
  using AdjacentSet =
      std::set<std::pair<const Vertex*, double>, UnderlyingVertexOrder>;
  using AdjacencyMap =
      std::map<const Vertex*, AdjacentSet, UnderlyingVertexOrder>;

 public:
  // Convenience typenames used for user input; not actual underlying types.
  using InputUnweightedAL = std::map<Vertex, std::set<Vertex>>;
  using InputWeightedAL = std::map<Vertex, std::map<Vertex, double>>;
  using InputVertexSet = std::set<Vertex>;

  // Constructs an empty graph.
  Graph(bool is_directed);

  // Constructs a graph given only a set of vertices, no edges.
  Graph(const InputVertexSet& vertex_set, bool is_directed);

  // Constructs a fully specified unweighted graph.
  Graph(InputUnweightedAL unweighted_al, bool is_directed);

  // Constructs a fully specified weighted graph.
  Graph(InputWeightedAL weighted_al, bool is_directed);

  // Given a Vertex v, obtain a raw pointer to the singular instance of the
  // Vertex identical to v managed by this Graph object.
  const Vertex* GetVertexPtr(const Vertex& v) const;

  // Add a freshly-reset copy of the given Vertex to this graph. Duplicates are
  // ignored.
  void AddVertex(const Vertex& v);

  // Add "dest" to the adjacency set of "source", along with an associated edge
  // weight. If the given vertices were not already present in this Graph, they
  // are added.
  void AddEdge(const Vertex& source, const Vertex& dest,
               double edge_weight = 1);

  bool EdgeExists(const Vertex& source, const Vertex& dest) const;

  double EdgeWeight(const Vertex& source, const Vertex& dest) const;

  // Reset the state, color, etc, of all Vertices in this Graph object.
  // This is intended for direct user usage and is *not* automatically called at
  // the start of algorithmic functions present in this library.
  void ResetState();

  // Factory for a reversed copy of this Graph. Note that this naturally only
  // makes sense for directed graphs.
  std::unique_ptr<Graph> GetReverseGraph() const;

  // Return a string displaying all vertices (without adjacency sets).
  std::string GetVertexSetStr() const;

  AdjacentSet& GetMutableAdjacentSet(const Vertex* source);
  const AdjacentSet& GetAdjacentSet(const Vertex* source) const;

  const AdjacencyMap& GetAdjacencyMap() const { return adjacency_map_; }

  bool IsDirected() { return is_directed_; }

 protected:
  VertexSet vertex_set_;
  AdjacencyMap adjacency_map_;

  bool is_directed_;

  // Extensions of std::find
  VertexSet::const_iterator FindInVertexSet(const Vertex& v) const;
  AdjacencyMap::const_iterator FindInAdjacencyMap(const Vertex& v) const;
};

// For a given graph, return a string displaying all vertices and corresponding
// adjacency sets.
std::string to_string(const Graph& graph);

// An auxiliary Edge type.
struct Edge {
  Edge(const Vertex* v1, const Vertex* v2, double weight)
      : v1_(v1), v2_(v2), weight_(weight) {}

  const Vertex *v1_, *v2_;
  double weight_ = 0;
};

// Provides ordering for Edge min-heaps.
inline bool operator>(const Edge& lhs, const Edge& rhs) {
  return (lhs.weight_ > rhs.weight_);
}

inline std::string to_string(const Edge& edge) {
  return edge.v1_->name_ + " -> " + edge.v2_->name_ + " (" +
         std::to_string(edge.weight_) + ")\n";
}

}  // namespace graphlib
