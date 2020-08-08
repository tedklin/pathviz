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

The "Graph" class defines two typenames, "VertexMap" and "AdjacentSet", for
internal use as the underlying data structure.

The "VertexMap" typename (map<shared_ptr<Vertex>, AdjacentSet>), represents the
set of all Vertex instances contained in a Graph. It maps each Vertex instance
of a graph to a corresponding "AdjacentSet" type.

The "AdjacentSet" typename (map<shared_ptr<const Vertex>, double>) represents
the set of neighboring vertices to an arbitrary "source" vertex (the "source"
vertex is defined by the VertexMap key to which an AdjacentSet is bound). The
AdjacentSet type maps each neighboring vertex with a floating point "edge
weight". Altogether, a VertexMap key, AdjacentSet key, and floating point edge
weight represent the concept of one edge in a graph.

Note that the keyset of the Graph class member of type VertexMap is the only
copy of Vertex instances stored in a Graph. When processing a Graph, one should
generally use pointers to the keys of the VertexMap (see Graph::GetVertexPtr
member).

There exists an auxiliary "Edge" struct, which also represents the concept of an
edge in a graph, but this is only used for specific algorithms (like finding
MSTs) and not for defining Graphs themselves.

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

See the "example_initializer" function in examples/core_test.cpp for
verification that this works.
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

  // Since the underlying implementation of Graph relies on pointers to const
  // Vertex, any Vertex data member we want to be able to modify through the
  // Graph needs to be of mutable type. This gives us the flexibility to add
  // nonconst data members to Vertex types and still maintain const-correctness
  // when performing graph algorithms.
  mutable State state_ = State::UNDISCOVERED;     // search state
  mutable std::shared_ptr<const Vertex> parent_;  // search tree parent
  mutable int entry_time_ = 0, exit_time_ = 0;    // dfs time intervals

  void Reset() const {
    state_ = State::UNDISCOVERED;
    parent_.reset();
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
  bool operator()(const std::shared_ptr<Vertex>& lhs,
                  const std::shared_ptr<Vertex>& rhs) const {
    return *lhs < *rhs;
  }
  bool operator()(const std::shared_ptr<const Vertex>& lhs,
                  const std::shared_ptr<const Vertex>& rhs) const {
    return *lhs < *rhs;
  }
};

class Graph {
 protected:
  // Underlying data structure types. Well-tuned unordered maps should also work
  // here if we need a performance boost.
  using AdjacentSet =
      std::map<std::shared_ptr<const Vertex>, double, UnderlyingVertexOrder>;
  using VertexMap =
      std::map<std::shared_ptr<Vertex>, AdjacentSet, UnderlyingVertexOrder>;

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
  Graph(InputWeightedAL adjacency_list, bool is_directed);

  // Given a Vertex, obtain a pointer to the singular instance of that Vertex
  // within this Graph object (i.e. in the keyset of vertex_map_).
  std::shared_ptr<const Vertex> GetVertexPtr(const Vertex& v) const;

  // Add a freshly-reset copy of the given Vertex to this graph. Duplicates are
  // ignored.
  void AddVertex(const Vertex& v);

  // Add an internal pointer to "dest" to the adjacency set of "source", along
  // with an associated edge weight. If the given vertices were not already
  // present in this Graph, they are added.
  void AddEdge(const Vertex& source, const Vertex& dest,
               double edge_weight = 1);

  bool EdgeExists(const Vertex& source, const Vertex& dest) const;

  double EdgeWeight(const Vertex& source, const Vertex& dest) const;

  // Reset the state, color, etc, of all Vertices in this Graph object.
  // This is intended for direct user usage and is *not* automatically called at
  // the start of algorithmic functions present in this library.
  void ResetState();

  // Return a (pointer to a) new copy of this Graph, but with all edges
  // reversed. Note that this naturally only makes sense for directed graphs.
  std::shared_ptr<Graph> GetReverseGraph() const;

  // Return a string displaying all vertices (without adjacency sets).
  std::string GetVertexSetStr() const;

  AdjacentSet& GetMutableAdjacentSet(const std::shared_ptr<Vertex>& source);
  AdjacentSet& GetMutableAdjacentSet(const Vertex& source);

  const AdjacentSet& GetAdjacentSet(
      const std::shared_ptr<Vertex>& source) const;
  const AdjacentSet& GetAdjacentSet(const Vertex& source) const;

  const VertexMap& GetVertexMap() const { return vertex_map_; }

  bool IsDirected() { return is_directed_; }

 protected:
  // The keyset of vertex_map_ represents the only copy of Vertices this Graph
  // stores.
  VertexMap vertex_map_;

  bool is_directed_;

  // Necessary internal helper for interfacing directly with the map.
  std::shared_ptr<Vertex> GetMutableVertexPtr(const Vertex& v) const;

  // Extensions of std::find
  VertexMap::const_iterator FindInVertexMap(const Vertex& v) const;
  VertexMap::const_iterator FindInAdjacentSet(const Vertex& v,
                                              const AdjacentSet& set) const;
};

// For a given graph, return a string displaying all vertices and corresponding
// adjacency sets.
std::string to_string(const Graph& graph);

// An auxiliary Edge type.
struct Edge {
  Edge(std::shared_ptr<const Vertex> v1, std::shared_ptr<const Vertex> v2,
       double weight)
      : v1_(v1), v2_(v2), weight_(weight) {}

  std::shared_ptr<const Vertex> v1_, v2_;
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
