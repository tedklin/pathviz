// 2d geometric extensions of core Vertex and Graph types.

#pragma once

#include <cmath>

#include "graphlib/graph.hpp"

namespace graphlib {

struct Vertex2d : public Vertex {
  Vertex2d(double x, double y)
      : Vertex("(" + std::to_string(x) + ", " + std::to_string(y) + ")"),
        x_(x),
        y_(y) {}

  const double x_, y_;
};

inline double distance_2d(const Vertex2d& v1, const Vertex2d& v2) {
  return std::sqrt(std::pow(v1.x_ - v2.x_, 2) + std::pow(v1.y_ - v2.y_, 2));
}

// The main difference between Graph2d and Graph is that Graph2d doesn't allow
// direct user specification of edge weights. All edge weights are implicitly
// calculated geometrically.
class Graph2d : public Graph {
 public:
  using Input2dAL = std::map<Vertex2d, std::set<Vertex2d>>;

  // Constructs an empty graph.
  Graph2d(bool is_directed);

  // Constructs a graph given only a set of vertices, no edges.
  Graph2d(const InputVertexSet& vertex_set, bool is_directed);

  // Constructs a fully specified graph.
  Graph2d(const Input2dAL& al, bool is_directed);

  void AddEdge(const Vertex2d& source, const Vertex2d& dest);

  double EdgeWeight(const Vertex2d& source, const Vertex2d& dest);

  // Delete inherited functions that allow direct user specification of edge
  // weights.
  void AddEdge(const Vertex& source, const Vertex& dest) = delete;
  void AddEdge(const Vertex& source, const Vertex& dest,
               double edge_weight) = delete;
};

}  // namespace graphlib
