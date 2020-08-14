#include "graphlib/geometry/graph_2d.hpp"

namespace graphlib {

Graph2d::Graph2d(bool is_directed) : Graph(is_directed) {}

Graph2d::Graph2d(const InputVertexSet& vertex_set, bool is_directed)
    : Graph(vertex_set, is_directed) {}

Graph2d::Graph2d(const Input2dAL& al, bool is_directed) : Graph(is_directed) {
  for (const auto& v : al) {
    Vertex2d source = v.first;
    AddVertex(source);
    for (const Vertex2d& dest : v.second) {
      AddEdge(source, dest);
    }
  }
}

void Graph2d::AddVertex(const Vertex2d& v) {
  v.Reset();
  if (FindInVertexSet(v) == vertex_set_.end()) {
    vertex_set_.insert(std::make_unique<Vertex2d>(v));
  }
  if (FindInAdjacencyMap(v) == adjacency_map_.end()) {
    adjacency_map_[GetVertexPtr(v)];
  }
}

void Graph2d::AddEdge(const Vertex2d& source, const Vertex2d& dest) {
  AddVertex(source);
  AddVertex(dest);

  auto source_ptr = GetVertexPtr(source);
  auto dest_ptr = GetVertexPtr(dest);

  double edge_weight = distance_2d(source, dest);
  adjacency_map_.at(source_ptr).insert({dest_ptr, edge_weight});
  if (!is_directed_) {
    adjacency_map_.at(dest_ptr).insert({source_ptr, edge_weight});
  }
}

double Graph2d::EdgeWeight(const Vertex2d& source, const Vertex2d& dest) {
  return distance_2d(source, dest);
}

}  // namespace graphlib
