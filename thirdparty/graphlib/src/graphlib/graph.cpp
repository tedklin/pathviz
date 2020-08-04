#include "graphlib/graph.hpp"

#include <stdexcept>
#include <string>

namespace graphlib {

std::string to_string(const Vertex::State& state) {
  switch (state) {
    case Vertex::State::UNDISCOVERED:
      return "0";
      break;
    case Vertex::State::DISCOVERED:
      return "1";
      break;
    case Vertex::State::PROCESSED:
      return "2";
      break;
  }
}

Graph::Graph(bool is_directed) : is_directed_(is_directed) {}

Graph::Graph(const InputVertexSet& vertex_set, bool is_directed)
    : is_directed_(is_directed) {
  for (const Vertex& v : vertex_set) {
    AddVertex(v);
  }
}

Graph::Graph(InputUnweightedAL unweighted_al, bool is_directed)
    : is_directed_(is_directed) {
  for (const auto& v : unweighted_al) {
    Vertex source = v.first;
    AddVertex(source);
    for (const Vertex& dest : v.second) {
      AddEdge(source, dest);
    }
  }
}

Graph::Graph(InputWeightedAL weighted_al, bool is_directed)
    : is_directed_(is_directed) {
  for (const auto& v : weighted_al) {
    Vertex source = v.first;
    AddVertex(source);
    for (const auto& adj : v.second) {
      AddEdge(source, adj.first, adj.second);
    }
  }
}

void Graph::AddVertex(const Vertex& v) {
  v.Reset();
  vertex_map_[v];
}

const Vertex* Graph::GetInternalVertexPtr(const Vertex& v) const {
  auto vertex_iter = vertex_map_.find(v);
  if (vertex_iter == vertex_map_.end()) {
    throw std::runtime_error(
        "Graph::GetInternalVertexPtr error! Tried to obtain pointer to "
        "nonexistent vertex (" +
        v.name_ + ")\n");
  }
  return &(vertex_iter->first);
}

void Graph::AddEdge(const Vertex& source, const Vertex& dest,
                    double edge_weight) {
  AddVertex(source);
  AddVertex(dest);

  // There should only be one instance of each Vertex in a Graph, so we store
  // adjacent vertices as pointers to the main set of Vertices (i.e. the keyset
  // of vertex_map_).
  vertex_map_[source][GetInternalVertexPtr(dest)] = edge_weight;
  if (!is_directed_) {
    vertex_map_[dest][GetInternalVertexPtr(source)] = edge_weight;
  }
}

bool Graph::EdgeExists(const Vertex& source, const Vertex& dest) const {
  auto source_iter = vertex_map_.find(source);
  auto dest_iter = vertex_map_.find(dest);
  if (source_iter == vertex_map_.end() || dest_iter == vertex_map_.end()) {
    throw std::runtime_error(
        "Graph::EdgeExists error! Given nonexistent vertices.\n");
  }

  // logarithmic time!
  return this->GetAdjacentSet(source).find(this->GetInternalVertexPtr(dest)) !=
         this->GetAdjacentSet(source).end();
}

double Graph::EdgeWeight(const Vertex& source, const Vertex& dest) const {
  if (!EdgeExists(source, dest)) {
    throw std::runtime_error(
        "Graph::EdgeWeight error! Given nonexistent edge.\n");
  }
  return this->GetAdjacentSet(source).at(this->GetInternalVertexPtr(dest));
}

void Graph::ResetState() {
  for (auto& v : vertex_map_) {
    v.first.Reset();
  }
}

std::string Graph::GetVertexSetStr() const {
  std::string s("Vertex set:\n");
  for (const auto& v : vertex_map_) {
    s += v.first.name_ + "(state=" + graphlib::to_string(v.first.state_) + ")" +
         " | ";
  }
  s += '\n';
  return s;
}

std::string to_string(const Graph& graph) {
  std::string s("Adjacency lists:\n");
  for (const auto& v : graph.GetVertexMap()) {
    s += v.first.name_ + " -> ";
    for (const auto& adj : v.second) {
      s += adj.first->name_ + "(wgt=" + std::to_string(adj.second) + ") | ";
    }
    s += '\n';
  }
  s += '\n';
  return s;
}

std::shared_ptr<Graph> Graph::GetReverseGraph() const {
  std::shared_ptr<Graph> reverse = std::make_shared<Graph>(true);
  for (const auto& v : this->GetVertexMap()) {
    Vertex source = v.first;
    reverse->AddVertex(source);
    for (const auto& adj : v.second) {
      reverse->AddEdge(*(adj.first), source, adj.second);
    }
  }
  return reverse;
}

}  // namespace graphlib
