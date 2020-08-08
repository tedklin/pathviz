#include "graphlib/graph.hpp"

#include <algorithm>
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

Graph::VertexMap::const_iterator Graph::FindInVertexMap(const Vertex& v) const {
  return std::find_if(
      vertex_map_.cbegin(), vertex_map_.cend(),
      [&](const VertexMap::value_type& p) -> bool { return *(p.first) == v; });
}

void Graph::AddVertex(const Vertex& v) {
  v.Reset();
  if (FindInVertexMap(v) == vertex_map_.end()) {
    vertex_map_[std::make_shared<Vertex>(v)];
  }
}

std::shared_ptr<Vertex> Graph::GetMutableVertexPtr(const Vertex& v) const {
  auto vertex_iter = FindInVertexMap(v);
  if (vertex_iter == vertex_map_.end()) {
    throw std::runtime_error(
        "Graph::GetVertexPtr error! Tried to obtain pointer to "
        "nonexistent vertex (" +
        v.name_ + ")\n");
  }
  return vertex_iter->first;
}

std::shared_ptr<const Vertex> Graph::GetVertexPtr(const Vertex& v) const {
  return GetMutableVertexPtr(v);  // adds a low-level const
}

void Graph::AddEdge(const Vertex& source, const Vertex& dest,
                    double edge_weight) {
  AddVertex(source);
  AddVertex(dest);

  // There should only be one instance of each Vertex in a Graph, so we store
  // adjacent vertices as pointers to the main set of Vertices (i.e. the keyset
  // of vertex_map_).
  vertex_map_[GetMutableVertexPtr(source)][GetVertexPtr(dest)] = edge_weight;
  if (!is_directed_) {
    vertex_map_[GetMutableVertexPtr(dest)][GetVertexPtr(source)] = edge_weight;
  }
}

bool Graph::EdgeExists(const Vertex& source, const Vertex& dest) const {
  auto source_iter = FindInVertexMap(source);
  auto dest_iter = FindInVertexMap(dest);

  if (source_iter == vertex_map_.end() || dest_iter == vertex_map_.end()) {
    throw std::runtime_error(
        "Graph::EdgeExists error! Given nonexistent vertices.\n");
  }

  auto adj_iter =
      std::find_if(source_iter->second.cbegin(), source_iter->second.cend(),
                   [&](const AdjacentSet::value_type& p) -> bool {
                     return *(p.first) == dest;
                   });
  return adj_iter != source_iter->second.end();
}

double Graph::EdgeWeight(const Vertex& source, const Vertex& dest) const {
  if (!EdgeExists(source, dest)) {
    throw std::runtime_error(
        "Graph::EdgeWeight error! Given nonexistent edge.\n");
  }
  return this->GetAdjacentSet(source).at(this->GetVertexPtr(dest));
}

void Graph::ResetState() {
  for (auto& v : vertex_map_) {
    v.first->Reset();
  }
}

std::string Graph::GetVertexSetStr() const {
  std::string s("Vertex set:\n");
  for (const auto& v : vertex_map_) {
    s += v.first->name_ + "(state=" + graphlib::to_string(v.first->state_) +
         ") | ";
  }
  s += '\n';
  return s;
}

std::shared_ptr<Graph> Graph::GetReverseGraph() const {
  std::shared_ptr<Graph> reverse = std::make_shared<Graph>(true);
  for (const auto& v : this->GetVertexMap()) {
    Vertex source = *(v.first);
    reverse->AddVertex(source);
    for (const auto& adj : v.second) {
      reverse->AddEdge(*(adj.first), source, adj.second);
    }
  }
  return reverse;
}

Graph::AdjacentSet& Graph::GetMutableAdjacentSet(
    const std::shared_ptr<Vertex>& source) {
  return vertex_map_.at(source);
}

Graph::AdjacentSet& Graph::GetMutableAdjacentSet(const Vertex& source) {
  return GetMutableAdjacentSet(GetMutableVertexPtr(source));
}

const Graph::AdjacentSet& Graph::GetAdjacentSet(
    const std::shared_ptr<Vertex>& source) const {
  return vertex_map_.at(source);
}

const Graph::AdjacentSet& Graph::GetAdjacentSet(const Vertex& source) const {
  return GetAdjacentSet(GetMutableVertexPtr(source));
}

std::string to_string(const Graph& graph) {
  std::string s("Adjacency lists:\n");
  for (const auto& v : graph.GetVertexMap()) {
    s += v.first->name_ + " -> ";
    for (const auto& adj : v.second) {
      s += adj.first->name_ + "(wgt=" + std::to_string(adj.second) + ") | ";
    }
    s += '\n';
  }
  s += '\n';
  return s;
}

}  // namespace graphlib
