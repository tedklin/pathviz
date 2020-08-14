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
    default:
      return "nonexistent Vertex state";
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

Graph::VertexSet::const_iterator Graph::FindInVertexSet(const Vertex& v) const {
  return std::find_if(
      vertex_set_.cbegin(), vertex_set_.cend(),
      [&](const VertexSet::value_type& p) -> bool { return *p == v; });
}

Graph::AdjacencyMap::const_iterator Graph::FindInAdjacencyMap(
    const Vertex& v) const {
  return std::find_if(adjacency_map_.cbegin(), adjacency_map_.cend(),
                      [&](const AdjacencyMap::value_type& p) -> bool {
                        return *(p.first) == v;
                      });
}

const Vertex* Graph::GetVertexPtr(const Vertex& v) const {
  auto vertex_iter = FindInVertexSet(v);
  if (vertex_iter == vertex_set_.end()) {
    throw std::runtime_error(
        "Graph::GetVertexPtr error! Tried to obtain pointer to "
        "nonexistent vertex (" +
        v.name_ + ")\n");
  }
  return (*vertex_iter).get();
}

void Graph::AddVertex(const Vertex& v) {
  v.Reset();
  if (FindInVertexSet(v) == vertex_set_.end()) {
    vertex_set_.insert(std::make_unique<Vertex>(v));
  }
  if (FindInAdjacencyMap(v) == adjacency_map_.end()) {
    adjacency_map_[GetVertexPtr(v)];
  }
}

void Graph::AddEdge(const Vertex& source, const Vertex& dest,
                    double edge_weight) {
  AddVertex(source);
  AddVertex(dest);

  auto source_ptr = GetVertexPtr(source);
  auto dest_ptr = GetVertexPtr(dest);

  adjacency_map_.at(source_ptr).insert({dest_ptr, edge_weight});
  if (!is_directed_) {
    adjacency_map_.at(dest_ptr).insert({source_ptr, edge_weight});
  }
}

bool Graph::EdgeExists(const Vertex& source, const Vertex& dest) const {
  auto adj_iter = FindInAdjacencyMap(source);
  const auto& adj_set = adj_iter->second;
  for (const auto& pair : adj_set) {
    if (*(pair.first) == dest) {
      return true;
    }
  }
  return false;
}

double Graph::EdgeWeight(const Vertex& source, const Vertex& dest) const {
  auto adj_iter = FindInAdjacencyMap(source);
  const auto& adj_set = adj_iter->second;
  for (const auto& pair : adj_set) {
    if (*(pair.first) == dest) {
      return pair.second;
    }
  }
  throw std::runtime_error(
      "Graph::EdgeWeight error! Given nonexistent edge.\n");
}

void Graph::ResetState() {
  for (auto& v : vertex_set_) {
    v->Reset();
  }
}

std::string Graph::GetVertexSetStr() const {
  std::string s("Vertex set:\n");
  for (const auto& v : vertex_set_) {
    s += v->name_ + "(state=" + graphlib::to_string(v->state_) + ") | ";
  }
  s += '\n';
  return s;
}

std::unique_ptr<Graph> Graph::GetReverseGraph() const {
  std::unique_ptr<Graph> reverse = std::make_unique<Graph>(true);
  for (const auto& v : this->GetAdjacencyMap()) {
    Vertex source = *(v.first);
    reverse->AddVertex(source);
    for (const auto& adj : v.second) {
      reverse->AddEdge(*(adj.first), source, adj.second);
    }
  }
  return reverse;
}

Graph::AdjacentSet& Graph::GetMutableAdjacentSet(const Vertex* source) {
  return adjacency_map_.at(source);
}

const Graph::AdjacentSet& Graph::GetAdjacentSet(const Vertex* source) const {
  return adjacency_map_.at(source);
}

std::string to_string(const Graph& graph) {
  std::string s("Adjacency lists:\n");
  for (const auto& v : graph.GetAdjacencyMap()) {
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
