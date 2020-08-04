// Quick ad-hoc tests for weighted shortest path graph algorithms.

#include "graphlib/algo/weighted_paths.hpp"
#include "graphlib/geometry/graph_2d.hpp"
#include "graphlib/graph.hpp"

#include <random>

#include "ros/ros.h"

using graphlib::Graph;
using graphlib::Graph2d;
using graphlib::Vertex;
using graphlib::Vertex2d;

void tiny_ewd_dijkstras() {
  // "tiny_ewd" graph example provided in Sedgewick.
  // See expected results in Sedgewick (p.653)
  Vertex v0("0"), v1("1"), v2("2"), v3("3"), v4("4"), v5("5"), v6("6"), v7("7");
  Graph::InputWeightedAL al = {{v0, {{v4, 0.38}, {v2, 0.26}}},
                               {v1, {{v3, 0.29}}},
                               {v2, {{v7, 0.34}}},
                               {v3, {{v6, 0.52}}},
                               {v4, {{v5, 0.35}, {v7, 0.37}}},
                               {v5, {{v4, 0.35}, {v7, 0.28}, {v1, 0.32}}},
                               {v6, {{v2, 0.4}, {v0, 0.58}, {v4, 0.93}}},
                               {v7, {{v5, 0.28}, {v3, 0.39}}}};
  Graph tiny_ewd(al, true);

  std::cout << "Shortest paths parent tree:\n";
  std::cout << "search root: " << v0.name_ << '\n';
  graphlib::dijkstra(&tiny_ewd, tiny_ewd.GetInternalVertexPtr(v0));
  for (const auto& v : tiny_ewd.GetVertexMap()) {
    if (v.first != v0) {
      std::cout << v.first.name_ << " parent: " << v.first.parent_->name_
                << '\n';
    }
  }
  std::cout << '\n';

  std::cout << "Shortest weighted path from 0 to 1:\n";
  std::stack<const Vertex*> path = graphlib::shortest_pos_weight_path(
      &tiny_ewd, tiny_ewd.GetInternalVertexPtr(v0),
      tiny_ewd.GetInternalVertexPtr(v1));
  if (!path.empty()) {
    while (path.size() > 1) {
      std::cout << path.top()->name_ << " -> ";
      path.pop();
    }
    std::cout << path.top()->name_ << '\n';
  }
  std::cout << '\n';

  std::cout << "Shortest weighted path from 0 to 3:\n";
  path = graphlib::shortest_pos_weight_path(&tiny_ewd,
                                            tiny_ewd.GetInternalVertexPtr(v0),
                                            tiny_ewd.GetInternalVertexPtr(v3));
  if (!path.empty()) {
    while (path.size() > 1) {
      std::cout << path.top()->name_ << " -> ";
      path.pop();
    }
    std::cout << path.top()->name_ << '\n';
  }
  std::cout << '\n';

  std::cout << "Shortest weighted path from 0 to 6:\n";
  path = graphlib::shortest_pos_weight_path(&tiny_ewd,
                                            tiny_ewd.GetInternalVertexPtr(v0),
                                            tiny_ewd.GetInternalVertexPtr(v6));
  if (!path.empty()) {
    while (path.size() > 1) {
      std::cout << path.top()->name_ << " -> ";
      path.pop();
    }
    std::cout << path.top()->name_ << '\n';
  }
  std::cout << '\n';
}

void dijkstras_check() {
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> distribution(-10, 10);

  Vertex2d v0(distribution(mt), distribution(mt)),
      v1(distribution(mt), distribution(mt)),
      v2(distribution(mt), distribution(mt)),
      v3(distribution(mt), distribution(mt)),
      v4(distribution(mt), distribution(mt)),
      v5(distribution(mt), distribution(mt)),
      v6(distribution(mt), distribution(mt)),
      v7(distribution(mt), distribution(mt));

  Graph2d::Input2dAL al = {
      {v0, {v4, v2}}, {v1, {v3}},         {v2, {v7}},         {v3, {v6}},
      {v4, {v5, v7}}, {v5, {v4, v7, v1}}, {v6, {v2, v0, v4}}, {v7, {v5, v3}}};
  Graph2d graph(al, true);

  std::cout << "Shortest paths parent tree:\n";
  std::cout << "search root: " << v0.name_ << '\n';
  graphlib::dijkstra(&graph, graph.GetInternalVertexPtr(v0));
  for (const auto& v : graph.GetVertexMap()) {
    if (v.first != v0) {
      std::cout << v.first.name_ << " parent: " << v.first.parent_->name_
                << '\n';
    }
  }
  std::cout << '\n';

  std::cout << "Shortest weighted path from 0 to 1:\n";
  std::stack<const Vertex*> path = graphlib::shortest_pos_weight_path(
      &graph, graph.GetInternalVertexPtr(v0), graph.GetInternalVertexPtr(v1));
  if (!path.empty()) {
    while (path.size() > 1) {
      std::cout << path.top()->name_ << " -> ";
      path.pop();
    }
    std::cout << path.top()->name_ << '\n';
  }
  std::cout << '\n';

  std::cout << "Shortest weighted path from 0 to 3:\n";
  path = graphlib::shortest_pos_weight_path(
      &graph, graph.GetInternalVertexPtr(v0), graph.GetInternalVertexPtr(v3));
  if (!path.empty()) {
    while (path.size() > 1) {
      std::cout << path.top()->name_ << " -> ";
      path.pop();
    }
    std::cout << path.top()->name_ << '\n';
  }
  std::cout << '\n';

  std::cout << "Shortest weighted path from 0 to 6:\n";
  path = graphlib::shortest_pos_weight_path(
      &graph, graph.GetInternalVertexPtr(v0), graph.GetInternalVertexPtr(v6));
  if (!path.empty()) {
    while (path.size() > 1) {
      std::cout << path.top()->name_ << " -> ";
      path.pop();
    }
    std::cout << path.top()->name_ << '\n';
  }
  std::cout << '\n';
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nodeHandle;

  std::cout << "=============\n";
  std::cout << "TINY_EWD_DIJKSTRAS\n\n";
  tiny_ewd_dijkstras();

  std::cout << "=============\n";
  std::cout << "DIJKSTRAS_CHECK\n\n";
  dijkstras_check();

  ros::spin();
  return 0;
}
