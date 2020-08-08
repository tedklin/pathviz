#include "pathviz/visibility_map/visibility_graph.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <map>

namespace pathviz {
namespace visibility_map {

Terrain::Terrain(std::vector<geometry_2d::Polygon> obstacles) {
  for (const auto& obstacle : obstacles) {
    AddObstacle(obstacle);
  }
}

void Terrain::AddObstacle(const geometry_2d::Polygon& obstacle) {
  for (const geometry_2d::Point& p : obstacle.polygon_) {
    obstacle_index_.insert(std::make_pair(p, obstacles_.size()));
  }
  obstacles_.push_back(obstacle);
}

const std::vector<geometry_2d::Polygon>& Terrain::AllObstacles() const {
  return obstacles_;
}

std::set<geometry_2d::Point> Terrain::AllVertices() const {
  std::set<geometry_2d::Point> vertices;
  for (const auto& obstacle : obstacles_) {
    vertices.insert(obstacle.polygon_.cbegin(), obstacle.polygon_.cend());
  }
  return vertices;
}

const geometry_2d::Polygon& Terrain::GetObstacle(
    const geometry_2d::Point& vertex) const {
  if (obstacle_index_.find(vertex) == obstacle_index_.end()) {
    throw std::runtime_error(
        "visibility_graph::Terrain error: tried to access a vertex that "
        "doesn't exist");
  }
  return obstacles_.at(obstacle_index_.at(vertex));
}

// Current implementation simply searches all active edges for intersections. If
// in need for speed, we can maintain a balanced search tree for active edges.
bool is_visible(const geometry_2d::Point& source,
                const geometry_2d::Point& target,
                const std::vector<geometry_2d::LineSegment>& active_edges) {
  for (const auto& edge : active_edges) {
    if (geometry_2d::is_intersecting(geometry_2d::LineSegment{source, target},
                                     edge)) {
      return false;
    }
  }
  return true;
}

// Lee's rotational plane sweep algorithm.
// The implementation here sweeps counter-clockwise starting from the negative
// x-axis w.r.t. the source Point (natural ordering with atan2).
std::set<geometry_2d::Point> get_visible_vertices(
    const Terrain& terrain, const geometry_2d::Point& source, bool verbose) {
  std::set<geometry_2d::Point> visible_vertices;

  // Sort all non-source Points by sweep order.
  std::map<double, std::vector<geometry_2d::Point>> ordered_points;
  for (const auto& point : terrain.AllVertices()) {
    double angle = geometry_2d::angle_from_horizontal(source, point);
    if (point != source) {
      if (ordered_points.find(angle) == ordered_points.end()) {
        ordered_points[angle].push_back(point);
      } else {
        ordered_points.at(angle).push_back(point);
      }
    }
  }

  if (verbose) {
    std::cout << "sweep order:\n";
    for (const auto& angle : ordered_points) {
      std::cout << "points at angle " << std::to_string(angle.first)
                << " rads: ";
      for (const auto& point : angle.second) {
        std::cout << geometry_2d::to_string(point) << " | ";
      }
      std::cout << '\n';
    }
    std::cout << '\n';
  }

  // Initialize container for edges currrently crossed by our sweep line. The
  // order of the Points of each edge in this container matters (same direction
  // as ccw sweep).
  std::vector<geometry_2d::LineSegment> active_edges;
  for (const auto& polygon : terrain.AllObstacles()) {
    for (const auto& edge : polygon.AllEdges()) {
      // If intersecting negative x-axis, then add to initial active edges.
      if (!geometry_2d::is_horizontal(edge) &&
          geometry_2d::x_intercept(edge) < source.x &&
          std::max(edge.from.y, edge.to.y) > source.y &&
          std::min(edge.from.y, edge.to.y) < source.y) {
        // Ensure the edge we add is pointing downwards (ccw).
        if (edge.from.y > edge.to.y) {
          active_edges.push_back(edge);
        } else {
          active_edges.push_back(geometry_2d::reverse(edge));
        }
      }
    }
  }

  if (verbose) {
    std::cout << "initial active edges (negative x-axis):\n";
    for (const auto& edge : active_edges) {
      std::cout << geometry_2d::to_string(edge) << '\n';
    }
    std::cout << "\n\nbegin sweep\n\n";
  }

  // Sweep.
  for (auto iter = ordered_points.cbegin(); iter != ordered_points.cend();
       ++iter) {
    std::vector<geometry_2d::Point> colinear_points = iter->second;
    std::sort(colinear_points.begin(), colinear_points.end(),
              [&](const geometry_2d::Point& p1,
                  const geometry_2d::Point& p2) -> bool {
                geometry_2d::distance(source, p1) <
                    geometry_2d::distance(source, p2);
              });

    bool blocked = false;  // if a colinear point closer to the source is
                           // already known to be not visible, we can skip the
                           // usual visibility check
    for (const auto& point : colinear_points) {
      // Add visible vertices.
      if (!blocked && is_visible(source, point, active_edges)) {
        visible_vertices.insert(point);
      } else {
        blocked = true;
      }

      // Add new active edges and remove any that are actually ending at this
      // point.
      // Note that since we assume each Point is only in one polygon, all
      // possible edges that touch this point are given by the two incident
      // edges of this point's associated Polygon. Note also that each of these
      // two incident edges *must* be either already in active_edges, in which
      // case the edge is ending and must be removed, or not yet in
      // active_edges, in which case the edge is starting and must be added.
      auto incident_edges = terrain.GetObstacle(point).IncidentEdges(point);
      auto iter = std::find(active_edges.begin(), active_edges.end(),
                            geometry_2d::reverse(incident_edges.first));
      if (iter != active_edges.end()) {
        active_edges.erase(iter);
      } else {
        active_edges.push_back(incident_edges.first);
      }
      iter = std::find(active_edges.begin(), active_edges.end(),
                       geometry_2d::reverse(incident_edges.second));
      if (iter != active_edges.end()) {
        active_edges.erase(iter);
      } else {
        active_edges.push_back(incident_edges.second);
      }

      if (verbose) {
        std::cout << "active edges after processing "
                  << geometry_2d::to_string(point) << ":\n";
        for (const auto& edge : active_edges) {
          std::cout << geometry_2d::to_string(edge) << '\n';
        }
        std::cout << '\n';
      }
    }
  }

  if (verbose) {
    std::cout << "visible vertices from " << geometry_2d::to_string(source)
              << ":\n";
    for (const auto& point : visible_vertices) {
      std::cout << geometry_2d::to_string(point) << " | ";
    }
    std::cout << "\n\n";
  }
  return visible_vertices;
}

graphlib::Graph2d get_visibility_graph(const Terrain& terrain, bool verbose) {
  graphlib::Graph2d graph(false);
  for (const auto& point : terrain.AllVertices()) {
    auto visible_vertices = get_visible_vertices(terrain, point, verbose);
    for (const auto& visible_vertex : visible_vertices) {
      graph.AddEdge(graphlib::Vertex2d(point.x, point.y),
                    graphlib::Vertex2d(visible_vertex.x, visible_vertex.y));
    }
  }
  return graph;
}

}  // namespace visibility_map
}  // namespace pathviz
