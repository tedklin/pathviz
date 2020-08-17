#include "pathviz/visibility_map/visibility_graph.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>

namespace pathviz {
namespace visibility_map {

AnimationManager::AnimationManager(
    ros::Publisher* marker_pub, double update_rate_ms,
    const visualization::PointDescriptor& current_source_descriptor,
    const visualization::PointDescriptor& current_target_descriptor,
    const visualization::LineDescriptor& current_edge_descriptor,
    const visualization::LineDescriptor& active_edges_descriptor,
    const visualization::LineDescriptor& valid_edges_descriptor,
    const visualization::LineDescriptor& invalid_edges_descriptor)
    : marker_pub_(marker_pub), update_rate_ms_(update_rate_ms) {
  current_source_vertex_ = std::make_unique<visualization::PointListManager>(
      marker_pub, "current_source_vertex", current_source_descriptor);
  current_target_vertex_ = std::make_unique<visualization::PointListManager>(
      marker_pub, "current_target_vertex", current_target_descriptor);
  current_edge_ = std::make_unique<visualization::LineListManager>(
      marker_pub, "current_edge", current_edge_descriptor);
  active_edges_ = std::make_unique<visualization::LineListManager>(
      marker_pub, "active_edges", active_edges_descriptor);
  valid_edges_ = std::make_unique<visualization::LineListManager>(
      marker_pub, "valid_edges", valid_edges_descriptor);
  invalid_edges_ = std::make_unique<visualization::LineListManager>(
      marker_pub, "invalid_edges", invalid_edges_descriptor);
  total_valid_edges_ = std::make_unique<visualization::LineListManager>(
      marker_pub, "total_valid_edges", valid_edges_descriptor);
}

bool is_visible(const geometry_2d::Terrain& terrain,
                const geometry_2d::Point& source,
                const geometry_2d::Point& target,
                const std::vector<geometry_2d::LineSegment>& active_edges) {
  // Test if target is part of the same polygon as the source. Try-catch is
  // needed because the input points are not necessarily part of any polygon.
  bool same_polygon = true;
  try {
    same_polygon =
        terrain.GetObstacleIndex(source) == terrain.GetObstacleIndex(target);
  } catch (std::runtime_error e) {
    same_polygon = false;
  }

  // If the target is part of the same polygon as the source, check if the line
  // from source to target goes through the inside of the polygon. This is
  // accomplished by an angle bounds check using the incident edges of source.
  if (same_polygon) {
    auto incident_edges = terrain.GetObstacle(source).IncidentEdges(source);

    // "Left" and "right" from the transformed perspective where the y-axis is
    // the bisection of the interior angle formed by the incident edges. This
    // helps us determine the edge case where the actual angle bound crosses the
    // negative x-axis (where our angle calculation jumps from -pi to pi).
    double left_angle =
        geometry_2d::angle_from_horizontal(incident_edges.first);
    double right_angle =
        geometry_2d::angle_from_horizontal(incident_edges.second);
    double target_angle = geometry_2d::angle_from_horizontal(source, target);

    if (left_angle < right_angle) {
      // case where angle bound crosses negative x-axis
      if ((-geometry_2d::pi < target_angle && target_angle < left_angle) ||
          (right_angle < target_angle && target_angle <= geometry_2d::pi)) {
        return false;
      }
    } else {
      // regular case
      if (right_angle < target_angle && target_angle < left_angle) {
        return false;
      }
    }
  }

  // Check if the line from source to target intersects any active edges.
  // Current implementation simply searches all active edges for intersections.
  // If in need for speed, we can maintain a balanced search tree for active
  // edges and only check the active edge closest to source (de Berg et al).
  for (const auto& edge : active_edges) {
    if (geometry_2d::is_intersecting(geometry_2d::LineSegment{source, target},
                                     edge)) {
      return false;
    }
  }
  return true;
}

// Sort all non-source Points by sweep order.
// The implementation here sweeps counterclockwise starting from the negative
// x-axis w.r.t. the source Point (natural ordering with atan2).
std::map<double, std::vector<geometry_2d::Point>> sort_sweep_points(
    const geometry_2d::Terrain& terrain, const geometry_2d::Point& source) {
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
  return ordered_points;
}

// Initialize container for edges currrently crossed by our sweep line. The
// order of the Points of each edge in this container matters (same direction
// as ccw sweep).
std::vector<geometry_2d::LineSegment> initialize_active_edges(
    const geometry_2d::Terrain& terrain, const geometry_2d::Point& source) {
  std::vector<geometry_2d::LineSegment> active_edges;
  for (const auto& polygon : terrain.AllObstacles()) {
    for (const auto& edge : polygon.AllEdges()) {
      // If intersecting negative x-axis w.r.t. source point (the start of the
      // sweep), then add to initial active edges.
      if (!geometry_2d::is_horizontal(edge) &&
          std::max(edge.from.x, edge.to.x) < source.x &&
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
  return active_edges;
}

// Lee's rotational plane sweep algorithm.
std::set<geometry_2d::Point> get_visible_vertices(
    const geometry_2d::Terrain& terrain, const geometry_2d::Point& source,
    bool verbose, AnimationManager* animation_manager) {
  if (animation_manager) {
    animation_manager->current_source_vertex_->AddPoint(source);
    animation_manager->current_source_vertex_->Publish();
    visualization::sleep_ms(animation_manager->update_rate_ms_);
  }

  std::set<geometry_2d::Point> visible_vertices;

  auto ordered_points = sort_sweep_points(terrain, source);
  if (verbose) {
    std::cout << "========================\n";
    std::cout << "source vertex: " << geometry_2d::to_string(source) << '\n';
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

  auto active_edges = initialize_active_edges(terrain, source);
  if (verbose) {
    std::cout << "initial active edges (negative x-axis):\n";
    for (const auto& edge : active_edges) {
      std::cout << geometry_2d::to_string(edge) << '\n';
    }
    std::cout << "\n\nbegin sweep\n\n";
  }
  if (animation_manager) {
    for (const auto& edge : active_edges) {
      animation_manager->active_edges_->AddLine(edge);
    }
    animation_manager->active_edges_->Publish();
    visualization::sleep_ms(animation_manager->update_rate_ms_);
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
      if (verbose) {
        std::cout << "processing " << geometry_2d::to_string(point) << ":\n";
      }
      if (animation_manager) {
        animation_manager->current_target_vertex_->Clear();
        animation_manager->current_target_vertex_->AddPoint(point);
        animation_manager->current_target_vertex_->Publish();

        animation_manager->current_edge_->Clear();
        animation_manager->current_edge_->AddLine({source, point});
        animation_manager->current_edge_->Publish();
        visualization::sleep_ms(animation_manager->update_rate_ms_);
      }

      // Add visible vertices.
      if (!blocked && is_visible(terrain, source, point, active_edges)) {
        visible_vertices.insert(point);

        if (verbose) {
          std::cout << "VISIBLE!\n";
        }
        if (animation_manager) {
          animation_manager->total_valid_edges_->AddLine({source, point});
          animation_manager->valid_edges_->AddLine({source, point});
          animation_manager->valid_edges_->Publish();
        }
      } else {
        blocked = true;

        if (verbose) {
          std::cout << "NOT VISIBLE!\n";
        }
        if (animation_manager) {
          animation_manager->invalid_edges_->AddLine({source, point});
          animation_manager->invalid_edges_->Publish();
        }
      }

      if (animation_manager) {
        visualization::sleep_ms(animation_manager->update_rate_ms_);
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
        if (animation_manager) {
          animation_manager->active_edges_->RemoveLine(*iter);
        }
        active_edges.erase(iter);
      } else {
        if (animation_manager) {
          animation_manager->active_edges_->AddLine(incident_edges.first);
        }
        active_edges.push_back(incident_edges.first);
      }

      iter = std::find(active_edges.begin(), active_edges.end(),
                       geometry_2d::reverse(incident_edges.second));
      if (iter != active_edges.end()) {
        if (animation_manager) {
          animation_manager->active_edges_->RemoveLine(*iter);
        }
        active_edges.erase(iter);
      } else {
        if (animation_manager) {
          animation_manager->active_edges_->AddLine(incident_edges.second);
        }
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
      if (animation_manager) {
        animation_manager->active_edges_->Publish();
        visualization::sleep_ms(animation_manager->update_rate_ms_);
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
  if (animation_manager) {
    animation_manager->active_edges_->Clear();
    animation_manager->active_edges_->Publish();

    animation_manager->invalid_edges_->Clear();
    animation_manager->invalid_edges_->Publish();

    animation_manager->current_edge_->Clear();
    animation_manager->current_edge_->Publish();

    animation_manager->current_target_vertex_->Clear();
    animation_manager->current_target_vertex_->Publish();

    animation_manager->current_source_vertex_->Clear();
    animation_manager->current_source_vertex_->Publish();

    visualization::sleep_ms(animation_manager->update_rate_ms_);

    animation_manager->valid_edges_->Clear();
    animation_manager->valid_edges_->Publish();
  }

  return visible_vertices;
}

graphlib::Graph2d get_visibility_graph(const geometry_2d::Terrain& terrain,
                                       const geometry_2d::Point& start,
                                       const geometry_2d::Point& goal,
                                       bool verbose,
                                       AnimationManager* animation_manager) {
  graphlib::Graph2d graph(false);

  std::set<pathviz::geometry_2d::Point> all_vertices = terrain.AllVertices();
  all_vertices.insert(start);
  all_vertices.insert(goal);

  for (const auto& point : all_vertices) {
    auto visible_vertices =
        get_visible_vertices(terrain, point, verbose, animation_manager);
    for (const auto& visible_vertex : visible_vertices) {
      graph.AddEdge(graphlib::Vertex2d(point.x, point.y),
                    graphlib::Vertex2d(visible_vertex.x, visible_vertex.y));
    }

    if (animation_manager) {
      animation_manager->total_valid_edges_->Publish();
      visualization::sleep_ms(animation_manager->update_rate_ms_ * 3);

      animation_manager->total_valid_edges_->Hide();
    }
  }
  return graph;
}

}  // namespace visibility_map
}  // namespace pathviz
