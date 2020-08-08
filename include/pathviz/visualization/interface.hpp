#pragma once

#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "graphlib/geometry/graph_2d.hpp"

#include "pathviz/geometry/geometry_2d.hpp"
#include "pathviz/visibility_map/visibility_graph.hpp"

namespace pathviz {
namespace visualization {

geometry_msgs::Point to_geometry_msg(const geometry_2d::Point& point);

geometry_msgs::Point to_geometry_msg(const graphlib::Vertex2d& vertex);

std::vector<visualization_msgs::Marker> to_marker_list(
    const visibility_map::Terrain& terrain);

visualization_msgs::Marker to_marker(const graphlib::Graph2d& graph);

}  // namespace visualization
}  // namespace pathviz
