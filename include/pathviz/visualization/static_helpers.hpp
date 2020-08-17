#pragma once

#include <stack>

#include <ros/ros.h>

#include "graphlib/geometry/graph_2d.hpp"

#include "pathviz/geometry/geometry_2d.hpp"
#include "pathviz/visibility_map/visibility_graph.hpp"
#include "pathviz/visualization/interface.hpp"

namespace pathviz {
namespace visualization {

void publish_static_point(ros::Publisher* marker_pub,
                          const geometry_2d::Point point,
                          const PointDescriptor& point_descriptor);

void publish_static_terrain(ros::Publisher* marker_pub,
                            const visibility_map::Terrain& terrain,
                            const LineDescriptor& line_descriptor);

void publish_static_graph(ros::Publisher* marker_pub,
                          const graphlib::Graph2d& graph,
                          const PointDescriptor& point_descriptor,
                          const LineDescriptor& line_descriptor);

void publish_static_path(ros::Publisher* marker_pub,
                         std::stack<const graphlib::Vertex2d*> path,
                         const LineDescriptor& line_descriptor);

}  // namespace visualization
}  // namespace pathviz
