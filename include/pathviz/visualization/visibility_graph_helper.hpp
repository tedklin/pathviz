#pragma once

#include <ros/ros.h>

#include "graphlib/geometry/graph_2d.hpp"

#include "pathviz/visibility_map/visibility_graph.hpp"
#include "pathviz/visualization/interface.hpp"

namespace pathviz {
namespace visualization {

void publish_static_terrain(ros::Publisher* marker_pub,
                            const visibility_map::Terrain& terrain,
                            const LineListDescriptor& line_descriptor);

void publish_static_graph(ros::Publisher* marker_pub,
                          const graphlib::Graph2d& graph,
                          const PointListDescriptor& point_descriptor,
                          const LineListDescriptor& line_descriptor);

}  // namespace visualization
}  // namespace pathviz
