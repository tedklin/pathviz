#include "pathviz/visualization/visibility_graph_helper.hpp"

namespace pathviz {
namespace visualization {

static int terrain_index = 0;
void publish_static_terrain(ros::Publisher* marker_pub,
                            const visibility_map::Terrain& terrain,
                            const LineListDescriptor& line_descriptor) {
  LineListManager line_manager(
      marker_pub, "terrain" + std::to_string(terrain_index), line_descriptor);

  for (const auto& polygon : terrain.AllObstacles()) {
    for (const auto& edge : polygon.AllEdges()) {
      line_manager.AddLine(edge);
    }
  }
  line_manager.Publish();
  ++terrain_index;
}

static int graph_index = 0;
void publish_static_graph(ros::Publisher* marker_pub,
                          const graphlib::Graph2d& graph,
                          const PointListDescriptor& point_descriptor,
                          const LineListDescriptor& line_descriptor) {
  PointListManager point_manager(marker_pub,
                                 "graph_vertices" + std::to_string(graph_index),
                                 point_descriptor);
  LineListManager line_manager(
      marker_pub, "graph_edges" + std::to_string(graph_index), line_descriptor);

  for (const auto& v : graph.GetAdjacencyMap()) {
    graphlib::Vertex2d v1 = *(dynamic_cast<const graphlib::Vertex2d*>(v.first));
    point_manager.AddPoint(geometry_2d::Point{v1.x_, v1.y_});
    for (const auto& adj : v.second) {
      graphlib::Vertex2d v2 =
          *(dynamic_cast<const graphlib::Vertex2d*>(adj.first));
      line_manager.AddLine({{v1.x_, v1.y_}, {v2.x_, v2.y_}});
    }
  }
  point_manager.Publish();
  line_manager.Publish();
  ++graph_index;
}

}  // namespace visualization
}  // namespace pathviz
