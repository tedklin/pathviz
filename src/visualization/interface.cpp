#include "pathviz/visualization/interface.hpp"

namespace pathviz {
namespace visualization {

geometry_msgs::Point to_geometry_msg(const geometry_2d::Point& point) {
  geometry_msgs::Point p;
  p.x = point.x;
  p.y = point.y;
  return p;
}

geometry_msgs::Point to_geometry_msg(const graphlib::Vertex2d& vertex) {
  geometry_msgs::Point p;
  p.x = vertex.x_;
  p.y = vertex.y_;
  return p;
}

void set_marker_defaults(visualization_msgs::Marker& marker) {
  marker.header.frame_id = "my_frame";
  marker.header.stamp = ros::Time::now();
  marker.lifetime = ros::Duration();

  marker.color.a = 1.0;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
}

void set_marker_color(visualization_msgs::Marker& marker, Color color) {
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
}

visualization_msgs::Marker to_marker(const visibility_map::Terrain& terrain) {
  visualization_msgs::Marker marker;
  set_marker_defaults(marker);
  set_marker_color(marker, color::BLUE);

  marker.ns = "terrain";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.pose.position.z = 0.02;
  std::vector<geometry_msgs::Point> point_list;

  for (const auto& polygon : terrain.AllObstacles()) {
    for (const auto& edge : polygon.AllEdges()) {
      point_list.push_back(to_geometry_msg(edge.from));
      point_list.push_back(to_geometry_msg(edge.to));
    }
  }

  marker.points = point_list;
  return marker;
}

visualization_msgs::Marker to_marker(const graphlib::Graph2d& graph) {
  visualization_msgs::Marker marker;
  set_marker_defaults(marker);
  set_marker_color(marker, color::GREEN);

  marker.ns = "graph";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.025;
  std::vector<geometry_msgs::Point> point_list;

  for (const auto& v : graph.GetAdjacencyMap()) {
    for (const auto& adj : v.second) {
      point_list.push_back(
          to_geometry_msg(*(dynamic_cast<const graphlib::Vertex2d*>(v.first))));
      point_list.push_back(to_geometry_msg(
          *(dynamic_cast<const graphlib::Vertex2d*>(adj.first))));
    }
  }

  marker.points = point_list;
  return marker;
}

void publish_terrain(const visibility_map::Terrain& terrain,
                     ros::Publisher* marker_pub) {
  auto terrain_marker = visualization::to_marker(terrain);
  marker_pub->publish(terrain_marker);
}

void publish_graph(const graphlib::Graph2d& graph, ros::Publisher* marker_pub) {
  auto graph_marker = visualization::to_marker(graph);
  marker_pub->publish(graph_marker);
}

}  // namespace visualization
}  // namespace pathviz
