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

visualization_msgs::Marker to_marker(const graphlib::Graph2d& graph) {
  visualization_msgs::Marker marker;
  set_marker_defaults(marker);

  marker.ns = "graph";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.color.g = 1.0f;
  std::vector<geometry_msgs::Point> point_list;

  for (const auto& v : graph.GetVertexMap()) {
    for (const auto& adj : v.second) {
      point_list.push_back(to_geometry_msg(
          *(dynamic_cast<const graphlib::Vertex2d*>(v.first.get()))));
      point_list.push_back(to_geometry_msg(
          *(dynamic_cast<const graphlib::Vertex2d*>(adj.first.get()))));
    }
  }

  marker.points = point_list;
  return marker;
}

}  // namespace visualization
}  // namespace pathviz
