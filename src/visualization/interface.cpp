#include "pathviz/visualization/interface.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

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

void set_marker_color(visualization_msgs::Marker& marker, Color color) {
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;
  marker.color.a = color.a;
}

void set_marker_defaults(visualization_msgs::Marker& marker) {
  marker.header.frame_id = "my_frame";
  marker.header.stamp = ros::Time::now();
  marker.lifetime = ros::Duration();

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  set_marker_color(marker, color::BLACK);
}

VizManager::VizManager(ros::Publisher* marker_pub, const std::string& name)
    : marker_pub_(marker_pub) {
  set_marker_defaults(marker_);
  marker_.ns = name;
  marker_.id = 0;
  marker_.action = visualization_msgs::Marker::ADD;
}

LineListManager::LineListManager(ros::Publisher* marker_pub,
                                 const std::string& name,
                                 const LineListDescriptor& descriptor)
    : VizManager(marker_pub, name) {
  marker_.type = visualization_msgs::Marker::LINE_LIST;
  marker_.scale.x = descriptor.width_;
  marker_.pose.position.z = descriptor.z_;
  set_marker_color(marker_, descriptor.color_);
}

void LineListManager::AddLine(const geometry_2d::LineSegment& line) {
  if (std::find(lines_.begin(), lines_.end(), line) == lines_.end()) {
    lines_.push_back(line);
  }
}

void LineListManager::RemoveLine(const geometry_2d::LineSegment& line) {
  auto iter = std::find(lines_.begin(), lines_.end(), line);
  if (std::find(lines_.begin(), lines_.end(), line) != lines_.end()) {
    lines_.erase(iter);
  }
}

void LineListManager::Clear() { lines_.clear(); }

void LineListManager::Publish() {
  if (lines_.empty()) {
    marker_.action = visualization_msgs::Marker::DELETE;
    marker_pub_->publish(marker_);
    return;
  }

  std::vector<geometry_msgs::Point> point_list;
  for (const auto& line : lines_) {
    point_list.push_back(to_geometry_msg(line.from));
    point_list.push_back(to_geometry_msg(line.to));
  }
  marker_.points = point_list;  // TODO: convert to rvalue to move?

  marker_pub_->publish(marker_);
}

PointListManager::PointListManager(ros::Publisher* marker_pub,
                                   const std::string& name,
                                   const PointListDescriptor& descriptor)
    : VizManager(marker_pub, name) {
  marker_.type = visualization_msgs::Marker::POINTS;
  marker_.scale.x = descriptor.size_;
  marker_.scale.y = descriptor.size_;
  marker_.pose.position.z = descriptor.z_;
  set_marker_color(marker_, descriptor.color_);
}

void PointListManager::AddPoint(const geometry_2d::Point& point) {
  auto iter =
      std::find_if(point_msgs_.begin(), point_msgs_.end(),
                   [&point](const geometry_msgs::Point& point_msg) {
                     return point.x == point_msg.x && point.y == point_msg.y;
                   });
  if (iter == point_msgs_.end()) {
    point_msgs_.push_back(to_geometry_msg(point));
  }
}

void PointListManager::RemovePoint(const geometry_2d::Point& point) {
  auto iter =
      std::find_if(point_msgs_.begin(), point_msgs_.end(),
                   [&point](const geometry_msgs::Point& point_msg) {
                     return point.x == point_msg.x && point.y == point_msg.y;
                   });
  if (iter != point_msgs_.end()) {
    point_msgs_.erase(iter);
  }
}

void PointListManager::Clear() { point_msgs_.clear(); }

void PointListManager::Publish() {
  if (point_msgs_.empty()) {
    marker_.action = visualization_msgs::Marker::DELETE;
    marker_pub_->publish(marker_);
    return;
  }
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.points = point_msgs_;
  marker_pub_->publish(marker_);
}

void sleep_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static int terrain_index = 0;
void publish_static_terrain(ros::Publisher* marker_pub,
                            const visibility_map::Terrain& terrain,
                            const LineListDescriptor& line_descriptor) {
  LineListManager line_manager(
      marker_pub, "terrain" + std::to_string(terrain_index++), line_descriptor);

  for (const auto& polygon : terrain.AllObstacles()) {
    for (const auto& edge : polygon.AllEdges()) {
      line_manager.AddLine(edge);
    }
  }
  line_manager.Publish();
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
  graph_index++;
}

}  // namespace visualization
}  // namespace pathviz
