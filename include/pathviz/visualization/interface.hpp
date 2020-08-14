#pragma once

#include <string>
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

struct Color {
  const float r, g, b;
};

// TODO: can this be expressed more cleanly?
namespace color {
constexpr Color BLACK{0, 0, 0}, RED{1, 0, 0}, GREEN{0, 1, 0}, BLUE{0, 0, 1},
    WHITE{1, 1, 1}, YELLOW{1, 1, 0}, ORANGE{1, 0.5, 0}, PURPLE{1, 0, 1};
}  // namespace color

// Static visualization.
visualization_msgs::Marker to_marker(const visibility_map::Terrain& terrain);
visualization_msgs::Marker to_marker(const graphlib::Graph2d& graph);
void publish_terrain(const visibility_map::Terrain& terrain,
                     ros::Publisher* marker_pub);
void publish_graph(const graphlib::Graph2d& graph, ros::Publisher* marker_pub);

// Animated visualization.
class AnimationManager {
 public:
  AnimationManager(const std::string& name, ros::Publisher* marker_pub)
      : ns_(name), marker_pub_(marker_pub) {}

  void DisplayLine(const geometry_2d::LineSegment& line, Color color);
  void HideLine(const geometry_2d::LineSegment& line);

  void DisplayPoint(const geometry_2d::Point& point, Color color);
  void HidePoint(const geometry_2d::Point& point);

 private:
  std::string ns_;
  ros::Publisher* marker_pub_;

  // Index in vector is used as marker id.
  std::vector<geometry_2d::LineSegment> lines_;
  std::vector<geometry_2d::Point> points_;
};

}  // namespace visualization
}  // namespace pathviz
