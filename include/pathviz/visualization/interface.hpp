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

struct Color {
  const float r, g, b, a;
};

// TODO: can this be expressed more cleanly?
namespace color {
constexpr Color BLACK{0, 0, 0, 1}, RED{1, 0, 0, 1}, GREEN{0, 1, 0, 1},
    BLUE{0, 0, 1, 1}, WHITE{1, 1, 1, 1}, YELLOW{1, 1, 0, 1},
    ORANGE{1, 0.5, 0, 1}, PURPLE{1, 0, 1, 1}, TRANSPARENT{0, 0, 0, 0};
}  // namespace color

// Static visualization.
visualization_msgs::Marker to_marker(const visibility_map::Terrain& terrain);
visualization_msgs::Marker to_marker(const graphlib::Graph2d& graph);
void publish_terrain(const visibility_map::Terrain& terrain,
                     ros::Publisher* marker_pub);
void publish_graph(const graphlib::Graph2d& graph, ros::Publisher* marker_pub);

// Animated visualization.
class Animator {
 public:
  Animator(ros::Publisher* marker_pub, const std::string& name,
           const Color& color);

  virtual ~Animator() = default;

  virtual void Publish() = 0;

 protected:
  ros::Publisher* marker_pub_;
  visualization_msgs::Marker marker_;
};

class LineListAnimator : public Animator {
 public:
  LineListAnimator(ros::Publisher* marker_pub, const std::string& name,
                   const Color& color);

  void AddLine(const geometry_2d::LineSegment& line);
  void RemoveLine(const geometry_2d::LineSegment& line);
  void Clear();

  void Publish() override;

 private:
  std::vector<geometry_2d::LineSegment> lines_;
};

class PointListAnimator : public Animator {
 public:
  PointListAnimator(ros::Publisher* marker_pub, const std::string& name,
                    const Color& color);

  void AddPoint(const geometry_2d::Point& point);
  void RemovePoint(const geometry_2d::Point& point);
  void Clear();

  void Publish() override;

 private:
  std::vector<geometry_msgs::Point> point_msgs_;
};

void sleep_ms(int ms);

}  // namespace visualization
}  // namespace pathviz
