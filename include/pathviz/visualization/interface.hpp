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

namespace color {
constexpr Color BLACK{0, 0, 0, 1}, RED{1, 0, 0, 1}, GREEN{0, 1, 0, 1},
    BLUE{0, 0, 1, 1}, WHITE{1, 1, 1, 1}, YELLOW{1, 1, 0, 1},
    ORANGE{1, 0.5, 0, 1}, PURPLE{1, 0, 1, 1}, TRANSPARENT{0, 0, 0, 0};
}  // namespace color

struct LineListDescriptor {
  LineListDescriptor(const Color& color, double width, double z)
      : color_(color), width_(width), z_(z) {}

  Color color_;
  double width_, z_;
};

struct PointListDescriptor {
  PointListDescriptor(const Color& color, double size, double z)
      : color_(color), size_(size), z_(z) {}

  Color color_;
  double size_, z_;
};

class VizManager {
 public:
  VizManager(ros::Publisher* marker_pub, const std::string& name);

  virtual ~VizManager() = default;

  virtual void Publish() = 0;

 protected:
  ros::Publisher* marker_pub_;
  visualization_msgs::Marker marker_;
};

class LineListManager : public VizManager {
 public:
  LineListManager(ros::Publisher* marker_pub, const std::string& name,
                  const LineListDescriptor& descriptor);

  void AddLine(const geometry_2d::LineSegment& line);
  void RemoveLine(const geometry_2d::LineSegment& line);
  void Clear();

  void Publish() override;

 private:
  std::vector<geometry_2d::LineSegment> lines_;
};

class PointListManager : public VizManager {
 public:
  PointListManager(ros::Publisher* marker_pub, const std::string& name,
                   const PointListDescriptor& descriptor);

  void AddPoint(const geometry_2d::Point& point);
  void RemovePoint(const geometry_2d::Point& point);
  void Clear();

  void Publish() override;

 private:
  std::vector<geometry_msgs::Point> point_msgs_;
};

void sleep_ms(int ms);

void publish_static_terrain(ros::Publisher* marker_pub,
                            const visibility_map::Terrain& terrain,
                            const LineListDescriptor& line_descriptor);

void publish_static_graph(ros::Publisher* marker_pub,
                          const graphlib::Graph2d& graph,
                          const PointListDescriptor& point_descriptor,
                          const LineListDescriptor& line_descriptor);

}  // namespace visualization
}  // namespace pathviz
