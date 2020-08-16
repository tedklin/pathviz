#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "pathviz/geometry/geometry_2d.hpp"

namespace pathviz {
namespace visualization {

struct Color {
  const float r, g, b, a;
};

namespace color {
constexpr Color BLACK{0, 0, 0, 1}, RED{1, 0, 0, 1}, GREEN{0, 1, 0, 1},
    BLUE{0, 0, 1, 1}, WHITE{1, 1, 1, 1}, YELLOW{1, 1, 0, 1},
    ORANGE{1, 0.5, 0, 1}, PURPLE{1, 0, 1, 1}, TRANSPARENT{0, 0, 0, 0.0000001};
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

class MarkerManager {
 public:
  MarkerManager(ros::Publisher* marker_pub, const std::string& name);

  virtual ~MarkerManager() = default;

  virtual void Publish() = 0;

  void Hide();

 protected:
  ros::Publisher* marker_pub_;
  visualization_msgs::Marker marker_;
};

class LineListManager : public MarkerManager {
 public:
  LineListManager(ros::Publisher* marker_pub, const std::string& name,
                  const LineListDescriptor& descriptor);

  void AddLine(const geometry_2d::LineSegment& line);
  void RemoveLine(const geometry_2d::LineSegment& line);
  void Clear();

  void Publish() override;

 private:
  std::vector<geometry_2d::LineSegment> lines_;
  Color color_;
};

class PointListManager : public MarkerManager {
 public:
  PointListManager(ros::Publisher* marker_pub, const std::string& name,
                   const PointListDescriptor& descriptor);

  void AddPoint(const geometry_2d::Point& point);
  void RemovePoint(const geometry_2d::Point& point);
  void Clear();

  void Publish() override;

 private:
  std::vector<geometry_2d::Point> points_;
  Color color_;
};

void sleep_ms(int ms);

}  // namespace visualization
}  // namespace pathviz
