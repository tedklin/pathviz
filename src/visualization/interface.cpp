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

MarkerManager::MarkerManager(ros::Publisher* marker_pub,
                             const std::string& name)
    : marker_pub_(marker_pub) {
  set_marker_defaults(marker_);
  marker_.ns = name;
  marker_.id = 0;
  marker_.action = visualization_msgs::Marker::ADD;
}

void MarkerManager::Hide() {
  set_marker_color(marker_, color::TRANSPARENT);
  marker_pub_->publish(marker_);
}

LineListManager::LineListManager(ros::Publisher* marker_pub,
                                 const std::string& name,
                                 const LineListDescriptor& descriptor)
    : MarkerManager(marker_pub, name), color_(descriptor.color_) {
  marker_.type = visualization_msgs::Marker::LINE_LIST;
  marker_.scale.x = descriptor.width_;
  marker_.pose.position.z = descriptor.z_;
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
  if (!lines_.empty()) {
    std::vector<geometry_msgs::Point> point_list;
    for (const auto& line : lines_) {
      point_list.push_back(to_geometry_msg(line.from));
      point_list.push_back(to_geometry_msg(line.to));
    }
    marker_.points = point_list;  // TODO: convert to rvalue to move?
    set_marker_color(marker_, color_);
  } else {
    // If the vector of lines we want to display is empty but we don't want to
    // delete the marker itself yet, we can instead pass in an arbitrary
    // nonzero, even-numbered point list to the marker and set the color to
    // TRANSPARENT.
    geometry_msgs::Point p1, p2;
    p1.x = 0;
    p1.y = 0;
    p2.x = 1;
    p2.y = 1;
    marker_.points = std::vector<geometry_msgs::Point>{p1, p2};
    set_marker_color(marker_, color::TRANSPARENT);
  }

  marker_pub_->publish(marker_);
  sleep_ms(25);  // rviz can't handle too many markers at once
}

PointListManager::PointListManager(ros::Publisher* marker_pub,
                                   const std::string& name,
                                   const PointListDescriptor& descriptor)
    : MarkerManager(marker_pub, name), color_(descriptor.color_) {
  marker_.type = visualization_msgs::Marker::POINTS;
  marker_.scale.x = descriptor.size_;
  marker_.scale.y = descriptor.size_;
  marker_.pose.position.z = descriptor.z_;
}

void PointListManager::AddPoint(const geometry_2d::Point& point) {
  if (std::find(points_.begin(), points_.end(), point) == points_.end()) {
    points_.push_back(point);
  }
}

void PointListManager::RemovePoint(const geometry_2d::Point& point) {
  auto iter = std::find(points_.begin(), points_.end(), point);
  if (std::find(points_.begin(), points_.end(), point) != points_.end()) {
    points_.erase(iter);
  }
}

void PointListManager::Clear() { points_.clear(); }

void PointListManager::Publish() {
  if (!points_.empty()) {
    std::vector<geometry_msgs::Point> point_list;
    for (const auto& point : points_) {
      point_list.push_back(to_geometry_msg(point));
    }
    marker_.points = point_list;  // TODO: convert to rvalue to move?
    set_marker_color(marker_, color_);
  } else {
    // If the vector of points we want to display is empty but we don't want to
    // delete the marker itself yet, we can instead pass in an arbitrary nonzero
    // point list to the marker and set the color to TRANSPARENT.
    geometry_msgs::Point p;
    p.x = 0;
    p.y = 0;
    marker_.points = std::vector<geometry_msgs::Point>{p};
    set_marker_color(marker_, color::TRANSPARENT);
  }

  marker_pub_->publish(marker_);
  sleep_ms(25);  // rviz can't handle too many markers at once
}

void sleep_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

}  // namespace visualization
}  // namespace pathviz
