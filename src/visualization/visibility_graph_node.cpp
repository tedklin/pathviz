#include "pathviz/visibility_map/visibility_graph.hpp"

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "pathviz/visualization/interface.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "visibility_graph_node");
  ros::NodeHandle n;

  ros::Publisher marker_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // visualization_msgs::Marker marker;
  // marker.header.frame_id = "my_frame";
  // marker.header.stamp = ros::Time::now();
  // marker.ns = "my_namespace";
  // marker.id = 0;
  // marker.type = visualization_msgs::Marker::LINE_LIST;
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.scale.x = 0.1;
  // marker.color.r = 0.0f;
  // marker.color.g = 1.0f;
  // marker.color.b = 0.0f;
  // marker.color.a = 1.0;
  // marker.lifetime = ros::Duration();
  // marker.pose.position.x = 0;
  // marker.pose.position.y = 0;
  // marker.pose.position.z = 0;
  // marker.pose.orientation.x = 0.0;
  // marker.pose.orientation.y = 0.0;
  // marker.pose.orientation.z = 0.0;
  // marker.pose.orientation.w = 1.0;

  // geometry_msgs::Point p1, p2, p3, p4;
  // p1.x = 0;
  // p1.y = 0;
  // p2.x = 3;
  // p2.y = 3;
  // p3.x = 2;
  // p3.y = 1;
  // p4.x = -3;
  // p4.y = 3;
  // std::vector<geometry_msgs::Point> points{p1, p2, p3, p4};
  // marker.points = points;

  graphlib::Graph2d graph(false);
  graph.AddEdge(graphlib::Vertex2d(0, 0), graphlib::Vertex2d(3, 3));
  graph.AddEdge(graphlib::Vertex2d(0, 0), graphlib::Vertex2d(3, 1));
  graph.AddEdge(graphlib::Vertex2d(-3, -3), graphlib::Vertex2d(-1, -1));
  auto marker = pathviz::visualization::to_marker(graph);

  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  marker_pub.publish(marker);

  ros::spin();
  return 0;
}
