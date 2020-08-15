#include "pathviz/visibility_map/visibility_graph.hpp"

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "pathviz/visualization/interface.hpp"

using namespace pathviz;

int main(int argc, char** argv) {
  ros::init(argc, argv, "visibility_graph_node");
  ros::NodeHandle n;

  ros::Publisher marker_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  geometry_2d::Polygon p1(
      {{0, 0}, {0.5, 3}, {-0.7, 4.8}, {-4, 3.7}, {-3.3, 1.5}, {-1.5, 2.5}});
  geometry_2d::Polygon p2(
      {{-2.2, -0.7}, {-5.3, 0.8}, {-6, -1.8}, {-3.2, -2.3}});
  geometry_2d::Polygon p3({{2.7, 0}, {0, -2.7}, {5, -3}});

  visibility_map::Terrain terrain({p1, p2, p3});
  visualization::publish_terrain(terrain, &marker_pub);

  graphlib::Graph2d vis_graph = visibility_map::get_visibility_graph(terrain);
  // visualization::publish_graph(vis_graph, &marker_pub);

  visualization::LineListAnimator graph_animator(&marker_pub, "vis_graph",
                                                 visualization::color::ORANGE);
  for (const auto& v : vis_graph.GetAdjacencyMap()) {
    for (const auto& adj : v.second) {
      graphlib::Vertex2d v1 =
          *(dynamic_cast<const graphlib::Vertex2d*>(v.first));
      graphlib::Vertex2d v2 =
          *(dynamic_cast<const graphlib::Vertex2d*>(adj.first));
      graph_animator.AddLine({{v1.x_, v1.y_}, {v2.x_, v2.y_}});
      graph_animator.Publish();
      visualization::sleep_ms(250);
    }
  }
  for (const auto& v : vis_graph.GetAdjacencyMap()) {
    for (const auto& adj : v.second) {
      graphlib::Vertex2d v1 =
          *(dynamic_cast<const graphlib::Vertex2d*>(v.first));
      graphlib::Vertex2d v2 =
          *(dynamic_cast<const graphlib::Vertex2d*>(adj.first));
      graph_animator.RemoveLine({{v1.x_, v1.y_}, {v2.x_, v2.y_}});
      graph_animator.Publish();
      visualization::sleep_ms(250);
    }
  }

  ros::spin();
  return 0;
}
