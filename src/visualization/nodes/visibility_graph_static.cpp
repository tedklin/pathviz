#include "pathviz/visibility_map/visibility_graph.hpp"

#include <ros/ros.h>

#include "pathviz/geometry/geometry_2d.hpp"
#include "pathviz/visualization/interface.hpp"
#include "pathviz/visualization/static_helpers.hpp"

using namespace pathviz;
using namespace pathviz::visualization;

int main(int argc, char** argv) {
  ros::init(argc, argv, "visibility_graph_static");
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
  geometry_2d::Polygon p2({{-2, -0.7}, {-5.3, 0.8}, {-6, -1.8}, {-3.2, -2.3}});
  geometry_2d::Polygon p3({{2.7, 0.5}, {0, -2.7}, {5, -3}});

  visibility_map::Terrain terrain({p1, p2, p3});

  LineDescriptor terrain_descriptor(color::BLUE, 0.05, 0.02);
  publish_static_terrain(&marker_pub, terrain, terrain_descriptor);

  geometry_2d::Point start{-7, 4};
  geometry_2d::Point goal{7, -2};
  PointDescriptor endpoint_descriptor(color::YELLOW, 0.2, 0.04);
  publish_static_point(&marker_pub, start, endpoint_descriptor);
  publish_static_point(&marker_pub, goal, endpoint_descriptor);

  graphlib::Graph2d vis_graph =
      visibility_map::get_visibility_graph(terrain, start, goal);

  LineDescriptor graph_edge_descriptor(color::GREEN, 0.025, 0);
  PointDescriptor graph_vertex_descriptor(color::PURPLE, 0.2, 0);
  publish_static_graph(&marker_pub, vis_graph, graph_vertex_descriptor,
                       graph_edge_descriptor);

  ros::spin();
  return 0;
}
