#include "pathviz/graph_search/a_star.hpp"

#include <ros/ros.h>

#include "pathviz/visibility_map/visibility_graph.hpp"
#include "pathviz/visualization/interface.hpp"
#include "pathviz/visualization/static_helpers.hpp"

using namespace pathviz;
using namespace pathviz::visualization;

int main(int argc, char** argv) {
  ros::init(argc, argv, "a_star_animated");
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

  // First set up static visibility graph on which we run A*.

  LineDescriptor terrain_descriptor(color::TRANSPARENT, 0.1, 0);
  PointDescriptor endpoint_descriptor(color::YELLOW, 0.2, 0.02);
  LineDescriptor graph_edge_descriptor(color::BLACK, 0.025, 0);
  PointDescriptor graph_vertex_descriptor(color::TRANSPARENT, 0.2, 0);

  geometry_2d::Polygon p1(
      {{0, 0}, {0.5, 3}, {-0.7, 4.8}, {-4, 3.7}, {-3.3, 1.5}, {-1.5, 2.5}});
  geometry_2d::Polygon p2({{-2, -0.7}, {-5.3, 0.8}, {-6, -1.8}, {-3.2, -2.3}});
  geometry_2d::Polygon p3({{2.7, 0.5}, {0, -2.7}, {5, -3}});

  geometry_2d::Point start{-7, 4};
  geometry_2d::Point goal{7, -2};

  geometry_2d::Terrain terrain({p1, p2, p3});
  graphlib::Graph2d vis_graph =
      visibility_map::get_visibility_graph(terrain, start, goal);

  publish_static_terrain(&marker_pub, terrain, terrain_descriptor);
  publish_static_point(&marker_pub, start, endpoint_descriptor);
  publish_static_point(&marker_pub, goal, endpoint_descriptor);
  publish_static_graph(&marker_pub, vis_graph, graph_vertex_descriptor,
                       graph_edge_descriptor);

  visualization::sleep_ms(3000);

  // Run A* animation.

  PointDescriptor fringe_descriptor(color::ORANGE, 0.2, 0.04);
  PointDescriptor current_vertex_descriptor(color::GREEN, 0.2, 0.04);
  PointDescriptor relaxed_vertices_descriptor(color::BLUE, 0.2, 0);
  LineDescriptor relaxed_edges_descriptor(color::BLUE, 0.03, 0.04);
  LineDescriptor found_path_descriptor(color::YELLOW, 0.1, 0.06);

  double update_rate_ms = 500;
  graph_search::AnimationManager animation_manager(
      &marker_pub, update_rate_ms, fringe_descriptor, current_vertex_descriptor,
      relaxed_vertices_descriptor, relaxed_edges_descriptor);

  double heuristic_weight = 1;
  const graphlib::Vertex2d* start_vertex = dynamic_cast<decltype(start_vertex)>(
      vis_graph.GetVertexPtr(graphlib::Vertex2d(start.x, start.y)));
  const graphlib::Vertex2d* goal_vertex = dynamic_cast<decltype(goal_vertex)>(
      vis_graph.GetVertexPtr(graphlib::Vertex2d(goal.x, goal.y)));
  auto path = graph_search::a_star(&vis_graph, start_vertex, goal_vertex,
                                   heuristic_weight, &animation_manager);

  publish_static_path(&marker_pub, path, found_path_descriptor);

  ros::spin();
  return 0;
}
