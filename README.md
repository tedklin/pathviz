# pathviz

[![Build Status](https://travis-ci.com/tedklin/pathviz.svg?branch=master)](https://travis-ci.com/tedklin/pathviz)

***pathviz*** aims to serve as a framework for concrete implementations and intuitive visualizations of algorithms that operate in a Cartesian coordinate system.

## About

The algorithms that have been / will be implemented here have been covered thoroughly in literature, as well as in various courses whose material is freely available online. In particular, ***Principles of Robot Motion (Choset, et al)*** and ***Computational Geometry (de Berg, et al)*** have been my main reference texts so far.

The primary goal of *pathviz* is providing a visual supplement to well-known algorithms. Each algorithm shown in the ["Visualization collection"](https://github.com/tedklin/pathviz#visualization-collection) below will at most come with a brief snippet of pseudocode to remind the reader of what it does. Complete information can be found in the aforementioned books or elsewhere online.

A secondary goal of *pathviz* is to provide usable, efficient implementations for each algorithm. The project has been designed such that the caller of an algorithm implementation can control whether visualization is disabled (default) or enabled. With visualization disabled, the implementation will completely ignore all visualization instructions and run as efficiently as the underlying algorithm logic dictates.

Additional notes and reflections on the design of this project can be found [here](https://github.com/tedklin/pathviz/tree/master/docs/design.md).

## Dependencies

- ROS Melodic
- rviz
- [graphlib](https://github.com/tedklin/back-to-basics/tree/master/02_pl-usage/cpp/graphlib) - a separate toy (i.e. for practice and far from production-ready) library I wrote for general graphs.

## Usage

To run existing visualizations on your own device, simply download and build *pathviz* in a ROS catkin workspace and launch the desired roslaunch file located [here](https://github.com/tedklin/pathviz/tree/master/launch)!

Visualization colors and animation speeds can be tuned according to preference. I'm considering setting up rosparam files for easier adjustment of visual parameters, but I haven't started implementing that yet.


## Visualization collection:

## Lee's rotational plane sweep algorithm

*for visibility graph construction*

### Pseudocode

(from *Principles of Robot Motion*)

![Visibility graph pseudocode](./media/visibility_graph_pseudocode.png)

### Animation

*Side notes:*
- *The implementation here sweeps counterclockwise starting at the negative x-axis w.r.t. the current source vertex.*
- *This animation clocks in at over 10 minutes in full, so you might want to refresh the page to watch the beginning. If you don't have a particularly intense desire to watch the animation all the way through, scroll down further to see the complete visiblity graph.*

| Color | Component |
| --- | --- |
| blue | static obstacle polygons |
| yellow | static start and goal points |
| purple | current source vertex |
| orange | rotational sweep line visiting all other vertices |
| green | successful line of visibility |
| black | unsuccessful line of visibility|
| red | active list of edges |

![Visibility graph animated](./media/visibility_graph_animated.gif)

### Result

![Visibility graph static](./media/visibility_graph_static.png)


## Dijkstra's algorithm and its heuristic variation A*

*for shortest paths in a positive-edge-weighted graph*

### Pseudocode

(from *Principles of Robot Motion*)

![A* pseudocode](./media/a_star_pseudocode.png)

### Animation

| Color | Component |
| --- | --- |
| black | static Euclidean graph |
| yellow points | static start and goal points |
| green | current vertex being expanded / relaxed |
| purple points | relaxed vertices |
| purple lines | *currently-known* best path from start point to a relaxed vertex |
| orange | priority queue of vertices to relax next ("fringe") |
| yellow line | final found path from start to goal points |

#### Dijkstra's algorithm

![Dijkstra's animated](./media/dijkstra_animated.gif)

#### A* search algorithm

![A* animated](./media/a_star_animated.gif)

## TBD
