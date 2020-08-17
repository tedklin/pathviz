# pathviz

This project aims to serve as a growing collection for concrete implementations and intuitive visualizations of classic algorithms related to path planning.

## About

The algorithms implemented here have been covered thoroughly in literature, as well as in various motion / path planning courses whose material is freely available online. In particular, ***Principles of Robot Motion (Choset, et al)*** and ***Computational Geometry (de Berg, et al)*** are my current self-study reference texts.

*pathviz* is not intended to be a standalone reference; instead it focuses on providing a visual supplement to well-known algorithms. At most, each algorithm shown in the "Visualization collection" below will come with a brief snippet of pseudocode to remind the reader of what it does. Complete information can be found in the aforementioned books or elsewhere online.

A secondary goal of *pathviz* is to provide usable implementations for each algorithm. The project has been designed such that it's easy to add visualization to an existing algorithm function implementation in a way that doesn't impact the underlying runtime speed too much. This flexibility might come at the expense of readability, as functions containing both underlying algorithm logic and visualization instructions can get pretty long.

## Dependencies

- ROS Melodic
- rviz
- [graphlib](https://github.com/tedklin/back-to-basics/tree/master/algorithms/graphlib) (A separate library I wrote for more general fundamental graph theory)

## Usage

To run existing visualizations, simply build *pathviz* in a ROS catkin workspace and launch the desired roslaunch file located [here](https://github.com/tedklin/pathviz/tree/master/launch)!

If you want to build your own visualizations, *pathviz* includes a small interface for animating [rviz Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker).

Animation speeds can be tuned according to preference. If it wasn't obvious, it should be noted that the animations below don't reflect the actual wall clock running speed of the algorithm. I've considered integrating rosparam files for easier and faster iteration, but I haven't implemented that yet.


## Visualization collection:

## Lee's rotational plane sweep algorithm

*for visibility graph construction*

### Pseudocode

(from *Principles of Robot Motion*)

![Visibility graph pseudocode](./media/visibility_graph_pseudocode.png)

### Animation

*Side notes:*
- *The implementation here sweeps counterclockwise starting at the negative x-axis w.r.t. the current source vertex.*
- *This animation clocks in at over 10 minutes in full, so you might want to refresh the page to watch the beginning. Also, if you don't have a particularly intense desire to see the animation all the way through, scroll down to see the complete visiblity graph.*

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
| yellow | static start and goal points |
| green | current vertex being expanded / relaxed |
| blue points | relaxed vertices |
| blue lines | currently-known best path from start point to a relaxed vertex (relaxed edges) |
| orange | priority queue of vertices to relax next (fringe) |

#### Dijkstra's algorithm

![Dijkstra's animated](./media/dijkstra_animated.gif)

#### A* search algorithm

![A* animated](./media/a_star_animated.gif)

## TBD
