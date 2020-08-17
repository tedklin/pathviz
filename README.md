# pathviz

The goal of ***pathviz*** is to serve as a growing hub for concrete implementations and intuitive visualizations of classic algorithms related to path planning. 

The algorithms implemented here have been covered thoroughly in literature, as well as in various motion / path planning courses whose material is freely available online. In particular, ***Principles of Robot Motion (Choset, et al)*** and ***Computational Geometry (de Berg, et al)*** served as this project's primary reference books.

This project is not intended to be a standalone reference; instead it focuses on providing a visual supplement to well-known algorithms. At most, each algorithm will come with a brief snippet of pseudocode to remind readers of what it does. Complete information can be found in the aforementioned books.

## Lee's rotational plane sweep algorithm

for visibility graph construction

### Pseudocode

(from *Principles of Robot Motion*)

![Visibility graph pseudocode](./media/visibility_graph_pseudocode.png)

### Animation

| Color | Component |
| --- | --- |
| blue | static obstacle polygons |
| yellow | static start and goal points |
| purple | current source vertex |
| orange | rotational sweep line visiting all other vertices |
| green | successful line of visibility |
| black | unsuccessful line of visibility|
| red | active list of edges |

*note: this animation clocks in at over 10 minutes, so unless you have some intense desire to see this all the way through, scroll down to see the complete visiblity graph.*

![Visibility graph animated](./media/visibility_graph_animated.gif)

### Result

![Visibility graph static](./media/visibility_graph_static.png)


## Dijkstra's algorithm and its heuristic variation A*

for shortest paths in a positive edge-weighted graph

### Pseudocode

(from *Principles of Robot Motion*)

![A* pseudocode](./media/a_star_pseudocode.png)

### Animation

| Color | Component |
| --- | --- |
| black | static Euclidean graph |
| yellow | static start and goal points |
| blue points | relaxed vertices |
| blue lines | currently-known best path from start point to a relaxed vertex (relaxed edges) |
| green | current vertex being expanded / relaxed |
| orange | priority queue of vertices to relax next (fringe) |

#### Dijkstra's algorithm

![Dijkstra's animated](./media/dijkstra_animated.gif)

#### A* search algorithm

![A* animated](./media/a_star_animated.gif)
