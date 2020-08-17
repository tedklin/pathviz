# Visibility graphs

There are well-known efficient algorithms for finding shortest paths in a well-defined graph (see Dijkstra's algorithm or its heuristic variation A*). We know that these algorithms work well for geometric graphs whose vertices specify points in space and whose edges specify (non-negative) distances. The question for path-planners then becomes: based on what we know about the environment and our start/goal points, how do we define the graph on which these shortest-paths algorithms could run?

The simplest environment is probably a 2D map with strictly polygonal obstacles. A natural way of abstracting this map into a Euclidean graph is to:
- define the graph's vertices as all corners of all obstacle polygons + the start/goal points for your robot.
- define the graph's edges as all linear connections between two vertices that don't go through an obstacle.

This is formally known as a *visibility graph*, as it relies on finding which vertices are "visible" from each other (as opposed to being "blocked" by an obstacle) to determine valid edges.


## Lee's rotational sweep algorithm

The simplest way to construct a visibility graph would be to iterate through every possible pair of vertices and check if the line connecting them intersects an obstacle polygon. To check if a single pair of vertices intersects any obstacle polygon, this naive algorithm would have to iterate through all edges defined by all polygons in the map.

We can make visibility graph construction more efficient by cutting down the number of edges we need to check to determine if a pair of vertices intersects an obstacle polygon.
