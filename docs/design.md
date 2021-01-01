# Design document

This document contains notes and reflections on the current design of *pathviz*. It isn't intended to be used as formal documentation.


## Directory-by-directory breakdown

### *geometry*

The core of *pathviz* consists of simple objects and functions for basic geometry. The code is mostly self-explanatory.

The examples I've created so far haven't run into any critical [floating point errors](https://docs.oracle.com/cd/E19957-01/806-3568/ncg_goldberg.html), but that is a potential problem I have yet to solve. Something to look into might be defining spatial location as relative distances between neighboring points, instead of having everything on a universal coordinate system with one origin (since floating point dictates that precision gets lost as you move away from that origin). Another thing to consider may be scaling down input to use some lower range of floating point numbers for numerical calculations under the hood (not entirely sure how effective this would be).


### *visualization*

Visualization is done using rviz [visualization markers](http://wiki.ros.org/rviz/DisplayTypes/Marker). To ensure performance and scalability, *pathviz* uses the List variant of rviz marker types instead of using the individual shape types. However, the List marker types do not provide much support for stateful operation. To streamline the work of animating these markers, the interface of *pathviz* wraps the necessary rviz functionality in the form of *Descriptors* and *Managers*.

*Descriptors* are simple structs that hold appearance-related information for different marker types. *Managers* are classes that define safe CRUD operations to support stateful markers. Together, they abstract away the boilerplate needed to define/use rviz markers and let the user focus on the important things like color, size, layering, and lifetime.

*pathviz* also contains helper functions to visualize common static structures, such as the underlying graph that an animated algorithm might run on top of.


### *graph_search, visibility_map*

Currently, *pathviz* only contains these two animations/implementations, but creating new ones would all follow the same simple pattern.

An *AnimationManager* is a collection of *Descriptors* and *Managers* needed to animate a specific algorithm implementation. An algorithm implementation takes in an *AnimationManager* instance as a raw pointer, with the purpose of letting the user choose whether to run an algorithm with visualization enabled or disabled based on the nullity of the pointer.

With visualization disabled, the algorithm implementation will completely ignore all visualization instructions and run as efficiently as the underlying algorithm logic dictates. However, this flexibility might come at the expense of code readability, as algorithm implementations containing both underlying algorithm logic and visualization instructions can get pretty long.


### *nodes*

[Here](https://github.com/tedklin/pathviz/tree/master/src/visualization/nodes) are examples of nodes (main methods) using *pathviz*. Running these are what create the [visualizations seen in the top-level README of this repo](https://github.com/tedklin/pathviz#visualization-collection).


## Overall reflections

*pathviz* attempts to be a catch-all solution that can cover both intuitive visualization and efficient implementation for path-planning algorithms. While there is value in trying to do both visualization and implementation within the same piece of code (DRY, consistency, visual debugging), the decrease in code readability and increase in complexity seem to be not worth it. For a production environment, it probably makes more sense to separate implementation and visualization completely, with different stacks suitable for each one's needs.
