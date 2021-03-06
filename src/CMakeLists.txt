#####################
## pathviz library ##
#####################

list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/geometry/geometry_2d.cpp")
list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/visualization/interface.cpp")
list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/visualization/static_helpers.cpp")

list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/visibility_map/visibility_graph.cpp")

list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/graph_search/a_star.cpp")
# ^^^ APPEND NEW LIBRARY SOURCE FILES TO THE SOURCE_LIST

# Make an automatic library - will be static or dynamic based on user setting
add_library(pathviz ${SOURCE_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(pathviz PUBLIC ${PROJECT_SOURCE_DIR}/src)

# Dependencies
target_link_libraries(pathviz PUBLIC graphlib ${catkin_LIBRARIES})

# All users of this library will need at least C++14
target_compile_features(pathviz PUBLIC cxx_std_14)


#######################
## pathviz ros nodes ##
#######################

# macro for adding ros nodes
macro(pathviz_add_node EXAMPLENAME)
  # create an exectuable in which the tests will be stored
  add_executable(${EXAMPLENAME} ${ARGN})
  target_compile_features(${EXAMPLENAME} PRIVATE cxx_std_17)

  # link pathviz (and its dependencies)
  target_link_libraries(${EXAMPLENAME} PRIVATE pathviz ${catkin_LIBRARIES})
endmacro()

pathviz_add_node(visibility_graph_static visualization/nodes/visibility_graph_static.cpp)
pathviz_add_node(visibility_graph_animated visualization/nodes/visibility_graph_animated.cpp)
pathviz_add_node(dijkstra_animated visualization/nodes/dijkstra_animated.cpp)
pathviz_add_node(a_star_animated visualization/nodes/a_star_animated.cpp)
# ^^^ ADD NEW NODES HERE
