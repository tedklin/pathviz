list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/geometry/geometry_2d.cpp")
list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/visibility_map/visibility_graph.cpp")
# ^^^ APPEND NEW SOURCE FILES TO THE SOURCE_LIST

# Make an automatic library - will be static or dynamic based on user setting
add_library(pathviz ${SOURCE_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(pathviz PUBLIC ${PROJECT_SOURCE_DIR}/src)

# Dependencies
target_link_libraries(pathviz PUBLIC graphlib ${catkin_LIBRARIES})

# All users of this library will need at least C++11
target_compile_features(pathviz PUBLIC cxx_std_11)
