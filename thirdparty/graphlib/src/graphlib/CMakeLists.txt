list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/graphlib/graph.cpp")

list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/graphlib/geometry/graph_2d.cpp")

list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/graphlib/algo/bfs.cpp")
list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/graphlib/algo/dfs.cpp")
list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/graphlib/algo/mst.cpp")
list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/graphlib/algo/weighted_paths.cpp")
# ^^^ APPEND NEW SOURCE FILES TO THE SOURCE_LIST

# Make an automatic library - will be static or dynamic based on user setting
add_library(graphlib ${SOURCE_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(graphlib PUBLIC ${PROJECT_SOURCE_DIR}/src)

# All users of this library will need at least C++14
target_compile_features(graphlib PUBLIC cxx_std_14)
set_target_properties(graphlib PROPERTIES CXX_EXTENSIONS OFF)

# Compiler flags
target_compile_options(graphlib PRIVATE "-fPIC" "-Wall")
