cmake_minimum_required(VERSION 2.8.12)
project(smpl)

set(CMAKE_BUILD_TYPE Release)
add_compile_options("-std=c++11")

find_package(Boost REQUIRED)

find_package(catkin
    REQUIRED
    COMPONENTS
        cmake_modules
        leatherman
        moveit_core
        moveit_msgs
        roscpp
        trajectory_msgs
        visualization_msgs)

find_package(Eigen REQUIRED)
find_package(sbpl REQUIRED)

set(sbpl_INCLUDE_DIRS ${SBPL_INCLUDE_DIRS})
set(sbpl_LIBRARIES ${SBPL_LIBRARIES})
catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        smpl
    CATKIN_DEPENDS
        leatherman
        moveit_core
        moveit_msgs
        roscpp
        trajectory_msgs
        visualization_msgs
    DEPENDS
        Boost
        Eigen
        sbpl)

add_definitions(-DSBPL_VISUALIZE_MIN_SEVERITY=SBPL_VISUALIZE_SEVERITY_DEBUG)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${sbpl_INCLUDE_DIRS})
include_directories(include)

add_library(
    smpl
    src/bfs3d.cpp
    src/csv_parser.cpp
    src/collision_checker.cpp
    src/occupancy_grid.cpp
    src/planning_params.cpp
    src/post_processing.cpp
    src/robot_model.cpp
    src/debug/visualize.cpp
    src/debug/visualizer_ros.cpp
    src/distance_map/chessboard_distance_map.cpp
    src/distance_map/distance_map_common.cpp
    src/distance_map/edge_euclid_distance_map.cpp
    src/distance_map/euclid_distance_map.cpp
    src/distance_map/sparse_distance_map.cpp
    src/geometry/bounding_spheres.cpp
    src/geometry/mesh_utils.cpp
    src/geometry/voxelize.cpp
    src/graph/action_space.cpp
    src/graph/adaptive_workspace_lattice.cpp
    src/graph/experience_graph.cpp
    src/graph/manip_lattice.cpp
    src/graph/manip_lattice_egraph.cpp
    src/graph/manip_lattice_action_space.cpp
    src/graph/robot_planning_space.cpp
    src/graph/workspace_lattice.cpp
    src/graph/workspace_lattice_base.cpp
    src/heuristic/bfs_heuristic.cpp
    src/heuristic/egraph_bfs_heuristic.cpp
    src/heuristic/generic_egraph_heuristic.cpp
    src/heuristic/euclid_dist_heuristic.cpp
    src/heuristic/robot_heuristic.cpp
    src/heuristic/joint_dist_heuristic.cpp
    src/heuristic/multi_frame_bfs_heuristic.cpp
    src/ros/adaptive_workspace_lattice_allocator.cpp
    src/ros/araplanner_allocator.cpp
    src/ros/traplanner_allocator.cpp
    src/ros/bfs_heuristic_allocator.cpp
    src/ros/dijkstra_egraph_3d_heuristic_allocator.cpp
    src/ros/experience_graph_planner_allocator.cpp
    src/ros/euclid_dist_heuristic_allocator.cpp
    src/ros/joint_dist_heuristic_allocator.cpp
    src/ros/joint_dist_egraph_heuristic_allocator.cpp
    src/ros/laraplanner_allocator.cpp
    src/ros/manip_lattice_allocator.cpp
    src/ros/manip_lattice_egraph_allocator.cpp
    src/ros/mhaplanner_allocator.cpp
    src/ros/multi_frame_bfs_heuristic_allocator.cpp
    src/ros/adaptive_planner_allocator.cpp
    src/ros/planner_interface.cpp
    src/ros/propagation_distance_field.cpp
    src/ros/workspace_lattice_allocator.cpp
    src/search/arastar.cpp
    src/search/experience_graph_planner.cpp
    src/search/adaptive_planner.cpp
    src/search/traplanner.cpp)

target_link_libraries(smpl ${catkin_LIBRARIES} ${sbpl_LIBRARIES})

install(
    TARGETS smpl
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
