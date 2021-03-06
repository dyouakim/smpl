cmake_minimum_required(VERSION 2.8.12)
project(smpl)

set(SMPL_ROS_BUILD ON CACHE BOOL "Build smpl as a catkin package")
set(SMPL_CONSOLE_ROS ${SMPL_ROS_BUILD} CACHE BOOL "Replace smpl log macros with rosconsole macros")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED COMPONENTS filesystem program_options)

find_package(Eigen3 REQUIRED)

find_package(sbpl REQUIRED)
set(sbpl_INCLUDE_DIRS ${SBPL_INCLUDE_DIRS})
set(sbpl_LIBRARIES ${SBPL_LIBRARIES})

if(SMPL_ROS_BUILD)
    find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs 
  tf
  tf_conversions
)

    catkin_package(
        INCLUDE_DIRS
            include ${EIGEN3_INCLUDE_DIR}
        LIBRARIES
            smpl
        CATKIN_DEPENDS
            roscpp
            visualization_msgs
        DEPENDS
            Boost
            sbpl)
endif()

set(SMPL_CLOCK_API SMPL_CLOCK_CHRONO_HIGH_RESOLUTION)

if(SMPL_ROS_BUILD)
    set(SMPL_SV_VISUALIZATION_MSGS ON)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(SMPL_ROS_BUILD AND SMPL_CONSOLE_ROS)
    add_definitions(-DSMPL_CONSOLE_ROS)
else()
    # configure printf backend
    add_definitions(-DSMPL_CONSOLE_COLORIZE_OUTPUT=1)
    add_definitions(-DSMPL_CONSOLE_UNBUFFERED=1)
    add_definitions(-DSMPL_LOG_FILE_AND_LINE=1)
endif()

add_definitions(-DSV_PACKAGE_NAME="smpl")

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
if(SMPL_ROS_BUILD)
    include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
endif()
include_directories(SYSTEM ${sbpl_INCLUDE_DIRS})
include_directories(include)

configure_file(
    ${PROJECT_SOURCE_DIR}/include/smpl/config.h.in
    ${PROJECT_SOURCE_DIR}/include/smpl/config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include/smpl)

add_library(
    smpl
src/distance_map/chessboard_distance_map.cpp
    src/distance_map/distance_map_common.cpp
    src/distance_map/edge_euclid_distance_map.cpp
    src/distance_map/euclid_distance_map.cpp
    src/distance_map/sparse_distance_map.cpp  
    src/occupancy_grid.cpp
    src/debug/colors.cpp
    src/debug/marker_utils.cpp
    src/debug/visualize.cpp
    src/bfs3d.cpp
    src/csv_parser.cpp
    src/collision_checker.cpp
    src/console/ansi.cpp
    src/console/console.cpp
    src/planning_params.cpp
    src/post_processing.cpp
    src/robot_model.cpp
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
    src/heuristic/attractor_heuristic.cpp
    src/heuristic/bfs_heuristic.cpp
    src/heuristic/multi_bfs_heuristic.cpp
    src/heuristic/egraph_bfs_heuristic.cpp
    src/heuristic/generic_egraph_heuristic.cpp
    src/heuristic/euclid_dist_heuristic.cpp
    src/heuristic/robot_heuristic.cpp
    src/heuristic/joint_dist_heuristic.cpp
    src/heuristic/multi_frame_bfs_heuristic.cpp
    src/search/arastar.cpp
    src/search/experience_graph_planner.cpp
    src/search/adaptive_planner.cpp
    src/search/trastar.cpp
    src/search/mhtrastar.cpp
    src/search/marastar.cpp
    src/search/focal_mhastar.cpp
    src/search/meta_mhastar_dts.cpp
    src/search/mhastarpp.cpp
    src/search/unconstrained_mhastar.cpp
    src/search/arastar.cpp
    src/search/experience_graph_planner.cpp
    src/search/adaptive_planner.cpp
    src/search/lazy_arastar.cpp
    src/search/lazy_mhastar.cpp
    src/search/awastar.cpp)
target_link_libraries(smpl ${Boost_LIBRARIES} ${sbpl_LIBRARIES})
if(SMPL_ROS_BUILD)
    target_link_libraries(smpl ${catkin_LIBRARIES})
endif()

if(SMPL_ROS_BUILD)
    set(SMPL_INCLUDE_DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    set(SMPL_ARCHIVE_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    set(SMPL_LIBRARY_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    set(SMPL_RUNTIME_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else()
    set(SMPL_INCLUDE_DESTINATION include)
    set(SMPL_ARCHIVE_DESTINATION lib)
    set(SMPL_LIBRARY_DESTINATION lib)
    set(SMPL_RUNTIME_DESTINATION bin)
endif()

install(
    DIRECTORY include
    DESTINATION ${SMPL_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")

install(
    TARGETS smpl
    ARCHIVE DESTINATION ${SMPL_ARCHIVE_DESTINATION}
    LIBRARY DESTINATION ${SMPL_LIBRARY_DESTINATION}
    RUNTIME DESTINATION ${SMPL_RUNTIME_DESTINATION})
