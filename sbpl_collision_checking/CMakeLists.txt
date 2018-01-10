cmake_minimum_required(VERSION 2.8)
project(sbpl_collision_checking)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Boost REQUIRED)

find_package(catkin
    REQUIRED
    COMPONENTS
        cmake_modules
        eigen_conversions
        geometric_shapes
        geometry_msgs
        leatherman
        message_filters
        moveit_core
        moveit_msgs
        octomap_msgs
        roscpp
        smpl
        smpl_ros
        sensor_msgs
        shape_msgs
        tf_conversions
        urdf
        visualization_msgs)

find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        sbpl_collision_checking
    CATKIN_DEPENDS
        eigen_conversions
        geometric_shapes
        geometry_msgs
        leatherman
        message_filters
        moveit_core
        moveit_msgs
        octomap_msgs
        roscpp
        smpl
        smpl_ros
        sensor_msgs
        shape_msgs
        tf_conversions
        urdf
        visualization_msgs
    DEPENDS
        Boost
        Eigen
        OCTOMAP)

add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO)

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${OCTOMAP_INCLUDE_DIRS})
include_directories(include)

add_library(
    sbpl_collision_checking
    src/attached_bodies_collision_model.cpp
    src/attached_bodies_collision_state.cpp
    src/base_collision_models.cpp
    src/base_collision_states.cpp
    src/collision_model_config.cpp
    src/collision_operations.cpp
    src/collision_space.cpp
    src/robot_collision_model.cpp
    src/robot_motion_collision_model.cpp
    src/robot_collision_state.cpp
    src/self_collision_model.cpp
    src/types.cpp
    src/voxel_operations.cpp
    src/world_collision_detector.cpp
    src/world_collision_model.cpp)
target_link_libraries(sbpl_collision_checking ${catkin_LIBRARIES})

install(
    TARGETS sbpl_collision_checking
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
