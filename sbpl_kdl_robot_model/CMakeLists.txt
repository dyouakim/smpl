cmake_minimum_required(VERSION 2.8)
project(sbpl_kdl_robot_model)

set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin
    REQUIRED
    COMPONENTS
        kdl_parser
        leatherman
        roscpp
        smpl
        urdf)

find_package(orocos_kdl REQUIRED)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        sbpl_kdl_robot_model
    CATKIN_DEPENDS
        leatherman
        roscpp
        smpl
        urdf
    DEPENDS
        orocos_kdl)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS})
include_directories(include)

add_library(sbpl_kdl_robot_model src/kdl_robot_model.cpp)

target_link_libraries(
    sbpl_kdl_robot_model
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES})

add_executable(test_kdl src/test_kdl_robot_model.cpp)
target_link_libraries(test_kdl ${catkin_LIBRARIES} sbpl_kdl_robot_model)

install(
    TARGETS sbpl_kdl_robot_model test_kdl
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
