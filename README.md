# Search-Based Motion Planning Library

## Overview

A set of packages implementing generic heuristic search algorithms for robotic motion planning. Includes the following packages:

* smpl - Provides a library of discrete graph representations of robot configuration spaces, heuristics defined over those representations, and an interface to cleanly integrate them into the sbpl planning framework. Also provides library interfaces to extend the framework to use arbitrary robot representations and collision checking.
* smpl_test - Provides self-contained examples to illustrate usage of the interfaces provided in the smpl package.
* sbpl_collision_checking - Provides a library for collision checking of robot states against themselves and the environment using an approximate model implemented as a hierarchy of bounding spheres. Implements the collision checking interface defined in the smpl package.
* sbpl_kdl_robot_model - Provides a library for integration of robot models, specified via URDF, with kinematics implemented using the KDL library.
* sbpl_pr2_robot_model - Provides a library for integration of the PR2 and UBR1 robot models, specified via URDF, with kinematics implemented using custom analytical IK solvers.

## Before you begin

Install ROS Indigo by following the instructions at www.ros.org and follow the
tutorials to create a catkin workspace to build smpl within.

## Installation

### Install SBPL from Source

SMPL requires the latest SBPL to be installed. Because catkin prefers to find packages in your current and parent workspaces, you may need to remove any binary packages that provide SBPL, e.g. ros-_distro_-sbpl, to prevent catkin from giving it higher priority.

	git clone https://github.com/sbpl/sbpl
	cd sbpl && mkdir build && cd build && cmake .. && make && sudo make install

### Clone sbpl_manipulation and its source dependencies

	git clone https://github.com/aurone/sbpl_manipulation
	git clone https://github.com/aurone/leatherman

### Install additional dependencies via rosdep

	rosdep install -i -y smpl
	rosdep install -i -y smpl_test
	rosdep install -i -y sbpl_collision_checking
	rosdep install -i -y sbpl_collision_checking_test
	rosdep install -i -y sbpl_kdl_robot_model
	rosdep install -i -y sbpl_pr2_robot_model

### (Re)build your catkin workspace

	cd _catkin_ws_
	catkin_make [-j#]

## Running

Coming "soon"
