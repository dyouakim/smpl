////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Benjamin Cohen

// standard includes
#include <string>
#include <vector>

// system includes
#include <geometry_msgs/Transform.h>
#include <ros/ros.h>
#include <moveit_msgs/PlanningScene.h>
#include <smpl/occupancy_grid.h>
#include <sbpl_collision_checking/collision_space.h>
#include <smpl/ros/propagation_distance_field.h>
#include <visualization_msgs/MarkerArray.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/visualizer_ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sbpl_collision_space_test");
    ros::NodeHandle nh;

    sbpl::VisualizerROS visualizer;
    sbpl::visual::set_visualizer(&visualizer);

    ros::NodeHandle ph("~");

    std::string group_name;
    std::string world_frame;
    double dims[3];
    double origin[3];
    std::vector<std::string> joint_names(7);

    ph.param<std::string>("group_name", group_name, "");
    ph.param<std::string>("world_frame", world_frame, "");
    ph.param("dims/x", dims[0], 2.0);
    ph.param("dims/y", dims[1], 2.0);
    ph.param("dims/z", dims[2], 2.0);
    ph.param("origin/x", origin[0], -0.75);
    ph.param("origin/y", origin[1], 1.25);
    ph.param("origin/z", origin[2], -0.3);
    ph.param<std::string>("joint_0", joint_names[0], "");
    ph.param<std::string>("joint_1", joint_names[1], "");
    ph.param<std::string>("joint_2", joint_names[2], "");
    ph.param<std::string>("joint_3", joint_names[3], "");
    ph.param<std::string>("joint_4", joint_names[4], "");
    ph.param<std::string>("joint_5", joint_names[5], "");
    ph.param<std::string>("joint_6", joint_names[6], "");

    auto rit = std::remove_if(
            begin(joint_names), end(joint_names),
            [](const std::string& name) { return name.empty(); });
    joint_names.erase(rit, end(joint_names));

    if (joint_names.empty()) {
        ROS_ERROR("No planning joints found on param server.");
        return 0;
    }

    ROS_INFO("Retrieved %zu planning joints from param server.", joint_names.size());

    const double res = 0.02;
    const double max_distance = 0.4;
    auto df = std::make_shared<sbpl::PropagationDistanceField>(
            origin[0], origin[1], origin[2],
            dims[0], dims[1], dims[2],
            res,
            max_distance);

    sbpl::OccupancyGrid grid(df);
    grid.setReferenceFrame(world_frame);

    std::string urdf_string;
    if (!nh.getParam("robot_description", urdf_string)) {
        ROS_ERROR("Failed to retrieve 'robot_description' from the param server");
        return 1;
    }

    sbpl::collision::CollisionModelConfig cspace_config;
    if (!sbpl::collision::CollisionModelConfig::Load(ros::NodeHandle(), cspace_config)) {
        ROS_ERROR("Failed to load Collision Model Config");
        return 1;
    }

    sbpl::collision::CollisionSpace cspace;
    if (!cspace.init(&grid, urdf_string, cspace_config, group_name, joint_names)) {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }

    ROS_INFO("Initialized the collision space.");

    // add robot's pose in map
    moveit_msgs::PlanningScene scene;
    scene.world.octomap.header.frame_id = world_frame;
    scene.robot_state.multi_dof_joint_state.header.frame_id = "base_link";
    scene.robot_state.multi_dof_joint_state.joint_names.push_back("torso_lift_link");

    geometry_msgs::Transform t;
    t.translation.x = -0.06;
    t.translation.y = 0.0;
    t.translation.z = 0.34;
    t.rotation.w = 1.0;
    t.rotation.x = t.rotation.y = t.rotation.z = 0.0;
    scene.robot_state.multi_dof_joint_state.transforms.push_back(t);

    scene.robot_state.joint_state.name.push_back("right_gripper_finger_joint");
    scene.robot_state.joint_state.name.push_back("left_gripper_finger_joint");
    scene.robot_state.joint_state.position.push_back(0.08);
    scene.robot_state.joint_state.position.push_back(0.08);
    cspace.setPlanningScene(scene);

    std::vector<double> angles(7, 0.0);
    angles[0] = -0.7;
    angles[1] = 0.3;
    angles[2] = 0.0;
    angles[3] = 0.5;
    angles[4] = 0.6;
    angles[5] = 0.8;
    angles[6] = 0.4;

    ros::spinOnce();
    SV_SHOW_INFO(cspace.getBoundingBoxVisualization());
    SV_SHOW_INFO(cspace.getOccupiedVoxelsVisualization());
    SV_SHOW_INFO(cspace.getDistanceFieldVisualization());
    SV_SHOW_INFO(cspace.getCollisionModelVisualization(angles));

    ros::spinOnce();
    sleep(1);

    ROS_INFO("Done");
    return 0;
}
