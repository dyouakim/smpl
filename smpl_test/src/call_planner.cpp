////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2012, Benjamin Cohen
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
#include <stdlib.h>
#include <string>
#include <thread>
#include <vector>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <smpl/ros/planner_interface.h>
#include <smpl/distance_map/edge_euclid_distance_map.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/ros/propagation_distance_field.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <sbpl_pr2_robot_model/pr2_kdl_robot_model.h>
#include <sbpl_pr2_robot_model/ubr1_kdl_robot_model.h>
#include <visualization_msgs/MarkerArray.h>
#include <smpl/angles.h>
#include <smpl/debug/visualizer_ros.h>

namespace smpl = sbpl::motion;

void FillGoalConstraint(
    const std::vector<double>& pose,
    std::string frame_id,
    moveit_msgs::Constraints& goals)
{
    if (pose.size() < 6) {
        return;
    }

    goals.position_constraints.resize(1);
    goals.orientation_constraints.resize(1);
    goals.position_constraints[0].header.frame_id = frame_id;

    goals.position_constraints[0].constraint_region.primitives.resize(1);
    goals.position_constraints[0].constraint_region.primitive_poses.resize(1);
    goals.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.x = pose[0];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.y = pose[1];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.z = pose[2];

//    goals.position_constraints[0].position.x = pose[0];
//    goals.position_constraints[0].position.y = pose[1];
//    goals.position_constraints[0].position.z = pose[2];

    Eigen::Quaterniond q;
    sbpl::angles::from_euler_zyx(pose[5], pose[4], pose[3], q);
    tf::quaternionEigenToMsg(q, goals.orientation_constraints[0].orientation);

    geometry_msgs::Pose p;
    p.position = goals.position_constraints[0].constraint_region.primitive_poses[0].position;
    p.orientation = goals.orientation_constraints[0].orientation;
    leatherman::printPoseMsg(p, "Goal");

    /// set tolerances
    goals.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3, 0.015);
    goals.orientation_constraints[0].absolute_x_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_y_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_z_axis_tolerance = 0.05;

    ROS_INFO("Done packing the goal constraints message.");
}

auto GetCollisionCube(
    const geometry_msgs::Pose& pose,
    std::vector<double>& dims,
    const std::string& frame_id,
    const std::string& id)
    -> moveit_msgs::CollisionObject
{
    moveit_msgs::CollisionObject object;
    object.id = id;
    object.operation = moveit_msgs::CollisionObject::ADD;
    object.header.frame_id = frame_id;
    object.header.stamp = ros::Time::now();

    shape_msgs::SolidPrimitive box_object;
    box_object.type = shape_msgs::SolidPrimitive::BOX;
    box_object.dimensions.resize(3);
    box_object.dimensions[0] = dims[0];
    box_object.dimensions[1] = dims[1];
    box_object.dimensions[2] = dims[2];

    object.primitives.push_back(box_object);
    object.primitive_poses.push_back(pose);
    return object;
}

auto GetCollisionCubes(
    std::vector<std::vector<double>>& objects,
    std::vector<std::string>& object_ids,
    const std::string& frame_id)
    -> std::vector<moveit_msgs::CollisionObject>
{
    std::vector<moveit_msgs::CollisionObject> objs;
    std::vector<double> dims(3,0);
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    if (object_ids.size() != objects.size()) {
        ROS_INFO("object id list is not same length as object list. exiting.");
        return objs;
    }

    for (size_t i = 0; i < objects.size(); i++) {
        pose.position.x = objects[i][0];
        pose.position.y = objects[i][1];
        pose.position.z = objects[i][2];
        dims[0] = objects[i][3];
        dims[1] = objects[i][4];
        dims[2] = objects[i][5];

        objs.push_back(GetCollisionCube(pose, dims, frame_id, object_ids.at(i)));
    }
    return objs;
}

auto GetCollisionObjects(
    const std::string& filename,
    const std::string& frame_id)
    -> std::vector<moveit_msgs::CollisionObject>
{
    char sTemp[1024];
    int num_obs = 0;
    std::vector<std::string> object_ids;
    std::vector<std::vector<double> > objects;
    std::vector<moveit_msgs::CollisionObject> objs;

    FILE* fCfg = fopen(filename.c_str(), "r");

    if (fCfg == NULL) {
        ROS_INFO("ERROR: unable to open objects file. Exiting.\n");
        return objs;
    }

    // get number of objects
    if (fscanf(fCfg,"%s",sTemp) < 1) {
        printf("Parsed string has length < 1.\n");
    }

    num_obs = atoi(sTemp);

    ROS_INFO("%i objects in file",num_obs);

    //get {x y z dimx dimy dimz} for each object
    objects.resize(num_obs);
    object_ids.clear();
    for (int i=0; i < num_obs; ++i) {
        if (fscanf(fCfg,"%s",sTemp) < 1) {
            printf("Parsed string has length < 1.\n");
        }
        object_ids.push_back(sTemp);

        objects[i].resize(6);
        for (int j=0; j < 6; ++j)
        {
            if (fscanf(fCfg,"%s",sTemp) < 1) {
                printf("Parsed string has length < 1.\n");
            }
            if (!feof(fCfg) && strlen(sTemp) != 0) {
                objects[i][j] = atof(sTemp);
            }
        }
    }

    return GetCollisionCubes(objects, object_ids, frame_id);
}

bool ReadInitialConfiguration(
    ros::NodeHandle& nh,
    moveit_msgs::RobotState& state)
{
    XmlRpc::XmlRpcValue xlist;

    // joint_state
    if (nh.hasParam("initial_configuration/joint_state")) {
        nh.getParam("initial_configuration/joint_state", xlist);

        if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("initial_configuration/joint_state is not an array.");
        }

        if (xlist.size() > 0) {
            std::cout << xlist << std::endl;
            for (int i = 0; i < xlist.size(); ++i) {
                state.joint_state.name.push_back(std::string(xlist[i]["name"]));

                if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                    state.joint_state.position.push_back(double(xlist[i]["position"]));
                }
                else {
                    ROS_DEBUG("Doubles in the yaml file have to contain decimal points. (Convert '0' to '0.0')");
                    if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        int pos = xlist[i]["position"];
                        state.joint_state.position.push_back(double(pos));
                    }
                }
            }
        }
    }
    else {
        ROS_WARN("initial_configuration/joint_state is not on the param server.");
    }

    // multi_dof_joint_state
    if (nh.hasParam("initial_configuration/multi_dof_joint_state")) {
        nh.getParam("initial_configuration/multi_dof_joint_state", xlist);

        if (xlist.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            if (xlist.size() != 0) {
                auto &multi_dof_joint_state(state.multi_dof_joint_state);
                multi_dof_joint_state.header.frame_id = std::string(xlist[0]["frame_id"]);
                multi_dof_joint_state.joint_names.resize(xlist.size());
                multi_dof_joint_state.transforms.resize(xlist.size());
                for (int i = 0; i < xlist.size(); ++i) {
                    multi_dof_joint_state.joint_names[i] = std::string(xlist[i]["child_frame_id"]);

                    Eigen::Quaterniond q;
                    sbpl::angles::from_euler_zyx(
                            (double)xlist[i]["yaw"], (double)xlist[i]["pitch"], (double)xlist[i]["roll"], q);

                    geometry_msgs::Quaternion orientation;
                    tf::quaternionEigenToMsg(q, orientation);

                    multi_dof_joint_state.transforms[i].translation.x = xlist[i]["x"];
                    multi_dof_joint_state.transforms[i].translation.y = xlist[i]["y"];
                    multi_dof_joint_state.transforms[i].translation.z = xlist[i]["z"];
                    multi_dof_joint_state.transforms[i].rotation.w = orientation.w;
                    multi_dof_joint_state.transforms[i].rotation.x = orientation.x;
                    multi_dof_joint_state.transforms[i].rotation.y = orientation.y;
                    multi_dof_joint_state.transforms[i].rotation.z = orientation.z;
                }
            } else {
                ROS_WARN("initial_configuration/multi_dof_joint_state array is empty");
            }
        } else {
            ROS_WARN("initial_configuration/multi_dof_joint_state is not an array.");
        }
    }

    ROS_INFO("Read initial state containing %zu joints and %zu multi-dof joints", state.joint_state.name.size(), state.multi_dof_joint_state.joint_names.size());
    return true;
}

struct RobotModelConfig
{
    std::string group_name;
    std::vector<std::string> planning_joints;
    std::string planning_link;
    std::string kinematics_frame;
    std::string chain_tip_link;
};

bool ReadRobotModelConfig(const ros::NodeHandle &nh, RobotModelConfig &config)
{
    if (!nh.getParam("group_name", config.group_name)) {
        ROS_ERROR("Failed to read 'group_name' from the param server");
        return false;
    }

    std::string planning_joint_list;
    if (!nh.getParam("planning_joints", planning_joint_list)) {
        ROS_ERROR("Failed to read 'planning_joints' from the param server");
        return false;
    }

    if (!nh.getParam("planning_link", config.planning_link)) {
        ROS_ERROR("Failed to read 'planning_link' from the param server");
        return false;
    }

    std::stringstream joint_name_stream(planning_joint_list);
    while (joint_name_stream.good() && !joint_name_stream.eof()) {
        std::string jname;
        joint_name_stream >> jname;
        if (jname.empty()) {
            continue;
        }
        config.planning_joints.push_back(jname);
    }

    // only required for generic kdl robot model?
    nh.getParam("kinematics_frame", config.kinematics_frame);
    nh.getParam("chain_tip_link", config.chain_tip_link);
    return true;
}

struct PlannerConfig
{
    std::string discretization;
    std::string mprim_filename;
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    double xyz_snap_dist_thresh;
    double rpy_snap_dist_thresh;
    double xyzrpy_snap_dist_thresh;
    double short_dist_mprims_thresh;
};

bool ReadPlannerConfig(const ros::NodeHandle &nh, PlannerConfig &config)
{
    if (!nh.getParam("discretization", config.discretization)) {
        ROS_ERROR("Failed to read 'discretization' from the param server");
        return false;
    }

    if (!nh.getParam("mprim_filename", config.mprim_filename)) {
        ROS_ERROR("Failed to read param 'mprim_filename' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyz_snap_mprim", config.use_xyz_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_rpy_snap_mprim", config.use_rpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_rpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyzrpy_snap_mprim", config.use_xyzrpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyzrpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_short_dist_mprims", config.use_short_dist_mprims)) {
        ROS_ERROR("Failed to read param 'use_short_dist_mprims' from the param server");
        return false;
    }

    if (!nh.getParam("xyz_snap_dist_thresh", config.xyz_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyz_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("rpy_snap_dist_thresh", config.rpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'rpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("xyzrpy_snap_dist_thresh", config.xyzrpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyzrpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("short_dist_mprims_thresh", config.short_dist_mprims_thresh)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    return true;
}

auto SetupRobotModel(
    const std::string& urdf,
    const RobotModelConfig &config,
    const std::string& planning_frame)
    -> std::unique_ptr<smpl::KDLRobotModel>
{
    std::unique_ptr<smpl::KDLRobotModel> rm;

    if (config.kinematics_frame.empty() || config.chain_tip_link.empty()) {
        ROS_ERROR("Failed to retrieve param 'kinematics_frame' or 'chain_tip_link' from the param server");
        return rm;
    }

    if (config.group_name == "right_arm") {
        ROS_INFO("Construct PR2 Robot Model");
        rm.reset(new smpl::PR2KDLRobotModel);
    } else if (config.group_name == "arm") {
        ROS_INFO("Construct UBR1 Robot Model");
        rm.reset(new smpl::UBR1KDLRobotModel);
    } else {
        ROS_INFO("Construct Generic KDL Robot Model");
        rm.reset(new sbpl::motion::KDLRobotModel);
    }

    if (!rm->init(
            urdf,
            config.planning_joints,
            config.kinematics_frame,
            config.chain_tip_link))
    {
        ROS_ERROR("Failed to initialize robot model.");
        rm.reset();
        return std::move(rm);
    }

    if (!rm->setPlanningLink(config.planning_link)) {
        ROS_ERROR("Failed to set planning link to '%s'", config.planning_link.c_str());
        rm.reset();
        return std::move(rm);
    }

    return std::move(rm);
}

void initAllowedCollisionsPR2(sbpl::collision::CollisionSpace &cspace)
{
    sbpl::collision::AllowedCollisionMatrix acm;
    // copied from the srdf for the pr2
    acm.setEntry("base_bellow_link", "base_footprint", true);
    acm.setEntry("base_bellow_link", "base_link", true);
    acm.setEntry("base_bellow_link", "bl_caster_l_wheel_link", true);
    acm.setEntry("base_bellow_link", "bl_caster_r_wheel_link", true);
    acm.setEntry("base_bellow_link", "bl_caster_rotation_link", true);
    acm.setEntry("base_bellow_link", "br_caster_l_wheel_link", true);
    acm.setEntry("base_bellow_link", "br_caster_r_wheel_link", true);
    acm.setEntry("base_bellow_link", "br_caster_rotation_link", true);
    acm.setEntry("base_bellow_link", "double_stereo_link", true);
    acm.setEntry("base_bellow_link", "fl_caster_l_wheel_link", true);
    acm.setEntry("base_bellow_link", "fl_caster_r_wheel_link", true);
    acm.setEntry("base_bellow_link", "fl_caster_rotation_link", true);
    acm.setEntry("base_bellow_link", "fr_caster_l_wheel_link", true);
    acm.setEntry("base_bellow_link", "fr_caster_r_wheel_link", true);
    acm.setEntry("base_bellow_link", "fr_caster_rotation_link", true);
    acm.setEntry("base_bellow_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("base_bellow_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("base_bellow_link", "head_mount_link", true);
    acm.setEntry("base_bellow_link", "head_mount_prosilica_link", true);
    acm.setEntry("base_bellow_link", "head_pan_link", true);
    acm.setEntry("base_bellow_link", "head_plate_frame", true);
    acm.setEntry("base_bellow_link", "head_tilt_link", true);
    acm.setEntry("base_bellow_link", "l_elbow_flex_link", true);
    acm.setEntry("base_bellow_link", "l_forearm_roll_link", true);
    acm.setEntry("base_bellow_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("base_bellow_link", "l_shoulder_lift_link", true);
    acm.setEntry("base_bellow_link", "l_shoulder_pan_link", true);
    acm.setEntry("base_bellow_link", "l_upper_arm_link", true);
    acm.setEntry("base_bellow_link", "l_upper_arm_roll_link", true);
    acm.setEntry("base_bellow_link", "l_wrist_flex_link", true);
    acm.setEntry("base_bellow_link", "l_wrist_roll_link", true);
    acm.setEntry("base_bellow_link", "laser_tilt_mount_link", true);
    acm.setEntry("base_bellow_link", "r_elbow_flex_link", true);
    acm.setEntry("base_bellow_link", "r_forearm_roll_link", true);
    acm.setEntry("base_bellow_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("base_bellow_link", "r_shoulder_lift_link", true);
    acm.setEntry("base_bellow_link", "r_shoulder_pan_link", true);
    acm.setEntry("base_bellow_link", "r_upper_arm_link", true);
    acm.setEntry("base_bellow_link", "r_upper_arm_roll_link", true);
    acm.setEntry("base_bellow_link", "sensor_mount_link", true);
    acm.setEntry("base_bellow_link", "torso_lift_link", true);
    acm.setEntry("base_footprint", "base_link", true);
    acm.setEntry("base_footprint", "bl_caster_l_wheel_link", true);
    acm.setEntry("base_footprint", "bl_caster_r_wheel_link", true);
    acm.setEntry("base_footprint", "bl_caster_rotation_link", true);
    acm.setEntry("base_footprint", "br_caster_l_wheel_link", true);
    acm.setEntry("base_footprint", "br_caster_r_wheel_link", true);
    acm.setEntry("base_footprint", "br_caster_rotation_link", true);
    acm.setEntry("base_footprint", "double_stereo_link", true);
    acm.setEntry("base_footprint", "fl_caster_l_wheel_link", true);
    acm.setEntry("base_footprint", "fl_caster_r_wheel_link", true);
    acm.setEntry("base_footprint", "fl_caster_rotation_link", true);
    acm.setEntry("base_footprint", "fr_caster_l_wheel_link", true);
    acm.setEntry("base_footprint", "fr_caster_r_wheel_link", true);
    acm.setEntry("base_footprint", "fr_caster_rotation_link", true);
    acm.setEntry("base_footprint", "head_mount_kinect_ir_link", true);
    acm.setEntry("base_footprint", "head_mount_kinect_rgb_link", true);
    acm.setEntry("base_footprint", "head_mount_link", true);
    acm.setEntry("base_footprint", "head_mount_prosilica_link", true);
    acm.setEntry("base_footprint", "head_pan_link", true);
    acm.setEntry("base_footprint", "head_plate_frame", true);
    acm.setEntry("base_footprint", "head_tilt_link", true);
    acm.setEntry("base_footprint", "l_elbow_flex_link", true);
    acm.setEntry("base_footprint", "l_forearm_link", true);
    acm.setEntry("base_footprint", "l_forearm_roll_link", true);
    acm.setEntry("base_footprint", "l_gripper_l_finger_link", true);
    acm.setEntry("base_footprint", "l_gripper_l_finger_tip_link", true);
    acm.setEntry("base_footprint", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("base_footprint", "l_gripper_palm_link", true);
    acm.setEntry("base_footprint", "l_gripper_r_finger_link", true);
    acm.setEntry("base_footprint", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("base_footprint", "l_shoulder_lift_link", true);
    acm.setEntry("base_footprint", "l_shoulder_pan_link", true);
    acm.setEntry("base_footprint", "l_upper_arm_link", true);
    acm.setEntry("base_footprint", "l_upper_arm_roll_link", true);
    acm.setEntry("base_footprint", "l_wrist_flex_link", true);
    acm.setEntry("base_footprint", "l_wrist_roll_link", true);
    acm.setEntry("base_footprint", "laser_tilt_mount_link", true);
    acm.setEntry("base_footprint", "r_elbow_flex_link", true);
    acm.setEntry("base_footprint", "r_forearm_link", true);
    acm.setEntry("base_footprint", "r_forearm_roll_link", true);
    acm.setEntry("base_footprint", "r_gripper_l_finger_link", true);
    acm.setEntry("base_footprint", "r_gripper_l_finger_tip_link", true);
    acm.setEntry("base_footprint", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("base_footprint", "r_gripper_palm_link", true);
    acm.setEntry("base_footprint", "r_gripper_r_finger_link", true);
    acm.setEntry("base_footprint", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("base_footprint", "r_shoulder_lift_link", true);
    acm.setEntry("base_footprint", "r_shoulder_pan_link", true);
    acm.setEntry("base_footprint", "r_upper_arm_link", true);
    acm.setEntry("base_footprint", "r_upper_arm_roll_link", true);
    acm.setEntry("base_footprint", "r_wrist_flex_link", true);
    acm.setEntry("base_footprint", "r_wrist_roll_link", true);
    acm.setEntry("base_footprint", "sensor_mount_link", true);
    acm.setEntry("base_footprint", "torso_lift_link", true);
    acm.setEntry("base_link", "bl_caster_l_wheel_link", true);
    acm.setEntry("base_link", "bl_caster_r_wheel_link", true);
    acm.setEntry("base_link", "bl_caster_rotation_link", true);
    acm.setEntry("base_link", "br_caster_l_wheel_link", true);
    acm.setEntry("base_link", "br_caster_r_wheel_link", true);
    acm.setEntry("base_link", "br_caster_rotation_link", true);
    acm.setEntry("base_link", "double_stereo_link", true);
    acm.setEntry("base_link", "fl_caster_l_wheel_link", true);
    acm.setEntry("base_link", "fl_caster_r_wheel_link", true);
    acm.setEntry("base_link", "fl_caster_rotation_link", true);
    acm.setEntry("base_link", "fr_caster_l_wheel_link", true);
    acm.setEntry("base_link", "fr_caster_r_wheel_link", true);
    acm.setEntry("base_link", "fr_caster_rotation_link", true);
    acm.setEntry("base_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("base_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("base_link", "head_mount_link", true);
    acm.setEntry("base_link", "head_mount_prosilica_link", true);
    acm.setEntry("base_link", "head_pan_link", true);
    acm.setEntry("base_link", "head_plate_frame", true);
    acm.setEntry("base_link", "head_tilt_link", true);
    acm.setEntry("base_link", "l_shoulder_lift_link", true);
    acm.setEntry("base_link", "l_shoulder_pan_link", true);
    acm.setEntry("base_link", "l_upper_arm_link", true);
    acm.setEntry("base_link", "l_upper_arm_roll_link", true);
    acm.setEntry("base_link", "laser_tilt_mount_link", true);
    acm.setEntry("base_link", "r_shoulder_lift_link", true);
    acm.setEntry("base_link", "r_shoulder_pan_link", true);
    acm.setEntry("base_link", "r_upper_arm_link", true);
    acm.setEntry("base_link", "r_upper_arm_roll_link", true);
    acm.setEntry("base_link", "sensor_mount_link", true);
    acm.setEntry("base_link", "torso_lift_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "bl_caster_r_wheel_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "bl_caster_rotation_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "br_caster_l_wheel_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "br_caster_r_wheel_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "br_caster_rotation_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "double_stereo_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "fl_caster_l_wheel_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "fl_caster_r_wheel_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "fl_caster_rotation_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "fr_caster_l_wheel_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "fr_caster_r_wheel_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "fr_caster_rotation_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "head_mount_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "head_mount_prosilica_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "head_pan_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "head_plate_frame", true);
    acm.setEntry("bl_caster_l_wheel_link", "head_tilt_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "l_elbow_flex_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "l_forearm_roll_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "l_shoulder_lift_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "l_shoulder_pan_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "l_upper_arm_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "l_upper_arm_roll_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "l_wrist_roll_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "laser_tilt_mount_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_elbow_flex_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_forearm_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_forearm_roll_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_gripper_l_finger_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_gripper_l_finger_tip_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_gripper_palm_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_gripper_r_finger_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_shoulder_lift_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_shoulder_pan_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_upper_arm_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_upper_arm_roll_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_wrist_flex_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "r_wrist_roll_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "sensor_mount_link", true);
    acm.setEntry("bl_caster_l_wheel_link", "torso_lift_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "bl_caster_rotation_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "br_caster_l_wheel_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "br_caster_r_wheel_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "br_caster_rotation_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "double_stereo_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "fl_caster_l_wheel_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "fl_caster_r_wheel_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "fl_caster_rotation_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "fr_caster_l_wheel_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "fr_caster_r_wheel_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "fr_caster_rotation_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "head_mount_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "head_mount_prosilica_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "head_pan_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "head_plate_frame", true);
    acm.setEntry("bl_caster_r_wheel_link", "head_tilt_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "l_elbow_flex_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "l_forearm_roll_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "l_shoulder_lift_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "l_shoulder_pan_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "l_upper_arm_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "l_upper_arm_roll_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "l_wrist_roll_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "laser_tilt_mount_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_elbow_flex_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_forearm_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_forearm_roll_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_gripper_l_finger_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_gripper_l_finger_tip_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_gripper_palm_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_gripper_r_finger_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_shoulder_lift_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_shoulder_pan_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_upper_arm_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_upper_arm_roll_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_wrist_flex_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "r_wrist_roll_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "sensor_mount_link", true);
    acm.setEntry("bl_caster_r_wheel_link", "torso_lift_link", true);
    acm.setEntry("bl_caster_rotation_link", "br_caster_l_wheel_link", true);
    acm.setEntry("bl_caster_rotation_link", "br_caster_r_wheel_link", true);
    acm.setEntry("bl_caster_rotation_link", "br_caster_rotation_link", true);
    acm.setEntry("bl_caster_rotation_link", "double_stereo_link", true);
    acm.setEntry("bl_caster_rotation_link", "fl_caster_l_wheel_link", true);
    acm.setEntry("bl_caster_rotation_link", "fl_caster_r_wheel_link", true);
    acm.setEntry("bl_caster_rotation_link", "fl_caster_rotation_link", true);
    acm.setEntry("bl_caster_rotation_link", "fr_caster_l_wheel_link", true);
    acm.setEntry("bl_caster_rotation_link", "fr_caster_r_wheel_link", true);
    acm.setEntry("bl_caster_rotation_link", "fr_caster_rotation_link", true);
    acm.setEntry("bl_caster_rotation_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("bl_caster_rotation_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("bl_caster_rotation_link", "head_mount_link", true);
    acm.setEntry("bl_caster_rotation_link", "head_mount_prosilica_link", true);
    acm.setEntry("bl_caster_rotation_link", "head_pan_link", true);
    acm.setEntry("bl_caster_rotation_link", "head_plate_frame", true);
    acm.setEntry("bl_caster_rotation_link", "head_tilt_link", true);
    acm.setEntry("bl_caster_rotation_link", "l_elbow_flex_link", true);
    acm.setEntry("bl_caster_rotation_link", "l_forearm_roll_link", true);
    acm.setEntry("bl_caster_rotation_link", "l_shoulder_lift_link", true);
    acm.setEntry("bl_caster_rotation_link", "l_shoulder_pan_link", true);
    acm.setEntry("bl_caster_rotation_link", "l_upper_arm_link", true);
    acm.setEntry("bl_caster_rotation_link", "l_upper_arm_roll_link", true);
    acm.setEntry("bl_caster_rotation_link", "laser_tilt_mount_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_elbow_flex_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_forearm_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_forearm_roll_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_gripper_l_finger_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_gripper_l_finger_tip_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_gripper_palm_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_gripper_r_finger_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_shoulder_lift_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_shoulder_pan_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_upper_arm_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_upper_arm_roll_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_wrist_flex_link", true);
    acm.setEntry("bl_caster_rotation_link", "r_wrist_roll_link", true);
    acm.setEntry("bl_caster_rotation_link", "sensor_mount_link", true);
    acm.setEntry("bl_caster_rotation_link", "torso_lift_link", true);
    acm.setEntry("br_caster_l_wheel_link", "br_caster_r_wheel_link", true);
    acm.setEntry("br_caster_l_wheel_link", "br_caster_rotation_link", true);
    acm.setEntry("br_caster_l_wheel_link", "double_stereo_link", true);
    acm.setEntry("br_caster_l_wheel_link", "fl_caster_l_wheel_link", true);
    acm.setEntry("br_caster_l_wheel_link", "fl_caster_r_wheel_link", true);
    acm.setEntry("br_caster_l_wheel_link", "fl_caster_rotation_link", true);
    acm.setEntry("br_caster_l_wheel_link", "fr_caster_l_wheel_link", true);
    acm.setEntry("br_caster_l_wheel_link", "fr_caster_r_wheel_link", true);
    acm.setEntry("br_caster_l_wheel_link", "fr_caster_rotation_link", true);
    acm.setEntry("br_caster_l_wheel_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("br_caster_l_wheel_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("br_caster_l_wheel_link", "head_mount_link", true);
    acm.setEntry("br_caster_l_wheel_link", "head_mount_prosilica_link", true);
    acm.setEntry("br_caster_l_wheel_link", "head_pan_link", true);
    acm.setEntry("br_caster_l_wheel_link", "head_plate_frame", true);
    acm.setEntry("br_caster_l_wheel_link", "head_tilt_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_elbow_flex_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_forearm_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_forearm_roll_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_gripper_l_finger_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_gripper_l_finger_tip_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_gripper_palm_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_gripper_r_finger_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_shoulder_lift_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_shoulder_pan_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_upper_arm_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_upper_arm_roll_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_wrist_flex_link", true);
    acm.setEntry("br_caster_l_wheel_link", "l_wrist_roll_link", true);
    acm.setEntry("br_caster_l_wheel_link", "laser_tilt_mount_link", true);
    acm.setEntry("br_caster_l_wheel_link", "r_elbow_flex_link", true);
    acm.setEntry("br_caster_l_wheel_link", "r_forearm_roll_link", true);
    acm.setEntry("br_caster_l_wheel_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("br_caster_l_wheel_link", "r_shoulder_lift_link", true);
    acm.setEntry("br_caster_l_wheel_link", "r_shoulder_pan_link", true);
    acm.setEntry("br_caster_l_wheel_link", "r_upper_arm_link", true);
    acm.setEntry("br_caster_l_wheel_link", "r_upper_arm_roll_link", true);
    acm.setEntry("br_caster_l_wheel_link", "sensor_mount_link", true);
    acm.setEntry("br_caster_l_wheel_link", "torso_lift_link", true);
    acm.setEntry("br_caster_r_wheel_link", "br_caster_rotation_link", true);
    acm.setEntry("br_caster_r_wheel_link", "double_stereo_link", true);
    acm.setEntry("br_caster_r_wheel_link", "fl_caster_l_wheel_link", true);
    acm.setEntry("br_caster_r_wheel_link", "fl_caster_r_wheel_link", true);
    acm.setEntry("br_caster_r_wheel_link", "fl_caster_rotation_link", true);
    acm.setEntry("br_caster_r_wheel_link", "fr_caster_l_wheel_link", true);
    acm.setEntry("br_caster_r_wheel_link", "fr_caster_r_wheel_link", true);
    acm.setEntry("br_caster_r_wheel_link", "fr_caster_rotation_link", true);
    acm.setEntry("br_caster_r_wheel_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("br_caster_r_wheel_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("br_caster_r_wheel_link", "head_mount_link", true);
    acm.setEntry("br_caster_r_wheel_link", "head_mount_prosilica_link", true);
    acm.setEntry("br_caster_r_wheel_link", "head_pan_link", true);
    acm.setEntry("br_caster_r_wheel_link", "head_plate_frame", true);
    acm.setEntry("br_caster_r_wheel_link", "head_tilt_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_elbow_flex_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_forearm_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_forearm_roll_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_gripper_l_finger_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_gripper_l_finger_tip_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_gripper_palm_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_gripper_r_finger_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_shoulder_lift_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_shoulder_pan_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_upper_arm_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_upper_arm_roll_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_wrist_flex_link", true);
    acm.setEntry("br_caster_r_wheel_link", "l_wrist_roll_link", true);
    acm.setEntry("br_caster_r_wheel_link", "laser_tilt_mount_link", true);
    acm.setEntry("br_caster_r_wheel_link", "r_elbow_flex_link", true);
    acm.setEntry("br_caster_r_wheel_link", "r_forearm_roll_link", true);
    acm.setEntry("br_caster_r_wheel_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("br_caster_r_wheel_link", "r_shoulder_lift_link", true);
    acm.setEntry("br_caster_r_wheel_link", "r_shoulder_pan_link", true);
    acm.setEntry("br_caster_r_wheel_link", "r_upper_arm_link", true);
    acm.setEntry("br_caster_r_wheel_link", "r_upper_arm_roll_link", true);
    acm.setEntry("br_caster_r_wheel_link", "r_wrist_roll_link", true);
    acm.setEntry("br_caster_r_wheel_link", "sensor_mount_link", true);
    acm.setEntry("br_caster_r_wheel_link", "torso_lift_link", true);
    acm.setEntry("br_caster_rotation_link", "double_stereo_link", true);
    acm.setEntry("br_caster_rotation_link", "fl_caster_l_wheel_link", true);
    acm.setEntry("br_caster_rotation_link", "fl_caster_r_wheel_link", true);
    acm.setEntry("br_caster_rotation_link", "fl_caster_rotation_link", true);
    acm.setEntry("br_caster_rotation_link", "fr_caster_l_wheel_link", true);
    acm.setEntry("br_caster_rotation_link", "fr_caster_r_wheel_link", true);
    acm.setEntry("br_caster_rotation_link", "fr_caster_rotation_link", true);
    acm.setEntry("br_caster_rotation_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("br_caster_rotation_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("br_caster_rotation_link", "head_mount_link", true);
    acm.setEntry("br_caster_rotation_link", "head_mount_prosilica_link", true);
    acm.setEntry("br_caster_rotation_link", "head_pan_link", true);
    acm.setEntry("br_caster_rotation_link", "head_plate_frame", true);
    acm.setEntry("br_caster_rotation_link", "head_tilt_link", true);
    acm.setEntry("br_caster_rotation_link", "l_elbow_flex_link", true);
    acm.setEntry("br_caster_rotation_link", "l_forearm_link", true);
    acm.setEntry("br_caster_rotation_link", "l_forearm_roll_link", true);
    acm.setEntry("br_caster_rotation_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("br_caster_rotation_link", "l_gripper_palm_link", true);
    acm.setEntry("br_caster_rotation_link", "l_gripper_r_finger_link", true);
    acm.setEntry("br_caster_rotation_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("br_caster_rotation_link", "l_shoulder_lift_link", true);
    acm.setEntry("br_caster_rotation_link", "l_shoulder_pan_link", true);
    acm.setEntry("br_caster_rotation_link", "l_upper_arm_link", true);
    acm.setEntry("br_caster_rotation_link", "l_upper_arm_roll_link", true);
    acm.setEntry("br_caster_rotation_link", "l_wrist_flex_link", true);
    acm.setEntry("br_caster_rotation_link", "l_wrist_roll_link", true);
    acm.setEntry("br_caster_rotation_link", "laser_tilt_mount_link", true);
    acm.setEntry("br_caster_rotation_link", "r_elbow_flex_link", true);
    acm.setEntry("br_caster_rotation_link", "r_forearm_roll_link", true);
    acm.setEntry("br_caster_rotation_link", "r_shoulder_lift_link", true);
    acm.setEntry("br_caster_rotation_link", "r_shoulder_pan_link", true);
    acm.setEntry("br_caster_rotation_link", "r_upper_arm_link", true);
    acm.setEntry("br_caster_rotation_link", "r_upper_arm_roll_link", true);
    acm.setEntry("br_caster_rotation_link", "sensor_mount_link", true);
    acm.setEntry("br_caster_rotation_link", "torso_lift_link", true);
    acm.setEntry("double_stereo_link", "fl_caster_l_wheel_link", true);
    acm.setEntry("double_stereo_link", "fl_caster_r_wheel_link", true);
    acm.setEntry("double_stereo_link", "fl_caster_rotation_link", true);
    acm.setEntry("double_stereo_link", "fr_caster_l_wheel_link", true);
    acm.setEntry("double_stereo_link", "fr_caster_r_wheel_link", true);
    acm.setEntry("double_stereo_link", "fr_caster_rotation_link", true);
    acm.setEntry("double_stereo_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("double_stereo_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("double_stereo_link", "head_mount_link", true);
    acm.setEntry("double_stereo_link", "head_mount_prosilica_link", true);
    acm.setEntry("double_stereo_link", "head_pan_link", true);
    acm.setEntry("double_stereo_link", "head_plate_frame", true);
    acm.setEntry("double_stereo_link", "head_tilt_link", true);
    acm.setEntry("double_stereo_link", "l_elbow_flex_link", true);
    acm.setEntry("double_stereo_link", "l_forearm_link", true);
    acm.setEntry("double_stereo_link", "l_forearm_roll_link", true);
    acm.setEntry("double_stereo_link", "l_gripper_l_finger_link", true);
    acm.setEntry("double_stereo_link", "l_gripper_l_finger_tip_link", true);
    acm.setEntry("double_stereo_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("double_stereo_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("double_stereo_link", "l_shoulder_lift_link", true);
    acm.setEntry("double_stereo_link", "l_shoulder_pan_link", true);
    acm.setEntry("double_stereo_link", "l_upper_arm_link", true);
    acm.setEntry("double_stereo_link", "l_upper_arm_roll_link", true);
    acm.setEntry("double_stereo_link", "l_wrist_flex_link", true);
    acm.setEntry("double_stereo_link", "l_wrist_roll_link", true);
    acm.setEntry("double_stereo_link", "laser_tilt_mount_link", true);
    acm.setEntry("double_stereo_link", "r_elbow_flex_link", true);
    acm.setEntry("double_stereo_link", "r_forearm_link", true);
    acm.setEntry("double_stereo_link", "r_forearm_roll_link", true);
    acm.setEntry("double_stereo_link", "r_gripper_l_finger_tip_link", true);
    acm.setEntry("double_stereo_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("double_stereo_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("double_stereo_link", "r_shoulder_lift_link", true);
    acm.setEntry("double_stereo_link", "r_shoulder_pan_link", true);
    acm.setEntry("double_stereo_link", "r_upper_arm_link", true);
    acm.setEntry("double_stereo_link", "r_upper_arm_roll_link", true);
    acm.setEntry("double_stereo_link", "r_wrist_flex_link", true);
    acm.setEntry("double_stereo_link", "r_wrist_roll_link", true);
    acm.setEntry("double_stereo_link", "sensor_mount_link", true);
    acm.setEntry("double_stereo_link", "torso_lift_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "fl_caster_r_wheel_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "fl_caster_rotation_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "fr_caster_l_wheel_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "fr_caster_r_wheel_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "fr_caster_rotation_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "head_mount_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "head_mount_prosilica_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "head_pan_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "head_plate_frame", true);
    acm.setEntry("fl_caster_l_wheel_link", "head_tilt_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "l_elbow_flex_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "l_forearm_roll_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "l_shoulder_lift_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "l_shoulder_pan_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "l_upper_arm_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "l_upper_arm_roll_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "laser_tilt_mount_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "r_elbow_flex_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "r_forearm_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "r_forearm_roll_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "r_shoulder_lift_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "r_shoulder_pan_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "r_upper_arm_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "r_upper_arm_roll_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "r_wrist_flex_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "r_wrist_roll_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "sensor_mount_link", true);
    acm.setEntry("fl_caster_l_wheel_link", "torso_lift_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "fl_caster_rotation_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "fr_caster_l_wheel_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "fr_caster_r_wheel_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "fr_caster_rotation_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "head_mount_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "head_mount_prosilica_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "head_pan_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "head_plate_frame", true);
    acm.setEntry("fl_caster_r_wheel_link", "head_tilt_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "l_elbow_flex_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "l_forearm_roll_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "l_shoulder_lift_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "l_shoulder_pan_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "l_upper_arm_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "l_upper_arm_roll_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "laser_tilt_mount_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "r_elbow_flex_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "r_forearm_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "r_forearm_roll_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "r_shoulder_lift_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "r_shoulder_pan_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "r_upper_arm_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "r_upper_arm_roll_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "r_wrist_flex_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "r_wrist_roll_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "sensor_mount_link", true);
    acm.setEntry("fl_caster_r_wheel_link", "torso_lift_link", true);
    acm.setEntry("fl_caster_rotation_link", "fr_caster_l_wheel_link", true);
    acm.setEntry("fl_caster_rotation_link", "fr_caster_r_wheel_link", true);
    acm.setEntry("fl_caster_rotation_link", "fr_caster_rotation_link", true);
    acm.setEntry("fl_caster_rotation_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("fl_caster_rotation_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("fl_caster_rotation_link", "head_mount_link", true);
    acm.setEntry("fl_caster_rotation_link", "head_mount_prosilica_link", true);
    acm.setEntry("fl_caster_rotation_link", "head_pan_link", true);
    acm.setEntry("fl_caster_rotation_link", "head_plate_frame", true);
    acm.setEntry("fl_caster_rotation_link", "head_tilt_link", true);
    acm.setEntry("fl_caster_rotation_link", "l_elbow_flex_link", true);
    acm.setEntry("fl_caster_rotation_link", "l_forearm_roll_link", true);
    acm.setEntry("fl_caster_rotation_link", "l_shoulder_lift_link", true);
    acm.setEntry("fl_caster_rotation_link", "l_shoulder_pan_link", true);
    acm.setEntry("fl_caster_rotation_link", "l_upper_arm_link", true);
    acm.setEntry("fl_caster_rotation_link", "l_upper_arm_roll_link", true);
    acm.setEntry("fl_caster_rotation_link", "laser_tilt_mount_link", true);
    acm.setEntry("fl_caster_rotation_link", "r_elbow_flex_link", true);
    acm.setEntry("fl_caster_rotation_link", "r_forearm_roll_link", true);
    acm.setEntry("fl_caster_rotation_link", "r_shoulder_lift_link", true);
    acm.setEntry("fl_caster_rotation_link", "r_shoulder_pan_link", true);
    acm.setEntry("fl_caster_rotation_link", "r_upper_arm_link", true);
    acm.setEntry("fl_caster_rotation_link", "r_upper_arm_roll_link", true);
    acm.setEntry("fl_caster_rotation_link", "sensor_mount_link", true);
    acm.setEntry("fl_caster_rotation_link", "torso_lift_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "fr_caster_r_wheel_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "fr_caster_rotation_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "head_mount_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "head_mount_prosilica_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "head_pan_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "head_plate_frame", true);
    acm.setEntry("fr_caster_l_wheel_link", "head_tilt_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "l_elbow_flex_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "l_forearm_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "l_forearm_roll_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "l_shoulder_lift_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "l_shoulder_pan_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "l_upper_arm_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "l_upper_arm_roll_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "l_wrist_flex_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "l_wrist_roll_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "laser_tilt_mount_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "r_elbow_flex_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "r_forearm_roll_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "r_shoulder_lift_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "r_shoulder_pan_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "r_upper_arm_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "r_upper_arm_roll_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "sensor_mount_link", true);
    acm.setEntry("fr_caster_l_wheel_link", "torso_lift_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "fr_caster_rotation_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "head_mount_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "head_mount_prosilica_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "head_pan_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "head_plate_frame", true);
    acm.setEntry("fr_caster_r_wheel_link", "head_tilt_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "l_elbow_flex_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "l_forearm_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "l_forearm_roll_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "l_shoulder_lift_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "l_shoulder_pan_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "l_upper_arm_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "l_upper_arm_roll_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "l_wrist_flex_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "l_wrist_roll_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "laser_tilt_mount_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "r_elbow_flex_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "r_forearm_roll_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "r_shoulder_lift_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "r_shoulder_pan_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "r_upper_arm_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "r_upper_arm_roll_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "sensor_mount_link", true);
    acm.setEntry("fr_caster_r_wheel_link", "torso_lift_link", true);
    acm.setEntry("fr_caster_rotation_link", "head_mount_kinect_ir_link", true);
    acm.setEntry("fr_caster_rotation_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("fr_caster_rotation_link", "head_mount_link", true);
    acm.setEntry("fr_caster_rotation_link", "head_mount_prosilica_link", true);
    acm.setEntry("fr_caster_rotation_link", "head_pan_link", true);
    acm.setEntry("fr_caster_rotation_link", "head_plate_frame", true);
    acm.setEntry("fr_caster_rotation_link", "head_tilt_link", true);
    acm.setEntry("fr_caster_rotation_link", "l_elbow_flex_link", true);
    acm.setEntry("fr_caster_rotation_link", "l_forearm_roll_link", true);
    acm.setEntry("fr_caster_rotation_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("fr_caster_rotation_link", "l_shoulder_lift_link", true);
    acm.setEntry("fr_caster_rotation_link", "l_shoulder_pan_link", true);
    acm.setEntry("fr_caster_rotation_link", "l_upper_arm_link", true);
    acm.setEntry("fr_caster_rotation_link", "l_upper_arm_roll_link", true);
    acm.setEntry("fr_caster_rotation_link", "laser_tilt_mount_link", true);
    acm.setEntry("fr_caster_rotation_link", "r_elbow_flex_link", true);
    acm.setEntry("fr_caster_rotation_link", "r_forearm_roll_link", true);
    acm.setEntry("fr_caster_rotation_link", "r_shoulder_lift_link", true);
    acm.setEntry("fr_caster_rotation_link", "r_shoulder_pan_link", true);
    acm.setEntry("fr_caster_rotation_link", "r_upper_arm_link", true);
    acm.setEntry("fr_caster_rotation_link", "r_upper_arm_roll_link", true);
    acm.setEntry("fr_caster_rotation_link", "sensor_mount_link", true);
    acm.setEntry("fr_caster_rotation_link", "torso_lift_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "head_mount_kinect_rgb_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "head_mount_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "head_mount_prosilica_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "head_pan_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "head_plate_frame", true);
    acm.setEntry("head_mount_kinect_ir_link", "head_tilt_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_elbow_flex_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_forearm_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_forearm_roll_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_gripper_l_finger_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_gripper_l_finger_tip_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_gripper_palm_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_gripper_r_finger_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_shoulder_lift_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_shoulder_pan_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_upper_arm_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_upper_arm_roll_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_wrist_flex_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "l_wrist_roll_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "laser_tilt_mount_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_elbow_flex_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_forearm_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_forearm_roll_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_gripper_l_finger_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_gripper_l_finger_tip_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_gripper_palm_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_gripper_r_finger_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_shoulder_lift_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_shoulder_pan_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_upper_arm_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_upper_arm_roll_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_wrist_flex_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "r_wrist_roll_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "sensor_mount_link", true);
    acm.setEntry("head_mount_kinect_ir_link", "torso_lift_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "head_mount_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "head_mount_prosilica_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "head_pan_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "head_plate_frame", true);
    acm.setEntry("head_mount_kinect_rgb_link", "head_tilt_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_elbow_flex_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_forearm_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_forearm_roll_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_gripper_l_finger_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_gripper_l_finger_tip_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_gripper_palm_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_gripper_r_finger_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_shoulder_lift_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_shoulder_pan_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_upper_arm_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_upper_arm_roll_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_wrist_flex_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "l_wrist_roll_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "laser_tilt_mount_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_elbow_flex_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_forearm_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_forearm_roll_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_gripper_l_finger_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_gripper_l_finger_tip_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_gripper_palm_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_gripper_r_finger_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_shoulder_lift_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_shoulder_pan_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_upper_arm_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_upper_arm_roll_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_wrist_flex_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "r_wrist_roll_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "sensor_mount_link", true);
    acm.setEntry("head_mount_kinect_rgb_link", "torso_lift_link", true);
    acm.setEntry("head_mount_link", "head_mount_prosilica_link", true);
    acm.setEntry("head_mount_link", "head_pan_link", true);
    acm.setEntry("head_mount_link", "head_plate_frame", true);
    acm.setEntry("head_mount_link", "head_tilt_link", true);
    acm.setEntry("head_mount_link", "l_elbow_flex_link", true);
    acm.setEntry("head_mount_link", "l_forearm_link", true);
    acm.setEntry("head_mount_link", "l_forearm_roll_link", true);
    acm.setEntry("head_mount_link", "l_gripper_l_finger_link", true);
    acm.setEntry("head_mount_link", "l_gripper_l_finger_tip_link", true);
    acm.setEntry("head_mount_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("head_mount_link", "l_gripper_palm_link", true);
    acm.setEntry("head_mount_link", "l_gripper_r_finger_link", true);
    acm.setEntry("head_mount_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("head_mount_link", "l_shoulder_lift_link", true);
    acm.setEntry("head_mount_link", "l_shoulder_pan_link", true);
    acm.setEntry("head_mount_link", "l_upper_arm_link", true);
    acm.setEntry("head_mount_link", "l_upper_arm_roll_link", true);
    acm.setEntry("head_mount_link", "l_wrist_flex_link", true);
    acm.setEntry("head_mount_link", "l_wrist_roll_link", true);
    acm.setEntry("head_mount_link", "laser_tilt_mount_link", true);
    acm.setEntry("head_mount_link", "r_elbow_flex_link", true);
    acm.setEntry("head_mount_link", "r_forearm_link", true);
    acm.setEntry("head_mount_link", "r_forearm_roll_link", true);
    acm.setEntry("head_mount_link", "r_gripper_l_finger_link", true);
    acm.setEntry("head_mount_link", "r_gripper_l_finger_tip_link", true);
    acm.setEntry("head_mount_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("head_mount_link", "r_gripper_palm_link", true);
    acm.setEntry("head_mount_link", "r_gripper_r_finger_link", true);
    acm.setEntry("head_mount_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("head_mount_link", "r_shoulder_lift_link", true);
    acm.setEntry("head_mount_link", "r_shoulder_pan_link", true);
    acm.setEntry("head_mount_link", "r_upper_arm_link", true);
    acm.setEntry("head_mount_link", "r_upper_arm_roll_link", true);
    acm.setEntry("head_mount_link", "r_wrist_flex_link", true);
    acm.setEntry("head_mount_link", "r_wrist_roll_link", true);
    acm.setEntry("head_mount_link", "sensor_mount_link", true);
    acm.setEntry("head_mount_link", "torso_lift_link", true);
    acm.setEntry("head_mount_prosilica_link", "head_pan_link", true);
    acm.setEntry("head_mount_prosilica_link", "head_plate_frame", true);
    acm.setEntry("head_mount_prosilica_link", "head_tilt_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_elbow_flex_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_forearm_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_forearm_roll_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_gripper_l_finger_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_gripper_l_finger_tip_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_gripper_palm_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_gripper_r_finger_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_shoulder_lift_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_shoulder_pan_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_upper_arm_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_upper_arm_roll_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_wrist_flex_link", true);
    acm.setEntry("head_mount_prosilica_link", "l_wrist_roll_link", true);
    acm.setEntry("head_mount_prosilica_link", "laser_tilt_mount_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_elbow_flex_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_forearm_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_forearm_roll_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_gripper_l_finger_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_gripper_l_finger_tip_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_gripper_palm_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_gripper_r_finger_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_shoulder_lift_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_shoulder_pan_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_upper_arm_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_upper_arm_roll_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_wrist_flex_link", true);
    acm.setEntry("head_mount_prosilica_link", "r_wrist_roll_link", true);
    acm.setEntry("head_mount_prosilica_link", "sensor_mount_link", true);
    acm.setEntry("head_mount_prosilica_link", "torso_lift_link", true);
    acm.setEntry("head_pan_link", "head_plate_frame", true);
    acm.setEntry("head_pan_link", "head_tilt_link", true);
    acm.setEntry("head_pan_link", "l_elbow_flex_link", true);
    acm.setEntry("head_pan_link", "l_forearm_roll_link", true);
    acm.setEntry("head_pan_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("head_pan_link", "l_shoulder_lift_link", true);
    acm.setEntry("head_pan_link", "l_shoulder_pan_link", true);
    acm.setEntry("head_pan_link", "l_upper_arm_link", true);
    acm.setEntry("head_pan_link", "l_upper_arm_roll_link", true);
    acm.setEntry("head_pan_link", "laser_tilt_mount_link", true);
    acm.setEntry("head_pan_link", "r_elbow_flex_link", true);
    acm.setEntry("head_pan_link", "r_forearm_roll_link", true);
    acm.setEntry("head_pan_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("head_pan_link", "r_shoulder_lift_link", true);
    acm.setEntry("head_pan_link", "r_shoulder_pan_link", true);
    acm.setEntry("head_pan_link", "r_upper_arm_link", true);
    acm.setEntry("head_pan_link", "r_upper_arm_roll_link", true);
    acm.setEntry("head_pan_link", "sensor_mount_link", true);
    acm.setEntry("head_pan_link", "torso_lift_link", true);
    acm.setEntry("head_plate_frame", "head_tilt_link", true);
    acm.setEntry("head_plate_frame", "l_elbow_flex_link", true);
    acm.setEntry("head_plate_frame", "l_forearm_link", true);
    acm.setEntry("head_plate_frame", "l_forearm_roll_link", true);
    acm.setEntry("head_plate_frame", "l_gripper_l_finger_link", true);
    acm.setEntry("head_plate_frame", "l_gripper_l_finger_tip_link", true);
    acm.setEntry("head_plate_frame", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("head_plate_frame", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("head_plate_frame", "l_shoulder_lift_link", true);
    acm.setEntry("head_plate_frame", "l_shoulder_pan_link", true);
    acm.setEntry("head_plate_frame", "l_upper_arm_link", true);
    acm.setEntry("head_plate_frame", "l_upper_arm_roll_link", true);
    acm.setEntry("head_plate_frame", "l_wrist_flex_link", true);
    acm.setEntry("head_plate_frame", "l_wrist_roll_link", true);
    acm.setEntry("head_plate_frame", "laser_tilt_mount_link", true);
    acm.setEntry("head_plate_frame", "r_elbow_flex_link", true);
    acm.setEntry("head_plate_frame", "r_forearm_link", true);
    acm.setEntry("head_plate_frame", "r_forearm_roll_link", true);
    acm.setEntry("head_plate_frame", "r_gripper_l_finger_link", true);
    acm.setEntry("head_plate_frame", "r_gripper_l_finger_tip_link", true);
    acm.setEntry("head_plate_frame", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("head_plate_frame", "r_gripper_palm_link", true);
    acm.setEntry("head_plate_frame", "r_gripper_r_finger_link", true);
    acm.setEntry("head_plate_frame", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("head_plate_frame", "r_shoulder_lift_link", true);
    acm.setEntry("head_plate_frame", "r_shoulder_pan_link", true);
    acm.setEntry("head_plate_frame", "r_upper_arm_link", true);
    acm.setEntry("head_plate_frame", "r_upper_arm_roll_link", true);
    acm.setEntry("head_plate_frame", "r_wrist_flex_link", true);
    acm.setEntry("head_plate_frame", "r_wrist_roll_link", true);
    acm.setEntry("head_plate_frame", "sensor_mount_link", true);
    acm.setEntry("head_plate_frame", "torso_lift_link", true);
    acm.setEntry("head_tilt_link", "l_elbow_flex_link", true);
    acm.setEntry("head_tilt_link", "l_forearm_link", true);
    acm.setEntry("head_tilt_link", "l_forearm_roll_link", true);
    acm.setEntry("head_tilt_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("head_tilt_link", "l_shoulder_lift_link", true);
    acm.setEntry("head_tilt_link", "l_shoulder_pan_link", true);
    acm.setEntry("head_tilt_link", "l_upper_arm_link", true);
    acm.setEntry("head_tilt_link", "l_upper_arm_roll_link", true);
    acm.setEntry("head_tilt_link", "l_wrist_roll_link", true);
    acm.setEntry("head_tilt_link", "laser_tilt_mount_link", true);
    acm.setEntry("head_tilt_link", "r_elbow_flex_link", true);
    acm.setEntry("head_tilt_link", "r_forearm_link", true);
    acm.setEntry("head_tilt_link", "r_forearm_roll_link", true);
    acm.setEntry("head_tilt_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("head_tilt_link", "r_shoulder_lift_link", true);
    acm.setEntry("head_tilt_link", "r_shoulder_pan_link", true);
    acm.setEntry("head_tilt_link", "r_upper_arm_link", true);
    acm.setEntry("head_tilt_link", "r_upper_arm_roll_link", true);
    acm.setEntry("head_tilt_link", "r_wrist_flex_link", true);
    acm.setEntry("head_tilt_link", "r_wrist_roll_link", true);
    acm.setEntry("head_tilt_link", "sensor_mount_link", true);
    acm.setEntry("head_tilt_link", "torso_lift_link", true);
    acm.setEntry("l_elbow_flex_link", "l_forearm_link", true);
    acm.setEntry("l_elbow_flex_link", "l_forearm_roll_link", true);
    acm.setEntry("l_elbow_flex_link", "l_gripper_l_finger_link", true);
    acm.setEntry("l_elbow_flex_link", "l_gripper_l_finger_tip_link", true);
    acm.setEntry("l_elbow_flex_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("l_elbow_flex_link", "l_gripper_palm_link", true);
    acm.setEntry("l_elbow_flex_link", "l_gripper_r_finger_link", true);
    acm.setEntry("l_elbow_flex_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("l_elbow_flex_link", "l_shoulder_lift_link", true);
    acm.setEntry("l_elbow_flex_link", "l_shoulder_pan_link", true);
    acm.setEntry("l_elbow_flex_link", "l_upper_arm_link", true);
    acm.setEntry("l_elbow_flex_link", "l_upper_arm_roll_link", true);
    acm.setEntry("l_elbow_flex_link", "l_wrist_flex_link", true);
    acm.setEntry("l_elbow_flex_link", "l_wrist_roll_link", true);
    acm.setEntry("l_elbow_flex_link", "laser_tilt_mount_link", true);
    acm.setEntry("l_elbow_flex_link", "r_shoulder_lift_link", true);
    acm.setEntry("l_elbow_flex_link", "r_shoulder_pan_link", true);
    acm.setEntry("l_elbow_flex_link", "r_upper_arm_roll_link", true);
    acm.setEntry("l_elbow_flex_link", "sensor_mount_link", true);
    acm.setEntry("l_elbow_flex_link", "torso_lift_link", true);
    acm.setEntry("l_forearm_link", "l_forearm_roll_link", true);
    acm.setEntry("l_forearm_link", "l_gripper_l_finger_link", true);
    acm.setEntry("l_forearm_link", "l_gripper_l_finger_tip_link", true);
    acm.setEntry("l_forearm_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("l_forearm_link", "l_gripper_palm_link", true);
    acm.setEntry("l_forearm_link", "l_gripper_r_finger_link", true);
    acm.setEntry("l_forearm_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("l_forearm_link", "l_shoulder_lift_link", true);
    acm.setEntry("l_forearm_link", "l_upper_arm_link", true);
    acm.setEntry("l_forearm_link", "l_upper_arm_roll_link", true);
    acm.setEntry("l_forearm_link", "l_wrist_flex_link", true);
    acm.setEntry("l_forearm_link", "l_wrist_roll_link", true);
    acm.setEntry("l_forearm_link", "sensor_mount_link", true);
    acm.setEntry("l_forearm_roll_link", "l_gripper_l_finger_link", true);
    acm.setEntry("l_forearm_roll_link", "l_gripper_l_finger_tip_link", true);
    acm.setEntry("l_forearm_roll_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("l_forearm_roll_link", "l_gripper_palm_link", true);
    acm.setEntry("l_forearm_roll_link", "l_gripper_r_finger_link", true);
    acm.setEntry("l_forearm_roll_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("l_forearm_roll_link", "l_shoulder_lift_link", true);
    acm.setEntry("l_forearm_roll_link", "l_shoulder_pan_link", true);
    acm.setEntry("l_forearm_roll_link", "l_upper_arm_link", true);
    acm.setEntry("l_forearm_roll_link", "l_upper_arm_roll_link", true);
    acm.setEntry("l_forearm_roll_link", "l_wrist_flex_link", true);
    acm.setEntry("l_forearm_roll_link", "l_wrist_roll_link", true);
    acm.setEntry("l_forearm_roll_link", "laser_tilt_mount_link", true);
    acm.setEntry("l_forearm_roll_link", "r_shoulder_lift_link", true);
    acm.setEntry("l_forearm_roll_link", "r_shoulder_pan_link", true);
    acm.setEntry("l_forearm_roll_link", "r_upper_arm_roll_link", true);
    acm.setEntry("l_forearm_roll_link", "sensor_mount_link", true);
    acm.setEntry("l_forearm_roll_link", "torso_lift_link", true);
    acm.setEntry("l_gripper_l_finger_link", "l_gripper_l_finger_tip_link", true);
    acm.setEntry("l_gripper_l_finger_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("l_gripper_l_finger_link", "l_gripper_palm_link", true);
    acm.setEntry("l_gripper_l_finger_link", "l_gripper_r_finger_link", true);
    acm.setEntry("l_gripper_l_finger_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("l_gripper_l_finger_link", "l_shoulder_lift_link", true);
    acm.setEntry("l_gripper_l_finger_link", "l_upper_arm_link", true);
    acm.setEntry("l_gripper_l_finger_link", "l_upper_arm_roll_link", true);
    acm.setEntry("l_gripper_l_finger_link", "l_wrist_flex_link", true);
    acm.setEntry("l_gripper_l_finger_link", "l_wrist_roll_link", true);
    acm.setEntry("l_gripper_l_finger_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("l_gripper_l_finger_link", "sensor_mount_link", true);
    acm.setEntry("l_gripper_l_finger_tip_link", "l_gripper_motor_accelerometer_link", true);
    acm.setEntry("l_gripper_l_finger_tip_link", "l_gripper_palm_link", true);
    acm.setEntry("l_gripper_l_finger_tip_link", "l_gripper_r_finger_link", true);
    acm.setEntry("l_gripper_l_finger_tip_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("l_gripper_l_finger_tip_link", "l_shoulder_lift_link", true);
    acm.setEntry("l_gripper_l_finger_tip_link", "l_upper_arm_link", true);
    acm.setEntry("l_gripper_l_finger_tip_link", "l_upper_arm_roll_link", true);
    acm.setEntry("l_gripper_l_finger_tip_link", "l_wrist_flex_link", true);
    acm.setEntry("l_gripper_l_finger_tip_link", "l_wrist_roll_link", true);
    acm.setEntry("l_gripper_l_finger_tip_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("l_gripper_l_finger_tip_link", "sensor_mount_link", true);
    acm.setEntry("l_gripper_motor_accelerometer_link", "l_gripper_palm_link", true);
    acm.setEntry("l_gripper_motor_accelerometer_link", "l_gripper_r_finger_link", true);
    acm.setEntry("l_gripper_motor_accelerometer_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("l_gripper_motor_accelerometer_link", "l_shoulder_lift_link", true);
    acm.setEntry("l_gripper_motor_accelerometer_link", "l_upper_arm_link", true);
    acm.setEntry("l_gripper_motor_accelerometer_link", "l_upper_arm_roll_link", true);
    acm.setEntry("l_gripper_motor_accelerometer_link", "l_wrist_flex_link", true);
    acm.setEntry("l_gripper_motor_accelerometer_link", "l_wrist_roll_link", true);
    acm.setEntry("l_gripper_motor_accelerometer_link", "laser_tilt_mount_link", true);
    acm.setEntry("l_gripper_motor_accelerometer_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("l_gripper_motor_accelerometer_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("l_gripper_motor_accelerometer_link", "r_upper_arm_roll_link", true);
    acm.setEntry("l_gripper_motor_accelerometer_link", "sensor_mount_link", true);
    acm.setEntry("l_gripper_palm_link", "l_gripper_r_finger_link", true);
    acm.setEntry("l_gripper_palm_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("l_gripper_palm_link", "l_shoulder_lift_link", true);
    acm.setEntry("l_gripper_palm_link", "l_upper_arm_link", true);
    acm.setEntry("l_gripper_palm_link", "l_upper_arm_roll_link", true);
    acm.setEntry("l_gripper_palm_link", "l_wrist_flex_link", true);
    acm.setEntry("l_gripper_palm_link", "l_wrist_roll_link", true);
    acm.setEntry("l_gripper_r_finger_link", "l_gripper_r_finger_tip_link", true);
    acm.setEntry("l_gripper_r_finger_link", "l_shoulder_lift_link", true);
    acm.setEntry("l_gripper_r_finger_link", "l_upper_arm_link", true);
    acm.setEntry("l_gripper_r_finger_link", "l_upper_arm_roll_link", true);
    acm.setEntry("l_gripper_r_finger_link", "l_wrist_flex_link", true);
    acm.setEntry("l_gripper_r_finger_link", "l_wrist_roll_link", true);
    acm.setEntry("l_gripper_r_finger_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("l_gripper_r_finger_tip_link", "l_shoulder_lift_link", true);
    acm.setEntry("l_gripper_r_finger_tip_link", "l_upper_arm_link", true);
    acm.setEntry("l_gripper_r_finger_tip_link", "l_upper_arm_roll_link", true);
    acm.setEntry("l_gripper_r_finger_tip_link", "l_wrist_flex_link", true);
    acm.setEntry("l_gripper_r_finger_tip_link", "l_wrist_roll_link", true);
    acm.setEntry("l_gripper_r_finger_tip_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("l_gripper_r_finger_tip_link", "sensor_mount_link", true);
    acm.setEntry("l_shoulder_lift_link", "l_shoulder_pan_link", true);
    acm.setEntry("l_shoulder_lift_link", "l_upper_arm_link", true);
    acm.setEntry("l_shoulder_lift_link", "l_upper_arm_roll_link", true);
    acm.setEntry("l_shoulder_lift_link", "l_wrist_flex_link", true);
    acm.setEntry("l_shoulder_lift_link", "l_wrist_roll_link", true);
    acm.setEntry("l_shoulder_lift_link", "laser_tilt_mount_link", true);
    acm.setEntry("l_shoulder_lift_link", "r_elbow_flex_link", true);
    acm.setEntry("l_shoulder_lift_link", "r_forearm_roll_link", true);
    acm.setEntry("l_shoulder_lift_link", "r_shoulder_lift_link", true);
    acm.setEntry("l_shoulder_lift_link", "r_upper_arm_link", true);
    acm.setEntry("l_shoulder_lift_link", "r_upper_arm_roll_link", true);
    acm.setEntry("l_shoulder_lift_link", "sensor_mount_link", true);
    acm.setEntry("l_shoulder_lift_link", "torso_lift_link", true);
    acm.setEntry("l_shoulder_pan_link", "l_upper_arm_link", true);
    acm.setEntry("l_shoulder_pan_link", "l_upper_arm_roll_link", true);
    acm.setEntry("l_shoulder_pan_link", "laser_tilt_mount_link", true);
    acm.setEntry("l_shoulder_pan_link", "r_elbow_flex_link", true);
    acm.setEntry("l_shoulder_pan_link", "r_forearm_roll_link", true);
    acm.setEntry("l_shoulder_pan_link", "sensor_mount_link", true);
    acm.setEntry("l_shoulder_pan_link", "torso_lift_link", true);
    acm.setEntry("l_upper_arm_link", "l_upper_arm_roll_link", true);
    acm.setEntry("l_upper_arm_link", "l_wrist_flex_link", true);
    acm.setEntry("l_upper_arm_link", "l_wrist_roll_link", true);
    acm.setEntry("l_upper_arm_link", "laser_tilt_mount_link", true);
    acm.setEntry("l_upper_arm_link", "r_shoulder_lift_link", true);
    acm.setEntry("l_upper_arm_link", "r_upper_arm_roll_link", true);
    acm.setEntry("l_upper_arm_link", "sensor_mount_link", true);
    acm.setEntry("l_upper_arm_link", "torso_lift_link", true);
    acm.setEntry("l_upper_arm_roll_link", "l_wrist_flex_link", true);
    acm.setEntry("l_upper_arm_roll_link", "l_wrist_roll_link", true);
    acm.setEntry("l_upper_arm_roll_link", "laser_tilt_mount_link", true);
    acm.setEntry("l_upper_arm_roll_link", "r_elbow_flex_link", true);
    acm.setEntry("l_upper_arm_roll_link", "r_forearm_roll_link", true);
    acm.setEntry("l_upper_arm_roll_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("l_upper_arm_roll_link", "r_shoulder_lift_link", true);
    acm.setEntry("l_upper_arm_roll_link", "r_shoulder_pan_link", true);
    acm.setEntry("l_upper_arm_roll_link", "r_upper_arm_roll_link", true);
    acm.setEntry("l_upper_arm_roll_link", "sensor_mount_link", true);
    acm.setEntry("l_upper_arm_roll_link", "torso_lift_link", true);
    acm.setEntry("l_wrist_flex_link", "l_wrist_roll_link", true);
    acm.setEntry("l_wrist_flex_link", "sensor_mount_link", true);
    acm.setEntry("l_wrist_roll_link", "laser_tilt_mount_link", true);
    acm.setEntry("l_wrist_roll_link", "sensor_mount_link", true);
    acm.setEntry("laser_tilt_mount_link", "r_elbow_flex_link", true);
    acm.setEntry("laser_tilt_mount_link", "r_forearm_roll_link", true);
    acm.setEntry("laser_tilt_mount_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("laser_tilt_mount_link", "r_shoulder_lift_link", true);
    acm.setEntry("laser_tilt_mount_link", "r_shoulder_pan_link", true);
    acm.setEntry("laser_tilt_mount_link", "r_upper_arm_link", true);
    acm.setEntry("laser_tilt_mount_link", "r_upper_arm_roll_link", true);
    acm.setEntry("laser_tilt_mount_link", "r_wrist_roll_link", true);
    acm.setEntry("laser_tilt_mount_link", "sensor_mount_link", true);
    acm.setEntry("laser_tilt_mount_link", "torso_lift_link", true);
    acm.setEntry("r_elbow_flex_link", "r_forearm_link", true);
    acm.setEntry("r_elbow_flex_link", "r_forearm_roll_link", true);
    acm.setEntry("r_elbow_flex_link", "r_gripper_l_finger_link", true);
    acm.setEntry("r_elbow_flex_link", "r_gripper_l_finger_tip_link", true);
    acm.setEntry("r_elbow_flex_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("r_elbow_flex_link", "r_gripper_palm_link", true);
    acm.setEntry("r_elbow_flex_link", "r_gripper_r_finger_link", true);
    acm.setEntry("r_elbow_flex_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("r_elbow_flex_link", "r_shoulder_lift_link", true);
    acm.setEntry("r_elbow_flex_link", "r_shoulder_pan_link", true);
    acm.setEntry("r_elbow_flex_link", "r_upper_arm_link", true);
    acm.setEntry("r_elbow_flex_link", "r_upper_arm_roll_link", true);
    acm.setEntry("r_elbow_flex_link", "r_wrist_flex_link", true);
    acm.setEntry("r_elbow_flex_link", "r_wrist_roll_link", true);
    acm.setEntry("r_elbow_flex_link", "sensor_mount_link", true);
    acm.setEntry("r_elbow_flex_link", "torso_lift_link", true);
    acm.setEntry("r_forearm_link", "r_forearm_roll_link", true);
    acm.setEntry("r_forearm_link", "r_gripper_l_finger_link", true);
    acm.setEntry("r_forearm_link", "r_gripper_l_finger_tip_link", true);
    acm.setEntry("r_forearm_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("r_forearm_link", "r_gripper_palm_link", true);
    acm.setEntry("r_forearm_link", "r_gripper_r_finger_link", true);
    acm.setEntry("r_forearm_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("r_forearm_link", "r_shoulder_lift_link", true);
    acm.setEntry("r_forearm_link", "r_upper_arm_link", true);
    acm.setEntry("r_forearm_link", "r_upper_arm_roll_link", true);
    acm.setEntry("r_forearm_link", "r_wrist_flex_link", true);
    acm.setEntry("r_forearm_link", "r_wrist_roll_link", true);
    acm.setEntry("r_forearm_link", "sensor_mount_link", true);
    acm.setEntry("r_forearm_roll_link", "r_gripper_l_finger_link", true);
    acm.setEntry("r_forearm_roll_link", "r_gripper_l_finger_tip_link", true);
    acm.setEntry("r_forearm_roll_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("r_forearm_roll_link", "r_gripper_palm_link", true);
    acm.setEntry("r_forearm_roll_link", "r_gripper_r_finger_link", true);
    acm.setEntry("r_forearm_roll_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("r_forearm_roll_link", "r_shoulder_lift_link", true);
    acm.setEntry("r_forearm_roll_link", "r_shoulder_pan_link", true);
    acm.setEntry("r_forearm_roll_link", "r_upper_arm_link", true);
    acm.setEntry("r_forearm_roll_link", "r_upper_arm_roll_link", true);
    acm.setEntry("r_forearm_roll_link", "r_wrist_flex_link", true);
    acm.setEntry("r_forearm_roll_link", "r_wrist_roll_link", true);
    acm.setEntry("r_forearm_roll_link", "sensor_mount_link", true);
    acm.setEntry("r_forearm_roll_link", "torso_lift_link", true);
    acm.setEntry("r_gripper_l_finger_link", "r_gripper_l_finger_tip_link", true);
    acm.setEntry("r_gripper_l_finger_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("r_gripper_l_finger_link", "r_gripper_palm_link", true);
    acm.setEntry("r_gripper_l_finger_link", "r_gripper_r_finger_link", true);
    acm.setEntry("r_gripper_l_finger_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("r_gripper_l_finger_link", "r_shoulder_lift_link", true);
    acm.setEntry("r_gripper_l_finger_link", "r_upper_arm_link", true);
    acm.setEntry("r_gripper_l_finger_link", "r_upper_arm_roll_link", true);
    acm.setEntry("r_gripper_l_finger_link", "r_wrist_flex_link", true);
    acm.setEntry("r_gripper_l_finger_link", "r_wrist_roll_link", true);
    acm.setEntry("r_gripper_l_finger_link", "sensor_mount_link", true);
    acm.setEntry("r_gripper_l_finger_tip_link", "r_gripper_motor_accelerometer_link", true);
    acm.setEntry("r_gripper_l_finger_tip_link", "r_gripper_palm_link", true);
    acm.setEntry("r_gripper_l_finger_tip_link", "r_gripper_r_finger_link", true);
    acm.setEntry("r_gripper_l_finger_tip_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("r_gripper_l_finger_tip_link", "r_shoulder_lift_link", true);
    acm.setEntry("r_gripper_l_finger_tip_link", "r_upper_arm_link", true);
    acm.setEntry("r_gripper_l_finger_tip_link", "r_upper_arm_roll_link", true);
    acm.setEntry("r_gripper_l_finger_tip_link", "r_wrist_flex_link", true);
    acm.setEntry("r_gripper_l_finger_tip_link", "r_wrist_roll_link", true);
    acm.setEntry("r_gripper_l_finger_tip_link", "sensor_mount_link", true);
    acm.setEntry("r_gripper_motor_accelerometer_link", "r_gripper_palm_link", true);
    acm.setEntry("r_gripper_motor_accelerometer_link", "r_gripper_r_finger_link", true);
    acm.setEntry("r_gripper_motor_accelerometer_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("r_gripper_motor_accelerometer_link", "r_shoulder_lift_link", true);
    acm.setEntry("r_gripper_motor_accelerometer_link", "r_upper_arm_link", true);
    acm.setEntry("r_gripper_motor_accelerometer_link", "r_upper_arm_roll_link", true);
    acm.setEntry("r_gripper_motor_accelerometer_link", "r_wrist_flex_link", true);
    acm.setEntry("r_gripper_motor_accelerometer_link", "r_wrist_roll_link", true);
    acm.setEntry("r_gripper_motor_accelerometer_link", "sensor_mount_link", true);
    acm.setEntry("r_gripper_palm_link", "r_gripper_r_finger_link", true);
    acm.setEntry("r_gripper_palm_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("r_gripper_palm_link", "r_shoulder_lift_link", true);
    acm.setEntry("r_gripper_palm_link", "r_upper_arm_link", true);
    acm.setEntry("r_gripper_palm_link", "r_upper_arm_roll_link", true);
    acm.setEntry("r_gripper_palm_link", "r_wrist_flex_link", true);
    acm.setEntry("r_gripper_palm_link", "r_wrist_roll_link", true);
    acm.setEntry("r_gripper_palm_link", "sensor_mount_link", true);
    acm.setEntry("r_gripper_r_finger_link", "r_gripper_r_finger_tip_link", true);
    acm.setEntry("r_gripper_r_finger_link", "r_shoulder_lift_link", true);
    acm.setEntry("r_gripper_r_finger_link", "r_upper_arm_link", true);
    acm.setEntry("r_gripper_r_finger_link", "r_upper_arm_roll_link", true);
    acm.setEntry("r_gripper_r_finger_link", "r_wrist_flex_link", true);
    acm.setEntry("r_gripper_r_finger_link", "r_wrist_roll_link", true);
    acm.setEntry("r_gripper_r_finger_link", "sensor_mount_link", true);
    acm.setEntry("r_gripper_r_finger_tip_link", "r_shoulder_lift_link", true);
    acm.setEntry("r_gripper_r_finger_tip_link", "r_upper_arm_link", true);
    acm.setEntry("r_gripper_r_finger_tip_link", "r_upper_arm_roll_link", true);
    acm.setEntry("r_gripper_r_finger_tip_link", "r_wrist_flex_link", true);
    acm.setEntry("r_gripper_r_finger_tip_link", "r_wrist_roll_link", true);
    acm.setEntry("r_gripper_r_finger_tip_link", "sensor_mount_link", true);
    acm.setEntry("r_shoulder_lift_link", "r_shoulder_pan_link", true);
    acm.setEntry("r_shoulder_lift_link", "r_upper_arm_link", true);
    acm.setEntry("r_shoulder_lift_link", "r_upper_arm_roll_link", true);
    acm.setEntry("r_shoulder_lift_link", "r_wrist_flex_link", true);
    acm.setEntry("r_shoulder_lift_link", "r_wrist_roll_link", true);
    acm.setEntry("r_shoulder_lift_link", "sensor_mount_link", true);
    acm.setEntry("r_shoulder_lift_link", "torso_lift_link", true);
    acm.setEntry("r_shoulder_pan_link", "r_upper_arm_link", true);
    acm.setEntry("r_shoulder_pan_link", "r_upper_arm_roll_link", true);
    acm.setEntry("r_shoulder_pan_link", "sensor_mount_link", true);
    acm.setEntry("r_shoulder_pan_link", "torso_lift_link", true);
    acm.setEntry("r_upper_arm_link", "r_upper_arm_roll_link", true);
    acm.setEntry("r_upper_arm_link", "r_wrist_flex_link", true);
    acm.setEntry("r_upper_arm_link", "r_wrist_roll_link", true);
    acm.setEntry("r_upper_arm_link", "sensor_mount_link", true);
    acm.setEntry("r_upper_arm_link", "torso_lift_link", true);
    acm.setEntry("r_upper_arm_roll_link", "r_wrist_flex_link", true);
    acm.setEntry("r_upper_arm_roll_link", "r_wrist_roll_link", true);
    acm.setEntry("r_upper_arm_roll_link", "sensor_mount_link", true);
    acm.setEntry("r_upper_arm_roll_link", "torso_lift_link", true);
    acm.setEntry("r_wrist_flex_link", "r_wrist_roll_link", true);
    acm.setEntry("r_wrist_flex_link", "sensor_mount_link", true);
    acm.setEntry("r_wrist_roll_link", "sensor_mount_link", true);
    acm.setEntry("sensor_mount_link", "torso_lift_link", true);
    cspace.setAllowedCollisionMatrix(acm);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "smpl_test");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    sbpl::VisualizerROS visualizer(nh, 100);
    sbpl::viz::set_visualizer(&visualizer);

    // let publishers set up
    ros::Duration(1.0).sleep();

    // everyone needs to know the name of the planning frame for reasons...
    std::string planning_frame;
    if (!ph.getParam("planning_frame", planning_frame)) {
        ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
        return 1;
    }

    ROS_INFO("Planning Frame: %s", planning_frame.c_str());

    /////////////////
    // Robot Model //
    /////////////////

    const char *robot_description_key = "robot_description";
    std::string robot_description_param;
    if (!nh.searchParam(robot_description_key, robot_description_param)) {
        ROS_ERROR("Failed to find 'robot_description' key on the param server");
        return 1;
    }

    std::string urdf;
    if (!nh.getParam(robot_description_param, urdf)) {
        ROS_ERROR("Failed to retrieve param 'robot_description' from the param server");
        return 1;
    }

    RobotModelConfig rm_config;
    if (!ReadRobotModelConfig(ros::NodeHandle("~robot_model"), rm_config)) {
        ROS_ERROR("Failed to read robot model config from param server");
        return 1;
    }

    const std::string &group_name(rm_config.group_name);
    const std::vector<std::string> &planning_joints(rm_config.planning_joints);
    const std::string &planning_link(rm_config.planning_link);

    auto rm = SetupRobotModel(urdf, rm_config, planning_frame);
    if (!rm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    ////////////////////
    // Occupancy Grid //
    ////////////////////

    const double df_size_x = 3.0;
    const double df_size_y = 3.0;
    const double df_size_z = 3.0;
    const double df_res = 0.02;
    const double df_origin_x = -0.75;
    const double df_origin_y = -1.5;
    const double df_origin_z = 0.0;
    const double max_distance = 1.8;

//    typedef sbpl::EdgeEuclidDistanceMap DistanceMapType;
    typedef sbpl::EuclidDistanceMap DistanceMapType;
//    typedef sbpl::PropagationDistanceField DistanceMapType;

    ROS_INFO("Create distance map");
    auto df = std::make_shared<DistanceMapType>(
            df_origin_x, df_origin_y, df_origin_z,
            df_size_x, df_size_y, df_size_z,
            df_res,
            max_distance);

    ROS_INFO("Create grid");
    const bool ref_counted = false;
    sbpl::OccupancyGrid grid(df, ref_counted);

    grid.setReferenceFrame(planning_frame);
    SV_SHOW_INFO(grid.getBoundingBoxVisualization());

    ///////////////////////
    // Collision Checker //
    ///////////////////////

    sbpl::collision::CollisionModelConfig cc_conf;
    if (!sbpl::collision::CollisionModelConfig::Load(ph, cc_conf)) {
        ROS_ERROR("Failed to load Collision Model Config");
        return 1;
    }

    sbpl::collision::CollisionSpace cc;
    if (!cc.init(&grid, urdf, cc_conf, group_name, planning_joints)) {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }

    if (cc.robotCollisionModel()->name() == "pr2") {
        initAllowedCollisionsPR2(cc);
    }

    /////////////////
    // Scene Setup //
    /////////////////

    std::string object_filename;
    ph.param<std::string>("object_filename", object_filename, "");

    // collision objects
    moveit_msgs::PlanningScene scene;
    if (!object_filename.empty()) {
        scene.world.collision_objects = GetCollisionObjects(object_filename, planning_frame);
    }

    // fill start state
    if (!ReadInitialConfiguration(ph, scene.robot_state)) {
        ROS_ERROR("Failed to get initial configuration.");
        return 0;
    }
    scene.robot_model_name = cc.robotCollisionModel()->name();
    scene.robot_state.joint_state.header.frame_id = planning_frame;
    scene.world.octomap.header.frame_id = planning_frame;
    scene.world.octomap.octomap.binary = true;
    scene.is_diff = true;

    cc.setWorldToModelTransform(Eigen::Affine3d::Identity());

    // set planning scene
    if (!cc.setPlanningScene(scene)) {
        ROS_ERROR("Failed to update Collision Checker from Planning Scene");
        return 1;
    }

    SV_SHOW_INFO(grid.getDistanceFieldVisualization(0.2));

    // set the kinematics to planning transform if found in the initial
    // configuration as a multi-dof transform...this is to account for the kdl
    // robot model not generating forward kinematics for the robot as a whole
    const auto &multi_dof_joint_state(scene.robot_state.multi_dof_joint_state);
    if (multi_dof_joint_state.header.frame_id == planning_frame) {
        ROS_INFO("Search for planning -> kinematics transform in multi-dof joint state");
        bool found = false;
        for (size_t i = 0; i < multi_dof_joint_state.joint_names.size(); ++i) {
            const std::string &joint_name(multi_dof_joint_state.joint_names[i]);
            const geometry_msgs::Transform &transform(multi_dof_joint_state.transforms[i]);
            if (joint_name == rm->getKinematicsFrame()) {
                KDL::Frame f;
                tf::transformMsgToKDL(transform, f);
                rm->setKinematicsToPlanningTransform(f, "what?");
                found = true;
                break;
            }
        }
        if (!found) {
            ROS_WARN("You might want to provide the planning frame -> kinematics frame transform in the multi-dof joint state");
        }
    } else {
        ROS_WARN("You might want to provide the planning frame -> kinematics frame transform in the multi-dof joint state");
    }

    SV_SHOW_INFO(cc.getBoundingBoxVisualization());
    auto markers = cc.getCollisionWorldVisualization();
    ROS_INFO("Publish %zu collision world markers", markers.markers.size());
    SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    SV_SHOW_INFO(markers);
    SV_SHOW_INFO(cc.getOccupiedVoxelsVisualization());

    ///////////////////
    // Planner Setup //
    ///////////////////

    PlannerConfig planning_config;
    if (!ReadPlannerConfig(ros::NodeHandle("~planning"), planning_config)) {
        ROS_ERROR("Failed to read planner config");
        return 1;
    }

    smpl::PlannerInterface planner(rm.get(), &cc, &grid);

    smpl::PlanningParams params;
    params.planning_frame = planning_frame;

    params.planning_link_sphere_radius = 0.02;

    params.addParam("discretization", planning_config.discretization);
    params.addParam("mprim_filename", planning_config.mprim_filename);
    params.addParam("use_xyz_snap_mprim", planning_config.use_xyz_snap_mprim);
    params.addParam("use_rpy_snap_mprim", planning_config.use_rpy_snap_mprim);
    params.addParam("use_xyzrpy_snap_mprim", planning_config.use_xyzrpy_snap_mprim);
    params.addParam("use_short_dist_mprims", planning_config.use_short_dist_mprims);
    params.addParam("xyz_snap_dist_thresh", planning_config.xyz_snap_dist_thresh);
    params.addParam("rpy_snap_dist_thresh", planning_config.rpy_snap_dist_thresh);
    params.addParam("xyzrpy_snap_dist_thresh", planning_config.xyzrpy_snap_dist_thresh);
    params.addParam("short_dist_mprims_thresh", planning_config.short_dist_mprims_thresh);
    params.addParam("repair_time", 5.0);

    params.addParam("epsilon", 100.0);

    if (!planner.init(params)) {
        ROS_ERROR("Failed to initialize Planner Interface");
        return 1;
    }

    //////////////
    // Planning //
    //////////////

    std::vector<double> pose(6, 0);
    std::vector<double> goal(6, 0);
    ph.param("goal/x", goal[0], 0.0);
    ph.param("goal/y", goal[1], 0.0);
    ph.param("goal/z", goal[2], 0.0);
    ph.param("goal/roll", goal[3], 0.0);
    ph.param("goal/pitch", goal[4], 0.0);
    ph.param("goal/yaw", goal[5], 0.0);

    moveit_msgs::MotionPlanRequest req;
    moveit_msgs::MotionPlanResponse res;

    req.allowed_planning_time = 60.0;
    req.goal_constraints.resize(1);
    FillGoalConstraint(goal, planning_frame, req.goal_constraints[0]);
    req.group_name = rm_config.group_name;
    req.max_acceleration_scaling_factor = 1.0;
    req.max_velocity_scaling_factor = 1.0;
    req.num_planning_attempts = 1;
//    req.path_constraints;
    req.planner_id = "arastar.bfs.manip";
    req.start_state = scene.robot_state;
//    req.trajectory_constraints;
//    req.workspace_parameters;

    // plan
    ROS_INFO("Calling solve...");
    if (!planner.solve(scene, req, res)) {
        ROS_ERROR("Failed to plan.");
        return 1;
    }

    ///////////////////////////////////
    // Visualizations and Statistics //
    ///////////////////////////////////

    std::map<std::string, double> planning_stats = planner.getPlannerStats();

    ROS_INFO("Planning statistics");
    for (const auto& entry : planning_stats) {
        ROS_INFO("    %s: %0.3f", entry.first.c_str(), entry.second);
    }

    ROS_INFO("Animate path");

    while (ros::ok()) {
        for (const auto &point : res.trajectory.joint_trajectory.points) {
            auto markers = cc.getCollisionRobotVisualization(point.positions);
            SV_SHOW_INFO(markers);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    return 0;
}
