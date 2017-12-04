////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

/// \author Andrew Dornbush

// standard includes
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>

// system includes
#include <ros/ros.h>
#include <smpl/distance_map/distance_map.h>
#include <smpl/distance_map/sparse_distance_map.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/occupancy_grid.h>
#include <sbpl_collision_checking/collision_space.h>
#include <urdf/model.h>

auto CreateGrid(const ros::NodeHandle& nh, double max_dist)
    -> std::unique_ptr<sbpl::OccupancyGrid>
{
    const char* world_collision_model_param = "world_collision_model";
    std::string wcm_key;
    if (!nh.searchParam(world_collision_model_param, wcm_key)) {
        ROS_ERROR("Failed to find 'world_collision_model' key on the param server");
        return nullptr;
    }

    XmlRpc::XmlRpcValue wcm_config;
    if (!nh.getParam(wcm_key, wcm_config)) {
        ROS_ERROR("Failed to retrieve '%s' from the param server", wcm_key.c_str());
        return nullptr;
    }

    if (wcm_config.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !wcm_config.hasMember("frame_id") ||
        !wcm_config.hasMember("size_x") ||
        !wcm_config.hasMember("size_y") ||
        !wcm_config.hasMember("size_z") ||
        !wcm_config.hasMember("origin_x") ||
        !wcm_config.hasMember("origin_y") ||
        !wcm_config.hasMember("origin_z") ||
        !wcm_config.hasMember("res_m") ||
        !wcm_config.hasMember("max_distance_m"))
    {
        ROS_ERROR("'%s' param is malformed", world_collision_model_param);
        ROS_ERROR_STREAM("has frame_id member " << wcm_config.hasMember("frame_id"));
        ROS_ERROR_STREAM("has size_x member " << wcm_config.hasMember("size_x"));
        ROS_ERROR_STREAM("has size_y member " << wcm_config.hasMember("size_y"));
        ROS_ERROR_STREAM("has size_z member " << wcm_config.hasMember("size_z"));
        ROS_ERROR_STREAM("has origin_x member " << wcm_config.hasMember("origin_x"));
        ROS_ERROR_STREAM("has origin_y member " << wcm_config.hasMember("origin_y"));
        ROS_ERROR_STREAM("has origin_z member " << wcm_config.hasMember("origin_z"));
        ROS_ERROR_STREAM("has res_m member " << wcm_config.hasMember("res_m"));
        ROS_ERROR_STREAM("has max_distance_m member " << wcm_config.hasMember("max_distance_m"));
        return nullptr;
    }

    const std::string world_frame = wcm_config["frame_id"];
    const double size_x = wcm_config["size_x"];
    const double size_y = wcm_config["size_y"];
    const double size_z = wcm_config["size_z"];
    const double res_m = wcm_config["res_m"];
    const double origin_x = wcm_config["origin_x"];
    const double origin_y = wcm_config["origin_y"];
    const double origin_z = wcm_config["origin_z"];
    const double cfg_max_dist = wcm_config["max_distance_m"];
    const double max_distance_m = std::max(max_dist + sqrt(3.0) * res_m, cfg_max_dist);
    const bool ref_counted = true;

    ROS_INFO("Occupancy Grid:");
    ROS_INFO("  origin: (%0.3f, %0.3f, %0.3f)", origin_x, origin_y, origin_z);
    ROS_INFO("  size: (%0.3f, %0.3f, %0.3f)", size_x, size_y, size_z);
    ROS_INFO("  res: %0.3f", res_m);
    ROS_INFO("  max_distance: %0.3f", max_distance_m);

    const int dflib = 2; // 0 -> my dense, 1 -> my sparse, 2 -> df

    std::unique_ptr<sbpl::OccupancyGrid> grid;
    if (dflib == 0) {
        auto df = std::make_shared<sbpl::EuclidDistanceMap>(
                origin_x, origin_y, origin_z,
                size_x, size_y, size_z,
                res_m,
                max_distance_m);

        grid = std::unique_ptr<sbpl::OccupancyGrid>(
                new sbpl::OccupancyGrid(df, ref_counted));

    } else if (dflib == 1){
        auto df = std::make_shared<sbpl::SparseDistanceMap>(
                origin_x, origin_y, origin_z,
                size_x, size_y, size_z,
                res_m,
                max_distance_m);
        grid = std::unique_ptr<sbpl::OccupancyGrid>(
                new sbpl::OccupancyGrid(df, ref_counted));
    } else {
        grid = std::unique_ptr<sbpl::OccupancyGrid>(
                new sbpl::OccupancyGrid(
                        size_x, size_y, size_z,
                        res_m,
                        origin_x, origin_y, origin_z,
                        max_distance_m,
                        ref_counted));
    }

    grid->setReferenceFrame(world_frame);
    return grid;
}

class CollisionSpaceProfiler
{
public:

    bool init();

    struct ProfileResults
    {
        int check_count;
    };

    ProfileResults profileCollisionChecks(double time_limit);
    ProfileResults profileDistanceChecks(double time_limit);
    int exportCheckedStates(const char* filename, int count);
    int verifyCheckedStates(const char* filename);

private:

    ros::NodeHandle m_nh;
    std::unique_ptr<sbpl::OccupancyGrid> m_grid;
    std::vector<std::string> m_planning_joints;
    sbpl::collision::RobotCollisionModelPtr m_rcm;
    sbpl::collision::CollisionSpace m_cspace;
    std::default_random_engine m_rng;
    ros::Publisher m_pub;

    std::vector<double> createRandomState();

    void initACM(sbpl::collision::AllowedCollisionMatrix& acm);
};

bool CollisionSpaceProfiler::init()
{
    std::string robot_description_key;
    if (!m_nh.searchParam("robot_description", robot_description_key)) {
        ROS_ERROR("Failed to find 'robot_description' key on the param server");
        return false;
    }

    urdf::Model urdf;
    if (!urdf.initParam(robot_description_key)) {
        ROS_ERROR("Failed to initialize URDF from parameter '%s'", robot_description_key.c_str());
        return false;
    }

    if (urdf.getName() != "pr2") {
        ROS_ERROR("This benchmark is intended for the PR2 robot");
        return false;
    }

    sbpl::collision::CollisionModelConfig config;
    if (!sbpl::collision::CollisionModelConfig::Load(m_nh, config)) {
        ROS_ERROR("Failed to load collision model config");
        return false;
    }

    m_rcm = sbpl::collision::RobotCollisionModel::Load(urdf, config);

    ROS_INFO("max leaf sphere radius: %0.3f", m_rcm->maxLeafSphereRadius());

    m_grid = CreateGrid(m_nh, m_rcm->maxSphereRadius());

    const std::string group_name = "right_arm";

    // hardcoded joint names corresponding to 'right_arm' joint group from SRDF
    m_planning_joints =
    {
        "r_shoulder_pan_joint",
        "r_shoulder_lift_joint",
        "r_upper_arm_roll_joint",
        "r_elbow_flex_joint",
        "r_forearm_roll_joint",
        "r_wrist_flex_joint",
        "r_wrist_roll_joint",
    };

    if (!m_cspace.init(m_grid.get(), m_rcm, group_name, m_planning_joints)) {
        ROS_ERROR("Failed to initialize Collision Space");
        return false;
    }

    sbpl::collision::AllowedCollisionMatrix acm;
    initACM(acm);
    m_cspace.setAllowedCollisionMatrix(acm);

    m_pub = m_nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 100);

    return true;
}

CollisionSpaceProfiler::ProfileResults
CollisionSpaceProfiler::profileCollisionChecks(double time_limit)
{
    ROS_INFO("Evaluating %0.3f seconds of collision checks", time_limit);

    ROS_INFO("Begin collision check benchmarking");

    int check_count = 0;
    double elapsed = 0.0;
    while (ros::ok() && elapsed < time_limit) {
        auto variables = createRandomState();
        auto start = std::chrono::high_resolution_clock::now();
        double dist;
        bool res = m_cspace.checkCollision(variables, dist);
        auto finish = std::chrono::high_resolution_clock::now();
        elapsed += std::chrono::duration<double>(finish - start).count();
        ++check_count;
    }

    ProfileResults res;
    res.check_count = check_count;
    return res;
}

CollisionSpaceProfiler::ProfileResults
CollisionSpaceProfiler::profileDistanceChecks(double time_limit)
{
    ROS_INFO("Evaluating %0.3f seconds of distance checks", time_limit);

    ROS_INFO("Begin distance check benchmarking");

    int check_count = 0;
    double elapsed = 0.0;
    while (ros::ok() && elapsed < time_limit) {
        auto variables = createRandomState();
        auto start = std::chrono::high_resolution_clock::now();
        double dist = m_cspace.collisionDistance(variables);
        auto finish = std::chrono::high_resolution_clock::now();
        elapsed += std::chrono::duration<double>(finish - start).count();
        ++check_count;
    }

    ProfileResults res;
    res.check_count = check_count;
    return res;
}

std::vector<double> CollisionSpaceProfiler::createRandomState()
{
    std::vector<double> out;
    out.reserve(m_planning_joints.size());
    for (const std::string& var_name : m_planning_joints) {
        if (m_rcm->jointVarIsContinuous(var_name)) {
            std::uniform_real_distribution<double> dist(-M_PI, M_PI);
            out.push_back(dist(m_rng));

        }
        else if (!m_rcm->jointVarHasPositionBounds(var_name)) {
            std::uniform_real_distribution<double> dist;
            out.push_back(dist(m_rng));
        }
        else {
            std::uniform_real_distribution<double> dist(
                    m_rcm->jointVarMinPosition(var_name),
                    m_rcm->jointVarMaxPosition(var_name));
            out.push_back(dist(m_rng));
        }
    }
    return out;
}

void CollisionSpaceProfiler::initACM(sbpl::collision::AllowedCollisionMatrix& acm)
{
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
}

int CollisionSpaceProfiler::exportCheckedStates(const char* filename, int count)
{
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        return 1;
    }

    for (int i = 0; i < count && ros::ok(); ++i) {
        auto variables = createRandomState();
        double dist;
        bool res = m_cspace.checkCollision(variables, dist);
        m_pub.publish(m_cspace.getCollisionRobotVisualization(variables));
        for (auto v : variables) {
            ofs << std::setprecision(12) << v << ' ';
        }
        ofs << (int)res << '\n';
    }

    return 0;
}

int CollisionSpaceProfiler::verifyCheckedStates(const char* filename)
{
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        ROS_ERROR("Failed to open %s", filename);
        return 1;
    }

    std::vector<double> vals(7);
    int ires;
    int line_count = 0;
    int diff = 0;
    int now_positive = 0;
    int now_negative = 0;
    while (ifs >> vals[0] >> vals[1] >> vals[2] >>vals[3] >> vals[4] >> vals[5] >> vals[6] >> ires) {
        ++line_count;
        double dist;
        bool res = m_cspace.checkCollision(vals, dist);
        if ((int)res != ires) {
            ROS_ERROR("Different result for line %d", line_count);
            ++diff;
            if (res) {
                ++now_positive;
            } else {
                ++now_negative;
            }
        }
    }

    if (diff) {
        ROS_ERROR("checks for %d configurations differ (%d positive, %d negative)", diff, now_positive, now_negative);
    }

    return diff;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "benchmark");
    ros::NodeHandle nh;

    if (argc < 2) {
        ROS_INFO("Usage: benchmark <command> <args...>");
        return 1;
    }

    const char* cmd = argv[1];

    CollisionSpaceProfiler prof;
    if (!prof.init()) {
        ROS_ERROR("Failed to initialize profiler");
        return 1;
    }

    if (0 == strcmp(cmd, "export")) {
        const char* filename = argc > 2 ? argv[2] : "checks.csv";
        int count = argc > 3 ? atoi(argv[3]) : 100000;
        return prof.exportCheckedStates(filename, count);
    }
    else if (0 == strcmp(cmd, "verify")) {
        if (argc < 3) {
            ROS_ERROR("Usage: benchmark verify <filename>");
            return 1;
        }
        const char* filename = argv[2];
        return prof.verifyCheckedStates(filename);
    }
    else if (0 == strcmp(cmd, "profile")) {
        const double time_limit = argc > 2 ? std::atof(argv[2]) : 10.0;
        if (time_limit == 0.0) {
            ROS_WARN("Did you make a mistake?");
        }

        {
            auto res = prof.profileCollisionChecks(time_limit);
            ROS_INFO("check count: %d", res.check_count);
            ROS_INFO("checks / second: %g", res.check_count / time_limit);
            ROS_INFO("seconds / check: %g", time_limit / res.check_count);
        }
        {
            auto res = prof.profileDistanceChecks(time_limit);
            ROS_INFO("check count: %d", res.check_count);
            ROS_INFO("checks / second: %g", res.check_count / time_limit);
            ROS_INFO("seconds / check: %g", time_limit / res.check_count);
        }
    } else if (0 == strcmp(cmd, "load")) {
        return 0;
    }

    return 0;
}
