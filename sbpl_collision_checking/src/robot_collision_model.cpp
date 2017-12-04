////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen, Andrew Dornbush
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
/// \author Andrew Dornbush

#include <sbpl_collision_checking/robot_collision_model.h>

// standard includes
#include <assert.h>
#include <stdlib.h>
#include <functional>
#include <stack>
#include <utility>

// system includes
#include <leatherman/print.h>
#include <leatherman/mesh_resource.h>
#include <leatherman/viz.h>
#include <ros/console.h>
#include <smpl/geometry/voxelize.h>
#include <smpl/geometry/bounding_spheres.h>
#include <urdf/model.h>

// project includes
#include <sbpl_collision_checking/robot_collision_state.h>
#include <sbpl_collision_checking/voxel_operations.h>
#include "transform_functions.h"

namespace sbpl {
namespace collision {

static const char* RCM_LOGGER = "robot_model";

RobotCollisionModelPtr RobotCollisionModel::Load(
    const urdf::ModelInterface& urdf,
    const CollisionModelConfig& config)
{
    //std::make_shared<RobotCollisionModel>();
    auto rcm = RobotCollisionModelPtr(new RobotCollisionModel);
    if (!rcm->init(urdf, config)) {
        return RobotCollisionModelPtr();
    }
    else {
        return rcm;
    }
}

RobotCollisionModel::RobotCollisionModel() :
    m_name(),
    m_model_frame(),
    m_jvar_names(),
    m_jvar_continuous(),
    m_jvar_has_position_bounds(),
    m_jvar_min_positions(),
    m_jvar_max_positions(),
    m_jvar_name_to_index(),
    m_joint_origins(),
    m_joint_axes(),
    m_joint_transforms(),
    m_link_names(),
    m_link_name_to_index(),
    m_spheres_models(),
    m_voxels_models(),
    m_group_models(),
    m_group_name_to_index(),
    m_link_spheres_models(),
    m_link_voxels_models()
{
}

RobotCollisionModel::~RobotCollisionModel()
{
}

bool RobotCollisionModel::init(
    const urdf::ModelInterface& urdf,
    const CollisionModelConfig& config)
{
    bool success = true;
    success = success && initRobotModel(urdf, config.world_joint);
    success = success && initCollisionModel(urdf, config);

    if (success) {
        m_config = config;
    }

    return success;
}

bool RobotCollisionModel::initRobotModel(
    const urdf::ModelInterface& urdf,
    const WorldJointConfig& config)
{
    m_name = urdf.getName();

    auto root_link = urdf.getRoot();
    if (!root_link) {
        ROS_ERROR("URDF specifies no root link");
        return false;
    }

    m_model_frame = root_link->name;

    // TODO: depth-first or post-traversal reordering to keep dependent
    // joints/links next to one another

    urdf::Joint world_joint;
    world_joint.child_link_name = root_link->name;
    world_joint.name = config.name;
    world_joint.parent_to_joint_origin_transform.position =
            urdf::Vector3(0.0, 0.0, 0.0);
    world_joint.parent_to_joint_origin_transform.rotation =
            urdf::Rotation(0.0, 0.0, 0.0, 1.0);
    world_joint.axis = urdf::Vector3(0.0, 0.0, 0.0);
    if (config.type == "floating") {
        world_joint.type = urdf::Joint::FLOATING;
    } else if (config.type == "planar") {
        world_joint.type = urdf::Joint::PLANAR;
    } else if (config.type == "fixed") {
        world_joint.type = urdf::Joint::FIXED;
    } else {
        ROS_ERROR("World joint config has invalid type");
        return false;
    }
    addJoint(world_joint);

    m_joint_parent_links.push_back(-1);

    // breadth-first traversal of all links in the robot
    typedef std::pair<boost::shared_ptr<const urdf::Link>, int>
    link_parent_joint_idx_pair;

    std::stack<boost::shared_ptr<const urdf::Link>> links;
    links.push(root_link);

    while (!links.empty()) {
        boost::shared_ptr<const urdf::Link> link;
        link = links.top();
        links.pop();

        m_link_names.push_back(link->name);

        const size_t lidx = m_link_names.size() - 1;
        m_link_name_to_index[m_link_names.back()] = lidx;

        m_link_children_joints.push_back(std::vector<int>());

        if (link->parent_joint) {
            addJoint(*link->parent_joint);
            m_joint_parent_links.push_back(m_link_name_to_index[link->parent_joint->parent_link_name]);
            m_link_parent_joints.push_back(m_joint_names.size() - 1);
        } else {
            m_link_parent_joints.push_back(0);
        }

        // NOTE: iterate in reverse to achieve order as if we had built the
        // tree using recursion
        for (auto it = link->child_joints.rbegin();
            it != link->child_joints.rend(); ++it)
        {
            const auto& joint = *it;

            // NOTE: delay mapping from link/joint to children until we know
            // indices

            // push the child link onto the queue
            auto child_link = urdf.getLink(joint->child_link_name);
            links.push(child_link);
        }
    }

    auto get_jidx = [&](const std::string& joint_name)
    {
        auto it = std::find(m_joint_names.begin(), m_joint_names.end(), joint_name);
        return std::distance(m_joint_names.begin(), it);
    };

    m_desc_joint_matrix.resize(m_joint_names.size() * m_joint_names.size(), false);
    for (const std::string& joint_name : m_joint_names) {
        int jidx = get_jidx(joint_name);
        auto joint = urdf.getJoint(joint_name);

        if (!joint) {
            // skip the world joint, not found in the urdf
            continue;
        }

        // every joint is descendant from the world joint
        m_desc_joint_matrix[jidx * m_joint_names.size()] = true;

        while (joint) {
            // get the parent joint
            auto plink = urdf.getLink(joint->parent_link_name);
            joint = plink->parent_joint;
            if (joint) {
                // set an entry
                int pjidx = get_jidx(joint->name);
                m_desc_joint_matrix[jidx * m_joint_names.size() + pjidx] = true;
            }
        }
    }

    // map joint -> child link
    m_joint_child_links.resize(m_joint_names.size());
    for (size_t lidx = 0; lidx < m_link_names.size(); ++lidx) {
        int pjidx = m_link_parent_joints[lidx];
        m_joint_child_links[pjidx] = lidx;
    }
    // map link -> child links
    for (size_t jidx = 0; jidx < m_joint_names.size(); ++jidx) {
        int plidx = m_joint_parent_links[jidx];
        if (plidx >= 0) {
            m_link_children_joints[plidx].push_back(jidx);
        }
    }

    ROS_DEBUG_NAMED(RCM_LOGGER, "ComputeFixedJointTransform: %p", ComputeFixedJointTransform);
    ROS_DEBUG_NAMED(RCM_LOGGER, "ComputeRevoluteJointTransform: %p", ComputeRevoluteJointTransform);
    ROS_DEBUG_NAMED(RCM_LOGGER, "ComputeContinuousJointTransform: %p", ComputeContinuousJointTransform);
    ROS_DEBUG_NAMED(RCM_LOGGER, "ComputePrismaticJointTransform: %p", ComputePrismaticJointTransform);
    ROS_DEBUG_NAMED(RCM_LOGGER, "ComputePlanarJointTransform: %p", ComputePlanarJointTransform);
    ROS_DEBUG_NAMED(RCM_LOGGER, "ComputeFloatingJointTransform: %p", ComputeFloatingJointTransform);

    ROS_DEBUG_NAMED(RCM_LOGGER, "Robot Model:");
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Name: %s", m_name.c_str());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Model Frame: %s", m_model_frame.c_str());
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Joint Variables:");
    for (size_t vidx = 0; vidx < m_jvar_names.size(); ++vidx) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "    Name: %s, Continuous: %d, Has Position Bounds: %d, Bounds: [ %0.3lg, %0.3lg ]",
                m_jvar_names[vidx].c_str(),
                (int)m_jvar_continuous[vidx],
                (int)m_jvar_has_position_bounds[vidx],
                m_jvar_min_positions[vidx],
                m_jvar_max_positions[vidx]);
    }
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Joints:");
    for (size_t jidx = 0; jidx < m_joint_origins.size(); ++jidx) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "    Origin: %s, Axis: (%0.3f, %0.3f, %0.3f), Transform Function: %p, Parent Link: %s, Child Link: %s, Joint Variables: [%d,%d)",
                AffineToString(m_joint_origins[jidx]).c_str(),
                m_joint_axes[jidx].x(),
                m_joint_axes[jidx].y(),
                m_joint_axes[jidx].z(),
                m_joint_transforms[jidx],
                (m_joint_parent_links[jidx] != -1) ? m_link_names[m_joint_parent_links[jidx]].c_str() : "(null)",
                m_link_names[m_joint_child_links[jidx]].c_str(),
                m_joint_var_indices[jidx].first,
                m_joint_var_indices[jidx].second);
    }
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Links:");
    for (size_t lidx = 0; lidx < m_link_names.size(); ++lidx) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "    Name: %s, Parent Joint: %d, Child Joints: %s, Index: %d",
                m_link_names[lidx].c_str(),
                m_link_parent_joints[lidx],
                to_string(m_link_children_joints[lidx]).c_str(),
                m_link_name_to_index[m_link_names[lidx]]);
    }

    return true;
}

void RobotCollisionModel::addJoint(const urdf::Joint& joint)
{
    m_joint_names.push_back(joint.name);
    m_joint_origins.push_back(
            poseUrdfToEigen(joint.parent_to_joint_origin_transform));
    m_joint_axes.push_back(
            Eigen::Vector3d(joint.axis.x, joint.axis.y, joint.axis.z));

    m_joint_var_indices.emplace_back(m_jvar_names.size(), 0);

    switch (joint.type) {
    case urdf::Joint::FIXED: {
        addFixedJoint(joint);
    }   break;
    case urdf::Joint::REVOLUTE: {
        addRevoluteJoint(joint);
    }   break;
    case urdf::Joint::PRISMATIC: {
        addPrismaticJoint(joint);
    }   break;
    case urdf::Joint::CONTINUOUS: {
        addContinuousJoint(joint);
    }   break;
    case urdf::Joint::PLANAR: {
        addPlanarJoint(joint);
    }   break;
    case urdf::Joint::FLOATING: {
        addFloatingJoint(joint);
    }   break;
    default: {
        ROS_ERROR_NAMED(RCM_LOGGER, "Unknown joint type encountered");
    }   break;
    }

    m_joint_var_indices.back().second = m_jvar_names.size();
}

void RobotCollisionModel::addFixedJoint(const urdf::Joint& joint)
{
    m_joint_transforms.push_back(ComputeFixedJointTransform);
    m_joint_types.push_back(FIXED);
}

void RobotCollisionModel::addRevoluteJoint(const urdf::Joint& joint)
{
    m_jvar_names.push_back(joint.name);
    m_jvar_continuous.push_back(false);
    m_jvar_has_position_bounds.push_back((bool)joint.limits);

    if (joint.safety) {
        m_jvar_min_positions.push_back(joint.safety->soft_lower_limit);
        m_jvar_max_positions.push_back(joint.safety->soft_upper_limit);
    } else {
        m_jvar_min_positions.push_back(joint.limits->lower);
        m_jvar_max_positions.push_back(joint.limits->upper);
    }
    m_jvar_joint_indices.push_back(m_joint_transforms.size());

    m_jvar_name_to_index[joint.name] = m_jvar_names.size() - 1;

    m_joint_types.push_back(REVOLUTE);
    const urdf::Vector3& axis = joint.axis;
    if (axis.x == 1.0 && axis.y == 0.0 && axis.z == 0.0) {
        m_joint_transforms.push_back(ComputeRevoluteJointTransformX);
    } else if (axis.x == 0.0 && axis.y == 1.0 && axis.z == 0.0) {
        m_joint_transforms.push_back(ComputeRevoluteJointTransformY);
    } else if (axis.x == 0.0 && axis.y == 0.0 && axis.z == 1.0) {
        m_joint_transforms.push_back(ComputeRevoluteJointTransformZ);
    } else {
        m_joint_transforms.push_back(ComputeRevoluteJointTransform);
    }
}

void RobotCollisionModel::addPrismaticJoint(const urdf::Joint& joint)
{
    m_jvar_names.push_back(joint.name);
    m_jvar_continuous.push_back(false);
    m_jvar_has_position_bounds.push_back((bool)joint.limits);

    if (joint.safety) {
        m_jvar_min_positions.push_back(joint.safety->soft_lower_limit);
        m_jvar_max_positions.push_back(joint.safety->soft_upper_limit);
    } else {
        m_jvar_min_positions.push_back(joint.limits->lower);
        m_jvar_max_positions.push_back(joint.limits->upper);
    }
    m_jvar_joint_indices.push_back(m_joint_transforms.size());

    m_jvar_name_to_index[joint.name] = m_jvar_names.size() - 1;

    m_joint_types.push_back(PRISMATIC);
    m_joint_transforms.push_back(ComputePrismaticJointTransform);
}

void RobotCollisionModel::addContinuousJoint(const urdf::Joint& joint)
{
    m_jvar_names.push_back(joint.name);
    m_jvar_continuous.push_back(true);
    m_jvar_has_position_bounds.push_back(false);
    m_jvar_min_positions.push_back(-std::numeric_limits<double>::infinity());
    m_jvar_max_positions.push_back(std::numeric_limits<double>::infinity());
    m_jvar_joint_indices.push_back(m_joint_transforms.size());

    m_jvar_name_to_index[joint.name] = m_jvar_names.size() - 1;

    m_joint_types.push_back(CONTINUOUS);
    const urdf::Vector3& axis = joint.axis;
    if (axis.x == 1.0 && axis.y == 0.0 && axis.z == 0.0) {
        m_joint_transforms.push_back(ComputeRevoluteJointTransformX);
    }
    else if (axis.x == 0.0 && axis.y == 1.0 && axis.z == 0.0) {
        m_joint_transforms.push_back(ComputeRevoluteJointTransformY);
    }
    else if (axis.x == 0.0 && axis.y == 0.0 && axis.z == 1.0) {
        m_joint_transforms.push_back(ComputeRevoluteJointTransformZ);
    }
    else {
        m_joint_transforms.push_back(ComputeRevoluteJointTransform);
    }
}

void RobotCollisionModel::addPlanarJoint(const urdf::Joint& joint)
{
    // NOTE: local joint variable names follow moveit conventions
    std::string var_name;

    var_name = joint.name + "/x";
    m_jvar_names.push_back(var_name);
    m_jvar_continuous.push_back(false);
    m_jvar_has_position_bounds.push_back(false);
    m_jvar_min_positions.push_back(-std::numeric_limits<double>::infinity());
    m_jvar_max_positions.push_back(std::numeric_limits<double>::infinity());
    m_jvar_joint_indices.push_back(m_joint_transforms.size());
    m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

    var_name = joint.name + "/y";
    m_jvar_names.push_back(var_name);
    m_jvar_continuous.push_back(false);
    m_jvar_has_position_bounds.push_back(false);
    m_jvar_min_positions.push_back(-std::numeric_limits<double>::infinity());
    m_jvar_max_positions.push_back(std::numeric_limits<double>::infinity());
    m_jvar_joint_indices.push_back(m_joint_transforms.size());
    m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

    var_name = joint.name + "/theta";
    m_jvar_names.push_back(var_name);
    m_jvar_continuous.push_back(true);
    m_jvar_has_position_bounds.push_back(false);
    m_jvar_min_positions.push_back(-std::numeric_limits<double>::infinity());
    m_jvar_max_positions.push_back(std::numeric_limits<double>::infinity());
    m_jvar_joint_indices.push_back(m_joint_transforms.size());
    m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

    m_joint_types.push_back(PLANAR);
    m_joint_transforms.push_back(ComputePlanarJointTransform);
}

void RobotCollisionModel::addFloatingJoint(const urdf::Joint& joint)
{
    // NOTE: local joint variable names follow moveit conventions
    std::string var_name;

    var_name = joint.name + "/trans_x";
    m_jvar_names.push_back(var_name);
    m_jvar_continuous.push_back(false);
    m_jvar_has_position_bounds.push_back(false);
    m_jvar_min_positions.push_back(-std::numeric_limits<double>::infinity());
    m_jvar_max_positions.push_back(std::numeric_limits<double>::infinity());
    m_jvar_joint_indices.push_back(m_joint_transforms.size());
    m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

    var_name = joint.name + "/trans_y";
    m_jvar_names.push_back(var_name);
    m_jvar_continuous.push_back(false);
    m_jvar_has_position_bounds.push_back(false);
    m_jvar_min_positions.push_back(-std::numeric_limits<double>::infinity());
    m_jvar_max_positions.push_back(std::numeric_limits<double>::infinity());
    m_jvar_joint_indices.push_back(m_joint_transforms.size());
    m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

    var_name = joint.name + "/trans_z";
    m_jvar_names.push_back(var_name);
    m_jvar_continuous.push_back(true);
    m_jvar_has_position_bounds.push_back(false);
    m_jvar_min_positions.push_back(-std::numeric_limits<double>::infinity());
    m_jvar_max_positions.push_back(std::numeric_limits<double>::infinity());
    m_jvar_joint_indices.push_back(m_joint_transforms.size());
    m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

    var_name = joint.name + "/rot_x";
    m_jvar_names.push_back(var_name);
    m_jvar_continuous.push_back(false);
    m_jvar_has_position_bounds.push_back(true);
    m_jvar_min_positions.push_back(-1.0);
    m_jvar_max_positions.push_back(1.0);
    m_jvar_joint_indices.push_back(m_joint_transforms.size());
    m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

    var_name = joint.name + "/rot_y";
    m_jvar_names.push_back(var_name);
    m_jvar_continuous.push_back(false);
    m_jvar_has_position_bounds.push_back(true);
    m_jvar_min_positions.push_back(-1.0);
    m_jvar_max_positions.push_back(1.0);
    m_jvar_joint_indices.push_back(m_joint_transforms.size());
    m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

    var_name = joint.name + "/rot_z";
    m_jvar_names.push_back(var_name);
    m_jvar_continuous.push_back(true);
    m_jvar_has_position_bounds.push_back(true);
    m_jvar_min_positions.push_back(-1.0);
    m_jvar_max_positions.push_back(1.0);
    m_jvar_joint_indices.push_back(m_joint_transforms.size());
    m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

    var_name = joint.name + "/rot_w";
    m_jvar_names.push_back(var_name);
    m_jvar_continuous.push_back(true);
    m_jvar_has_position_bounds.push_back(true);
    m_jvar_min_positions.push_back(-1.0);
    m_jvar_max_positions.push_back(1.0);
    m_jvar_joint_indices.push_back(m_joint_transforms.size());
    m_jvar_name_to_index[var_name] = m_jvar_names.size() - 1;

    m_joint_types.push_back(FLOATING);
    m_joint_transforms.push_back(ComputeFloatingJointTransform);
}

bool RobotCollisionModel::initCollisionModel(
    const urdf::ModelInterface& urdf,
    const CollisionModelConfig& config)
{
    if (!checkCollisionModelConfig(config)) {
        return false;
    }

    std::vector<CollisionGroupConfig> expanded_groups;
    if (!expandGroups(config.groups, expanded_groups)) {
        ROS_ERROR("failed to expand groups");
        return false;
    }

    // initialize spheres models
    m_spheres_models.reserve(config.spheres_models.size());
    for (size_t i = 0; i < config.spheres_models.size(); ++i) {
        const CollisionSpheresModelConfig& spheres_config = config.spheres_models[i];

        if (spheres_config.autogenerate) {
            std::vector<CollisionSphereConfig> auto_spheres;
            if (!generateSpheresModel(
                    urdf,
                    spheres_config.link_name,
                    spheres_config.radius,
                    auto_spheres) ||
                auto_spheres.empty())
            {
                continue;
            }
            m_spheres_models.push_back(CollisionSpheresModel());
            m_spheres_models.back().spheres.buildFrom(auto_spheres);
        }
        else {
            if (spheres_config.spheres.empty()) {
                continue;
            }
            m_spheres_models.push_back(CollisionSpheresModel());
            m_spheres_models.back().spheres.buildFrom(spheres_config.spheres);
        }

        CollisionSpheresModel& spheres_model = m_spheres_models.back();
        spheres_model.link_index = linkIndex(spheres_config.link_name);

        for (auto& sphere : spheres_model.spheres.m_tree) {
            sphere.parent = &spheres_model;
        }
    }

    // initialize voxels models
    m_voxels_models.resize(config.voxel_models.size());
    for (size_t i = 0; i < m_voxels_models.size(); ++i) {
        CollisionVoxelsModel& voxels_model = m_voxels_models[i];
        const std::string& link_name = config.voxel_models[i].link_name;
        voxels_model.link_index = linkIndex(link_name);
        voxels_model.voxel_res = config.voxel_models[i].res;
        if (!voxelizeLink(urdf, link_name, voxels_model)) {
            ROS_ERROR_NAMED(RCM_LOGGER, "Failed to voxelize link '%s'", link_name.c_str());
        }
    }

    // initialize groups
    m_group_models.resize(expanded_groups.size());
    for (size_t i = 0; i < m_group_models.size(); ++i) {
        CollisionGroupModel& group_model = m_group_models[i];
        const CollisionGroupConfig& group_config = expanded_groups[i];
        const std::string& group_name = group_config.name;
        group_model.name = group_name;

        for (size_t j = 0; j < group_config.links.size(); ++j) {
            const std::string& link_name = group_config.links[j];
            group_model.link_indices.push_back(linkIndex(link_name));
        }

        m_group_name_to_index[group_name] = i;
    }

    // initialize link spheres models
    m_link_spheres_models.assign(m_link_names.size(), nullptr);
    for (const CollisionSpheresModel& spheres_model : m_spheres_models) {
        m_link_spheres_models[spheres_model.link_index] = &spheres_model;
    }

    // initialize link voxels models
    m_link_voxels_models.assign(m_link_names.size(), nullptr);
    for (const CollisionVoxelsModel& voxels_model : m_voxels_models) {
        m_link_voxels_models[voxels_model.link_index] = &voxels_model;
    }

    assert(checkCollisionModelReferences());

    ROS_DEBUG_NAMED(RCM_LOGGER, "Collision Model:");
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Spheres Models: [%p, %p]", m_spheres_models.data(), m_spheres_models.data() + m_spheres_models.size());
    for (const auto& spheres_model : m_spheres_models) {
        ROS_DEBUG_STREAM_NAMED(RCM_LOGGER, "    link_index: " << spheres_model.link_index << ", spheres: " << spheres_model.spheres.size());
    }
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Voxels Models: [%p, %p]", m_voxels_models.data(), m_voxels_models.data() + m_voxels_models.size());
    for (const auto& voxels_model : m_voxels_models) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "    link_index: %d, voxel_res: %0.3f, voxel count: %zu", voxels_model.link_index, voxels_model.voxel_res, voxels_model.voxels.size());
    }
    ROS_DEBUG_NAMED(RCM_LOGGER, "  Group Models:");
    for (const auto& group_model : m_group_models) {
        ROS_DEBUG_NAMED(RCM_LOGGER, "    name: %s, link_indices: %s", group_model.name.c_str(), to_string(group_model.link_indices).c_str());
    }

    return true;
}

bool RobotCollisionModel::expandGroups(
    const std::vector<CollisionGroupConfig>& groups,
    std::vector<CollisionGroupConfig>& expanded_groups) const
{
    // container for expanded configurations
    std::vector<CollisionGroupConfig> expanded;

    /////////////////////////////////////////////////////////////////////////
    // initialized expanded configurations with explicitly specified links //
    /////////////////////////////////////////////////////////////////////////

    for (const CollisionGroupConfig& g : groups) {
        CollisionGroupConfig config;
        config.name = g.name;
        config.links = g.links;
        expanded.push_back(std::move(config));
    }

    ///////////////////
    // expand chains //
    ///////////////////

    for (size_t gidx = 0; gidx < groups.size(); ++gidx) {
        const CollisionGroupConfig& g = groups[gidx];
        for (const auto& chain : g.chains) {
            const std::string& base = std::get<0>(chain);
            const std::string& tip = std::get<1>(chain);

            std::vector<std::string> chain_links;

            std::string link_name = tip;
            chain_links.push_back(link_name);
            while (link_name != base) {
                if (!hasLink(link_name)) {
                    ROS_ERROR_NAMED(RCM_LOGGER, "link '%s' not found in the robot model", link_name.c_str());
                    return false;
                }

                int lidx = linkIndex(link_name);
                int pjidx = linkParentJointIndex(lidx);
                int plidx = jointParentLinkIndex(pjidx);

                if (plidx < 0) {
                    ROS_ERROR_NAMED(RCM_LOGGER, "(base: %s, tip: %s) is not a chain in the robot model", base.c_str(), tip.c_str());
                    return false;
                }

                link_name = linkName(plidx);
                chain_links.push_back(link_name);
            }

            CollisionGroupConfig& eg = expanded[gidx];
            eg.links.insert(eg.links.end(), chain_links.begin(), chain_links.end());
        }
    }

    //////////////////////
    // expand subgroups //
    //////////////////////

    auto is_named = [](const CollisionGroupConfig& g, const std::string& name) { return g.name == name; };

    auto name_to_index = [&](const std::string& name) {
        auto git = std::find_if(groups.begin(), groups.end(),
                [&](const CollisionGroupConfig& g) { return g.name == name; });
        return std::distance(groups.begin(), git);
    };

    // ad-hoc graph mapping groups to their dependencies, reverse dependencies,
    // and the number of groups they're waiting to be computed
    std::vector<std::vector<size_t>> deps(groups.size());
    std::vector<std::vector<size_t>> rdeps(groups.size());
    std::vector<size_t> waiting(groups.size(), 0);
    for (size_t gidx = 0; gidx < groups.size(); ++gidx) {
        const CollisionGroupConfig& group = groups[gidx];
        for (const std::string& dep : group.groups) {
            size_t didx = name_to_index(dep);
            deps[gidx].push_back(didx);
            rdeps[didx].push_back(gidx);
        }
        waiting[gidx] = group.groups.size();
    }


    std::vector<size_t> q(groups.size()); // start with all groups
    size_t n = 0;
    std::generate(q.begin(), q.end(), [&n]() { return n++; });

    while (!q.empty()) { // unexpanded groups remaining
        // find a config whose dependencies are finished expanding their config
        auto git = std::find_if(q.begin(), q.end(), [&](size_t i) { return waiting[i] == 0; });
        if (git == q.end()) {
            ROS_ERROR("cycle in group config");
            return false;
        }

        size_t gidx = *git;
        --waiting[gidx]; // -1 -> done
        q.erase(git);

        auto eit = std::find_if(expanded.begin(), expanded.end(), std::bind(is_named, std::placeholders::_1, groups[gidx].name));
        assert(eit != expanded.end());

        for (size_t didx : deps[gidx]) {
            // find the existing expanded config for the group dependency
            const std::string& dep = groups[didx].name;
            auto same_name = std::bind(is_named, std::placeholders::_1, dep);
            auto ggit = std::find_if(expanded.begin(), expanded.end(), same_name);
            assert(ggit != expanded.end());

            // merge expanded config
            eit->links.insert(eit->links.end(), ggit->links.begin(), ggit->links.end());
        }

        ROS_DEBUG_NAMED(RCM_LOGGER, "Group '%s' contains %zu links", eit->name.c_str(), eit->links.size());

        // notify reverse dependencies that we're finished
        for (size_t rdidx : rdeps[gidx]) {
            --waiting[rdidx];
        }
    }

    // remove any duplicates
    for (CollisionGroupConfig& eg : expanded) {
        sort(eg.links.begin(), eg.links.end());
        auto uit = std::unique(eg.links.begin(), eg.links.end());
        eg.links.erase(uit, eg.links.end());
    }

    expanded_groups = std::move(expanded);
    return true;
}

bool RobotCollisionModel::generateSpheresModel(
    const urdf::ModelInterface& urdf,
    const std::string& link_name,
    double radius,
    std::vector<CollisionSphereConfig>& spheres) const
{
    auto link = urdf.getLink(link_name);

    if (!link) {
        ROS_ERROR_NAMED(RCM_LOGGER, "Failed to find link '%s' in the URDF", link_name.c_str());
        return false;
    }

    if (!link->collision && link->collision_array.empty()) {
        ROS_WARN_NAMED(RCM_LOGGER, "Failed to find collision elements of link '%s'", link->name.c_str());
        return true;
    }

    if (!link->collision_array.empty()) {
        for (auto collision : link->collision_array) {
            if (!generateBoundingSpheres(*collision, radius, spheres)) {
                ROS_ERROR_NAMED(RCM_LOGGER, "Failed to sphere bound collision element for link '%s'", link_name.c_str());
                return false;
            }
        }
    } else if (link->collision) {
        if (!generateBoundingSpheres(*link->collision, radius, spheres)) {
            ROS_ERROR_NAMED(RCM_LOGGER, "Failed to sphere bound collision element for link '%s'", link_name.c_str());
            return false;
        }
    } else {
        ROS_ERROR_NAMED(RCM_LOGGER, "Hmm");
        return false;
    }

    if (spheres.empty()) {
        ROS_WARN_NAMED(RCM_LOGGER, "Voxelizing collision elements for link '%s' produced 0 voxels", link_name.c_str());
    }

    return true;
}

bool RobotCollisionModel::generateBoundingSpheres(
    const urdf::Collision& collision,
    double radius,
    std::vector<CollisionSphereConfig>& spheres) const
{
    auto geom = collision.geometry;

    if (!geom) {
        ROS_ERROR_NAMED(RCM_LOGGER, "Failed to find geometry for collision element");
        return false;
    }

    Eigen::Translation3d translation(
            collision.origin.position.x,
            collision.origin.position.y,
            collision.origin.position.z);
    Eigen::Quaterniond rotation;
    collision.origin.rotation.getQuaternion(
            rotation.x(), rotation.y(), rotation.z(), rotation.w());

    Eigen::Affine3d pose = translation * rotation;

    return generateBoundingSpheres(*geom, pose, radius, spheres);
}

bool RobotCollisionModel::generateBoundingSpheres(
    const urdf::Geometry& geom,
    const Eigen::Affine3d& pose,
    double radius,
    std::vector<CollisionSphereConfig>& spheres) const
{
    std::vector<Eigen::Vector3d> centers;
    if (geom.type == urdf::Geometry::MESH) {
        std::vector<Eigen::Vector3d> vertices;
        std::vector<int> triangles;
        urdf::Mesh* mesh = (urdf::Mesh*)&geom;
        if (!leatherman::getMeshComponentsFromResource(
                mesh->filename, Eigen::Vector3d::Ones(), triangles, vertices))
        {
            ROS_ERROR_NAMED(RCM_LOGGER, "Failed to get mesh from file. (%s)", mesh->filename.c_str());
            return false;
        }

        ROS_DEBUG_NAMED(RCM_LOGGER, "mesh: %s  triangles: %zu  vertices: %zu", mesh->filename.c_str(), triangles.size(), vertices.size());

        geometry::ComputeMeshBoundingSpheres(vertices, triangles, radius, centers);
        ROS_DEBUG_NAMED(RCM_LOGGER, " -> centers: %zu", centers.size());
    }
    else if (geom.type == urdf::Geometry::BOX) {
        urdf::Box* box = (urdf::Box*)&geom;
        ROS_DEBUG_NAMED(RCM_LOGGER, "box: { dims: %0.3f, %0.3f, %0.3f }", box->dim.x, box->dim.y, box->dim.z);
        geometry::ComputeBoxBoundingSpheres(box->dim.x, box->dim.y, box->dim.z, radius, centers);
        ROS_DEBUG_NAMED(RCM_LOGGER, " -> centers: %zu", centers.size());
    }
    else if (geom.type == urdf::Geometry::CYLINDER) {
        urdf::Cylinder* cyl = (urdf::Cylinder*)&geom;
        ROS_DEBUG_NAMED(RCM_LOGGER, "cylinder: { radius: %0.3f, length: %0.3f }", cyl->radius, cyl->length);
        geometry::ComputeCylinderBoundingSpheres(cyl->radius, cyl->length, radius, centers);
        ROS_DEBUG_NAMED(RCM_LOGGER, " -> centers: %zu", centers.size());
    }
    else if (geom.type == urdf::Geometry::SPHERE) {
        urdf::Sphere* sph = (urdf::Sphere*)&geom;
        ROS_DEBUG_NAMED(RCM_LOGGER, "sphere: { radius: %0.3f }", sph->radius);
        geometry::ComputeSphereBoundingSpheres(sph->radius, radius, centers);
        ROS_DEBUG_NAMED(RCM_LOGGER, " -> centers: %zu", centers.size());
    }
    else {
        ROS_ERROR_NAMED(RCM_LOGGER, "Unrecognized geometry type for voxelization");
        return false;
    }

    for (auto& center : centers) {
        center = pose * center;
        CollisionSphereConfig config;
        config.x = center.x();
        config.y = center.y();
        config.z = center.z();
        config.radius = radius;
        config.priority = 1;
        spheres.push_back(config);
    }
    ROS_INFO_NAMED(RCM_LOGGER, "Autogenerated %zu spheres for geometry", centers.size());

    return true;
}

bool RobotCollisionModel::checkCollisionModelConfig(
    const CollisionModelConfig& config)
{
    // TODO:: report the more fine-grained sources of errors

    for (const auto& spheres_config : config.spheres_models) {
        if (!hasLink(spheres_config.link_name)) {
            ROS_ERROR("No link '%s' found in robot model", spheres_config.link_name.c_str());
            return false;
        }
    }

    for (const auto& voxels_config : config.voxel_models) {
        if (!hasLink(voxels_config.link_name)) {
            ROS_ERROR("No link '%s' found in robot model", voxels_config.link_name.c_str());
            return false;
        }
    }

    for (const auto& group_config : config.groups) {
        for (const std::string& link_name : group_config.links) {
            if (!hasLink(link_name)) {
                ROS_ERROR("No link '%s' found in robot model", link_name.c_str());
                return false;
            }
        }
    }

    return true;
}

bool RobotCollisionModel::checkCollisionModelReferences() const
{
    for (const auto& spheres_model : m_spheres_models) {
        if (spheres_model.link_index != -1 &&
            (
                spheres_model.link_index < 0 ||
                spheres_model.link_index >= m_link_names.size()
            ))
        {
            return false;
        }
    }

    for (const auto& voxels_model : m_voxels_models) {
        if (voxels_model.link_index != -1 &&
            (
                voxels_model.link_index < 0 ||
                voxels_model.link_index >= m_link_names.size()
            ))
        {
            return false;
        }
    }

    for (const auto& group_model : m_group_models) {
        for (int lidx : group_model.link_indices) {
            if (lidx < 0 || lidx >= m_link_names.size()) {
                return false;
            }
        }
    }

    return true;
}

Eigen::Affine3d RobotCollisionModel::poseUrdfToEigen(const urdf::Pose& p) const
{
    return Eigen::Translation3d(p.position.x, p.position.y, p.position.z) *
            Eigen::Quaterniond(
                    p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z);
}

bool RobotCollisionModel::voxelizeLink(
    const urdf::ModelInterface& urdf,
    const std::string& link_name,
    CollisionVoxelsModel& model) const
{
    auto link = urdf.getLink(link_name);

    if (!link) {
        ROS_ERROR_NAMED(RCM_LOGGER, "Failed to find link '%s' in the URDF", link_name.c_str());
        return false;
    }

    if (!link->collision && link->collision_array.empty()) {
        ROS_WARN_NAMED(RCM_LOGGER, "Failed to find collision elements of link '%s'", link->name.c_str());
        return true;
    }

    if (link->collision) {
        if (!voxelizeCollisionElement(
            *link->collision, model.voxel_res, model.voxels))
        {
            ROS_ERROR_NAMED(RCM_LOGGER, "Failed to voxelize collision element for link '%s'", link_name.c_str());
            return false;
        }
    }

    if (!link->collision_array.empty()) {
        for (auto collision : link->collision_array) {
            if (!voxelizeCollisionElement(
                    *collision, model.voxel_res, model.voxels))
            {
                ROS_ERROR_NAMED(RCM_LOGGER, "Failed to voxelize collision element for link '%s'", link_name.c_str());
                return false;
            }
        }
    }

    if (model.voxels.empty()) {
        ROS_WARN_NAMED(RCM_LOGGER, "Voxelizing collision elements for link '%s' produced 0 voxels", link_name.c_str());
    }

    return true;
}

bool RobotCollisionModel::voxelizeCollisionElement(
    const urdf::Collision& collision,
    double res,
    std::vector<Eigen::Vector3d>& voxels) const
{
    auto geom = collision.geometry;

    if (!geom) {
        ROS_ERROR_NAMED(RCM_LOGGER, "Failed to find geometry for collision element");
        return false;
    }

    Eigen::Translation3d translation(
            collision.origin.position.x,
            collision.origin.position.y,
            collision.origin.position.z);
    Eigen::Quaterniond rotation;
    collision.origin.rotation.getQuaternion(
            rotation.x(), rotation.y(), rotation.z(), rotation.w());
    Eigen::Affine3d pose = translation * rotation;

    return voxelizeGeometry(*geom, pose, res, voxels);
}

bool RobotCollisionModel::voxelizeGeometry(
    const urdf::Geometry& geom,
    const Eigen::Affine3d& pose,
    double res,
    std::vector<Eigen::Vector3d>& voxels) const
{
    if (geom.type == urdf::Geometry::MESH) {
        std::vector<Eigen::Vector3d> vertices;
        std::vector<int> triangles;
        urdf::Mesh* mesh = (urdf::Mesh*)&geom;
        if (!leatherman::getMeshComponentsFromResource(
                mesh->filename, Eigen::Vector3d::Ones(), triangles, vertices))
        {
            ROS_ERROR_NAMED(RCM_LOGGER, "Failed to get mesh from file. (%s)", mesh->filename.c_str());
            return false;
        }

        ROS_DEBUG_NAMED(RCM_LOGGER, "mesh: %s  triangles: %zu  vertices: %zu", mesh->filename.c_str(), triangles.size(), vertices.size());

        geometry::VoxelizeMesh(vertices, triangles, pose, res, voxels, false);
        ROS_DEBUG_NAMED(RCM_LOGGER, " -> voxels: %zu", voxels.size());
    }
    else if (geom.type == urdf::Geometry::BOX) {
        urdf::Box* box = (urdf::Box*)&geom;
        ROS_DEBUG_NAMED(RCM_LOGGER, "box: { dims: %0.3f, %0.3f, %0.3f }", box->dim.x, box->dim.y, box->dim.z);
        geometry::VoxelizeBox(box->dim.x, box->dim.y, box->dim.z, pose, res, voxels, false);
        ROS_DEBUG_NAMED(RCM_LOGGER, " -> voxels: %zu", voxels.size());
    }
    else if (geom.type == urdf::Geometry::CYLINDER) {
        urdf::Cylinder* cyl = (urdf::Cylinder*)&geom;
        ROS_DEBUG_NAMED(RCM_LOGGER, "cylinder: { radius: %0.3f, length: %0.3f }", cyl->radius, cyl->length);
        geometry::VoxelizeCylinder(cyl->radius, cyl->length, pose, res, voxels, false);
        ROS_DEBUG_NAMED(RCM_LOGGER, " -> voxels: %zu", voxels.size());
    }
    else if (geom.type == urdf::Geometry::SPHERE) {
        urdf::Sphere* sph = (urdf::Sphere*)&geom;
        ROS_DEBUG_NAMED(RCM_LOGGER, "sphere: { radius: %0.3f }", sph->radius);
        geometry::VoxelizeSphere(sph->radius, pose, res, voxels, false);
        ROS_DEBUG_NAMED(RCM_LOGGER, " -> voxels: %zu", voxels.size());
    }
    else {
        ROS_ERROR_NAMED(RCM_LOGGER, "Unrecognized geometry type for voxelization");
        return false;
    }

    return true;
}

} // namespace collision
} // namespace sbpl
