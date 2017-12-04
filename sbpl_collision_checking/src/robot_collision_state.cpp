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

#include <sbpl_collision_checking/robot_collision_state.h>

// standard includes
#include <assert.h>
#include <stdlib.h>
#include <algorithm>
#include <functional>
#include <memory>
#include <utility>

// system includes
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <leatherman/viz.h>

// project includes
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/debug.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include "transform_functions.h"
#include <sbpl_collision_checking/voxel_operations.h>

namespace sbpl {
namespace collision {

bool RobotCollisionState::setJointVarPosition(int vidx, double position)
{
    ASSERT_VECTOR_RANGE(m_jvar_positions, vidx);

    if (m_jvar_positions[vidx] != position) {
        ROS_DEBUG_NAMED(RCS_LOGGER, "Setting joint position of joint %d to %0.3f", vidx, position);

        m_jvar_positions[vidx] = position;

        const int jidx = m_model->jointVarJointIndex(vidx);

        m_dirty_joint_transforms[jidx] = true;

        // TODO: cache affected link transforms in a per-joint array?

        std::vector<int>& q = m_q;
        q.clear();
        q.push_back(m_model->jointChildLinkIndex(jidx));
        while (!q.empty()) {
            int lidx = q.back();
            q.pop_back();

            ROS_DEBUG_NAMED(RCS_LOGGER, "Dirtying transform to link '%s'", m_model->linkName(lidx).c_str());

            // dirty the transform of the affected link
            m_dirty_link_transforms[lidx] = true;

            // dirty the voxels states of any attached voxels model
            CollisionVoxelsState* voxels_state = m_link_voxels_states[lidx];
            if (voxels_state) {
                int dvsidx = std::distance(m_voxels_states.data(), voxels_state);
                m_dirty_voxels_states[dvsidx] = true;
            }

            // add child links to the queue
            for (int cjidx : m_model->linkChildJointIndices(lidx)) {
                q.push_back(m_model->jointChildLinkIndex(cjidx));
            }
        }

        return true;
    }
    else {
        return false;
    }
}

bool RobotCollisionState::setJointVarPositions(const double* positions)
{
    std::vector<int>& ancestors = m_ancestors;
    ancestors.clear();
    for (size_t vidx = 0; vidx < m_jvar_positions.size(); ++vidx) {
        if (m_jvar_positions[vidx] != positions[vidx]) {
            m_jvar_positions[vidx] = positions[vidx];
            const int jidx = m_model->jointVarJointIndex(vidx);
            m_dirty_joint_transforms[jidx] = true;
            bool add = true;
            for (int& ancestor : ancestors) {
                if (m_model->isDescendantJoint(jidx, ancestor)) {
                    // only keep most ancestral joints
                    add = false;
                    break;
                }
                else if (m_model->isDescendantJoint(ancestor, jidx)) {
                    // replace the ancestor joint with this joint
                    add = false;
                    ancestor = jidx;
                    break;
                }
            }
            if (add) {
                ancestors.push_back(jidx);
            }
        }
    }

    if (ancestors.empty()) {
        return false;
    }

    std::vector<int>& q = m_q;
    q.clear();
    for (int ancestor : ancestors) {
        q.push_back(m_model->jointChildLinkIndex(ancestor));
    }
    while (!q.empty()) {
        int lidx = q.back();
        q.pop_back();

        ROS_DEBUG_NAMED(RCS_LOGGER, "Dirtying transform to link '%s'", m_model->linkName(lidx).c_str());

        // dirty the transform of the affected link
        m_dirty_link_transforms[lidx] = true;

        // dirty the voxels states of any attached voxels model
        CollisionVoxelsState* voxels_state = m_link_voxels_states[lidx];
        if (voxels_state) {
            int dvsidx = std::distance(m_voxels_states.data(), voxels_state);
            m_dirty_voxels_states[dvsidx] = true;
        }

        // add child links to the queue
        for (int cjidx : m_model->linkChildJointIndices(lidx)) {
            q.push_back(m_model->jointChildLinkIndex(cjidx));
        }
    }

    return true;
}

visualization_msgs::MarkerArray
RobotCollisionState::getVisualization(int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    const CollisionGroupState& group_state = m_group_states[gidx];

    std::vector<std::vector<double>> spheres;
    std::vector<double> rad;

    size_t sphere_count = 0;
    for (int ssidx : group_state.spheres_indices) {
        const CollisionSpheresState& spheres_state = m_spheres_states[ssidx];
        sphere_count += spheres_state.spheres.size();
    }

    spheres.reserve(sphere_count);
    rad.reserve(sphere_count);

    for (int ssidx : group_state.spheres_indices) {
        const CollisionSpheresState& spheres_state = m_spheres_states[ssidx];
        for (const CollisionSphereState& sphere_state : spheres_state.spheres) {
            if (!sphere_state.isLeaf()) {
                continue;
            }
            std::vector<double> sphere(4, 0.0);
            sphere[0] = sphere_state.pos.x();
            sphere[1] = sphere_state.pos.y();
            sphere[2] = sphere_state.pos.z();
            sphere[3] = sphere_state.model->radius;
            spheres.push_back(std::move(sphere));
            rad.push_back(sphere_state.model->radius);
        }
    }

    const int hue = 90;
    return viz::getSpheresMarkerArray(
            spheres, rad, hue, "", "collision_model", 0);
}

void RobotCollisionState::initRobotState()
{
    // NOTE: need to initialize this before determining per-joint offsets below
    m_jvar_positions.assign(m_model->jointVarCount(), 0.0);
    for (size_t vidx = 0; vidx < m_model->jointVarCount(); ++vidx) {
        if (m_model->jointVarHasPositionBounds(vidx)) {
            if (m_model->jointVarMinPosition(vidx) > 0.0 ||
                m_model->jointVarMaxPosition(vidx) < 0.0)
            {
                m_jvar_positions[vidx] = 0.5 *
                        (
                            m_model->jointVarMinPosition(vidx) +
                            m_model->jointVarMaxPosition(vidx)
                        );
                ROS_DEBUG_NAMED(RCS_LOGGER, "Set joint variable %zu to position %0.3f", vidx, m_jvar_positions[vidx]);
            }
        }
    }

    m_dirty_joint_transforms.assign(m_model->jointCount(), true);
    m_joint_transforms.assign(m_model->jointCount(), Eigen::Affine3d::Identity());

    m_dirty_link_transforms.assign(m_model->linkCount(), true);
    m_link_transforms.assign(m_model->linkCount(), Eigen::Affine3d::Identity());
    m_link_transform_versions.assign(m_model->linkCount(), -1);
    m_dirty_link_transforms[0] = false;
    m_link_transform_versions[0] = 0;

    ROS_DEBUG_NAMED(RCS_LOGGER, "Robot State:");
    ROS_DEBUG_NAMED(RCS_LOGGER, "  %zu Joint Positions", m_jvar_positions.size());
    ROS_DEBUG_NAMED(RCS_LOGGER, "  %zu Dirty Link Transforms", m_dirty_link_transforms.size());
    ROS_DEBUG_NAMED(RCS_LOGGER, "  %zu Link Transforms", m_link_transforms.size());
}

void RobotCollisionState::initCollisionState()
{
    // initialize sphere and spheres states
    m_spheres_states.assign(m_model->spheresModelCount(), CollisionSpheresState());
    for (size_t i = 0; i < m_model->spheresModelCount(); ++i) {
        const CollisionSpheresModel& spheres_model = m_model->spheresModel(i);
        CollisionSpheresState& spheres_state = m_spheres_states[i];

        // map spheres state -> spheres model
        spheres_state.model = &spheres_model;

        spheres_state.spheres.buildFrom(&spheres_state);

        spheres_state.index = i;
    }

    // initialize voxels states
    m_dirty_voxels_states.assign(m_model->voxelsModelCount(), true);
    m_voxels_states.assign(m_model->voxelsModelCount(), CollisionVoxelsState());

    for (size_t i = 0; i < m_model->voxelsModelCount(); ++i) {
        const CollisionVoxelsModel& voxels_model = m_model->voxelsModel(i);
              CollisionVoxelsState& voxels_state = m_voxels_states[i];
        // duplicate voxels from voxels model
        voxels_state.model = &voxels_model;
        voxels_state.voxels = voxels_model.voxels;
    }

    // initialize group states
    m_group_states.assign(m_model->groupCount(), CollisionGroupState());
    for (size_t i = 0; i < m_model->groupCount(); ++i) {
        const CollisionGroupModel& group_model = m_model->group(i);
              CollisionGroupState& group_state = m_group_states[i];
        // map group state -> group model
        group_state.model = &group_model;
    }

    // map group state -> spheres states
    for (size_t ssidx = 0; ssidx < m_spheres_states.size(); ++ssidx) {
        const CollisionSpheresState& spheres_state = m_spheres_states[ssidx];
        // get the link this spheres state is attached to
        const int lidx = spheres_state.model->link_index;
        for (size_t gidx = 0; gidx < m_model->groupCount(); ++gidx) {
            const CollisionGroupModel& group_model = m_model->group(gidx);
            // check if this link index is part of the group model
            if (std::find(
                    group_model.link_indices.begin(),
                    group_model.link_indices.end(),
                    lidx) !=
                group_model.link_indices.end())
            {
                m_group_states[gidx].spheres_indices.push_back(ssidx);
            }
        }
    }

    // map group state -> (outside) voxels states
    for (size_t vsidx = 0; vsidx < m_voxels_states.size(); ++vsidx) {
        const CollisionVoxelsState& voxels_state = m_voxels_states[vsidx];
        // get the link this spheres state is attached to
        const int lidx = voxels_state.model->link_index;
        for (size_t gidx = 0; gidx < m_model->groupCount(); ++gidx) {
            const CollisionGroupModel& group_model = m_model->group(gidx);
            // check if this link index is part of the group model
            if (std::find(
                    group_model.link_indices.begin(),
                    group_model.link_indices.end(),
                    lidx) ==
                group_model.link_indices.end())
            {
                m_group_states[gidx].voxels_indices.push_back(vsidx);
            }
        }
    }

    // initialize link voxels states
    m_link_voxels_states.assign(m_model->linkCount(), nullptr);
    for (int i = 0; i < m_voxels_states.size(); ++i) {
        const CollisionVoxelsModel& voxels_model = m_model->voxelsModel(i);
        CollisionVoxelsState* voxels_state = &m_voxels_states[i];
        m_link_voxels_states[voxels_model.link_index] = voxels_state;
    }

    // initialize link spheres states
    m_link_spheres_states.assign(m_model->linkCount(), nullptr);
    for (int i = 0; i < m_spheres_states.size(); ++i) {
        const CollisionSpheresModel& spheres_model = m_model->spheresModel(i);
        CollisionSpheresState* spheres_state = &m_spheres_states[i];
        m_link_spheres_states[spheres_model.link_index] = spheres_state;
    }

    assert(checkCollisionStateReferences());

    ROS_DEBUG_NAMED(RCS_LOGGER, "Collision State:");
    ROS_DEBUG_NAMED(RCS_LOGGER, "  Spheres States: [%p, %p]", m_spheres_states.data(), m_spheres_states.data() + m_spheres_states.size());
    for (const auto& spheres_state : m_spheres_states) {
        ROS_DEBUG_STREAM_NAMED(RCS_LOGGER, "    model: " << spheres_state.model << ", spheres: " << spheres_state.spheres);
    }
    ROS_DEBUG_NAMED(RCS_LOGGER, "  Dirty Voxels States: %zu", m_dirty_voxels_states.size());
    ROS_DEBUG_NAMED(RCS_LOGGER, "  Voxels States: [%p, %p]", m_voxels_states.data(), m_voxels_states.data() + m_voxels_states.size());
    for (const auto& voxels_state : m_voxels_states) {
        ROS_DEBUG_NAMED(RCS_LOGGER, "    model: %p, voxels: %zu", voxels_state.model, voxels_state.voxels.size());
    }
    ROS_DEBUG_NAMED(RCS_LOGGER, "  Group States: [%p, %p]", m_group_states.data(), m_group_states.data() + m_group_states.size());
    for (const auto& group_state : m_group_states) {
        ROS_DEBUG_NAMED(RCS_LOGGER, "    model: %p, spheres_indices: %s, voxels_indices: %s", group_state.model, to_string(group_state.spheres_indices).c_str(), to_string(group_state.voxels_indices).c_str());
    }
}

bool RobotCollisionState::checkCollisionStateReferences() const
{
    // c++14 would make me happier here...wtb generic lambdas :(
    auto within = [](const void* ptr, const void* start, const void* end) {
        return ptr >= start && ptr < end;
    };

    // check valid sphere state -> spheres state references
    for (const auto& spheres_state : m_spheres_states) {
        for (const auto& sphere_state : spheres_state.spheres) {
            if (!within(sphere_state.parent_state,
                    m_spheres_states.data(),
                    m_spheres_states.data() + m_spheres_states.size()))
            {
                return false;
            }
        }
    }

    for (const auto& group_state : m_group_states) {
        // check valid group state -> spheres states references
        if (std::any_of(
                group_state.spheres_indices.begin(),
                group_state.spheres_indices.end(),
                [&](int ssidx)
                {
                    return ssidx < 0 || ssidx >= m_spheres_states.size();
                }))
        {
            return false;
        }

        // check valid group state -> voxels states references
        if (std::any_of(
                group_state.voxels_indices.begin(),
                group_state.voxels_indices.end(),
                [&](int vsidx)
                {
                    return vsidx < 0 || vsidx >= m_voxels_states.size();
                }))
        {
            return false;
        }
    }

    return true;
}

} // namespace collision
} // namespace sbpl
