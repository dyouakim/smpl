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

#ifndef sbpl_collision_self_collision_model_h
#define sbpl_collision_self_collision_model_h

// standard includes
#include <memory>
#include <string>

// system includes
#include <smpl/forward.h>
#include <smpl/occupancy_grid.h>

// project includes
#include <sbpl_collision_checking/allowed_collisions_interface.h>
#include <sbpl_collision_checking/attached_bodies_collision_model.h>
#include <sbpl_collision_checking/attached_bodies_collision_state.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include <sbpl_collision_checking/robot_motion_collision_model.h>
#include <sbpl_collision_checking/robot_collision_state.h>
#include <sbpl_collision_checking/types.h>

namespace sbpl {
namespace collision {

class SelfCollisionModelImpl;

SBPL_CLASS_FORWARD(SelfCollisionModel);
class SelfCollisionModel
{
public:

    SelfCollisionModel(
        OccupancyGrid* grid,
        const RobotCollisionModel* rcm,
        const AttachedBodiesCollisionModel* ab_model);

    ~SelfCollisionModel();

    const AllowedCollisionMatrix& allowedCollisionMatrix() const;
    void updateAllowedCollisionMatrix(const AllowedCollisionMatrix& acm);
    void setAllowedCollisionMatrix(const AllowedCollisionMatrix& acm);

    void setPadding(double padding);

    void setWorldToModelTransform(const Eigen::Affine3d& transform);

    bool checkCollision(
        const RobotCollisionState& state,
        const AttachedBodiesCollisionState& ab_state,
        const int gidx,
        double& dist);

    bool checkCollision(
        const RobotCollisionState& state,
        const AttachedBodiesCollisionState& ab_state,
        const AllowedCollisionsInterface& aci,
        const int gidx,
        double& dist);

    bool checkMotionCollision(
        RobotCollisionState& state,
        AttachedBodiesCollisionState& ab_state,
        const RobotMotionCollisionModel& rmcm,
        const std::vector<double>& start,
        const std::vector<double>& finish,
        const int gidx,
        double& dist);

    bool checkMotionCollision(
        RobotCollisionState& state,
        AttachedBodiesCollisionState& ab_state,
        const AllowedCollisionsInterface& aci,
        const RobotMotionCollisionModel& rmcm,
        const std::vector<double>& start,
        const std::vector<double>& finish,
        const int gidx,
        double& dist);

    double collisionDistance(
        const RobotCollisionState& state,
        const AttachedBodiesCollisionState& ab_state,
        const int gidx);

    double collisionDistance(
        const RobotCollisionState& state,
        const AttachedBodiesCollisionState& ab_state,
        const AllowedCollisionsInterface& aci,
        const int gidx);

    bool collisionDetails(
        const RobotCollisionState& state,
        const AttachedBodiesCollisionState& ab_state,
        const int gidx,
        CollisionDetails& details);

    bool collisionDetails(
        const RobotCollisionState& state,
        const AttachedBodiesCollisionState& ab_state,
        const AllowedCollisionsInterface& aci,
        const int gidx,
        CollisionDetails& details);

    // TODO: contacts checks
    // TODO: detailed checks

private:

    std::unique_ptr<SelfCollisionModelImpl> m_impl;
};

} // namespace collision
} // namespace sbpl

#endif
