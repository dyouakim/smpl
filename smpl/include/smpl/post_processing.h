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

#ifndef SMPL_POST_PROCESSING_H
#define SMPL_POST_PROCESSING_H

// standard includes
#include <vector>

// project includes
#include <smpl/collision_checker.h>
#include <smpl/robot_model.h>
#include <smpl/planning_params.h>
#include <smpl/types.h>

namespace sbpl {
namespace motion {

void ShortcutPath(
    RobotModel* rm,
    CollisionChecker* cc,
    const std::vector<RobotState>& pin,
    std::vector<RobotState>& pout,
    ShortcutType type);

bool InterpolatePath(
    CollisionChecker& cc,
    std::vector<RobotState>& path);

bool CreatePositionVelocityPath(
    RobotModel* rm,
    const std::vector<RobotState>& path,
    std::vector<RobotState>& opath);

bool ExtractPositionPath(
    RobotModel* rm,
    const std::vector<RobotState>& pv_path,
    std::vector<RobotState>& path);

bool ComputePositionPathCosts(
    RobotModel* rm,
    const std::vector<RobotState>& path,
    std::vector<double>& costs);

bool ComputePositionVelocityPathCosts(
    RobotModel* rm,
    const std::vector<RobotState>& pv_path,
    std::vector<double>& costs);

} // namespace motion
} // namespace sbpl

#endif
