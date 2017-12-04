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

#include <smpl/ros/workspace_lattice_allocator.h>

// project includes
#include <smpl/angles.h>
#include <smpl/graph/workspace_lattice.h>

namespace sbpl {
namespace motion {

static const char* PI_LOGGER = "simple";

RobotPlanningSpacePtr WorkspaceLatticeAllocator::allocate(
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
{
    ROS_INFO_NAMED(PI_LOGGER, "Initialize Workspace Lattice");
    auto pspace = std::make_shared<WorkspaceLattice>(robot, checker, params);
    WorkspaceLatticeBase::Params wsp;
    wsp.res_x = m_grid->resolution();
    wsp.res_y = m_grid->resolution();
    wsp.res_z = m_grid->resolution();
    wsp.R_count = 10;
    wsp.P_count = 10;
    wsp.Y_count = 10;

    
    RedundantManipulatorInterface* rmi =
            robot->getExtension<RedundantManipulatorInterface>();
    if (!rmi) {
        ROS_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return RobotPlanningSpacePtr();
    }
    wsp.free_angle_res.resize(rmi->redundantVariableCount(), angles::to_radians(1.0));
    if (!pspace->init(wsp)) {
        ROS_ERROR("Failed to initialize Workspace Lattice");
        return RobotPlanningSpacePtr();
    }

    pspace->setVisualizationFrameId(m_grid->getReferenceFrame());

    return pspace;
}

} // namespace motion
} // namespace sbpl
