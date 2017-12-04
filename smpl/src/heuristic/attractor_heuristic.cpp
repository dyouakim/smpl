////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
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

#include <smpl/heuristic/attractor_heuristic.h>

#include <smpl/occupancy_grid.h>

namespace sbpl {
namespace motion {

bool AttractorHeuristic::init(
    RobotPlanningSpace* space,
    const OccupancyGrid* grid)
{
    if (grid == NULL) {
        return false;
    }

    if (!RobotHeuristic::init(space)) {
        return false;
    }

    m_grid = grid;
    m_ers = space->getExtension<ExtractRobotStateExtension>();
    return true;
}

double AttractorHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    return 0.0;
}

double AttractorHeuristic::getMetricStartDistance(double x, double y, double z)
{
    return 0.0;
}

Extension* AttractorHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

int AttractorHeuristic::GetGoalHeuristic(int state_id)
{
    if (state_id == planningSpace()->getGoalStateID()) {
        return 0;
    }
    if (!m_ers) {
        return 0;
    }
    if (planningSpace()->goal().type != GoalType::JOINT_STATE_GOAL) {
        return 0;
    }

    const RobotState& state = m_ers->extractState(state_id);
    if (state.size() != m_attractor.size()) {
        return 0;
    }

    double dsum = 0.0;
    for (size_t i = 0; i < state.size(); ++i) {
        double dj = (state[i] - m_attractor[i]);
        dsum += dj * dj;
    }
    dsum = 1000.0 * std::sqrt(dsum);
    return (int)dsum;
}

int AttractorHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int AttractorHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

} // namespace motion
} // namespace sbpl
