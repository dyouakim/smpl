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

#include <smpl/graph/robot_planning_space.h>

// project includes
#include <smpl/console/console.h>
#include <smpl/heuristic/robot_heuristic.h>

namespace sbpl {
namespace motion {

RobotPlanningSpace::~RobotPlanningSpace()
{
}

bool RobotPlanningSpace::init(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams* params)
{
    if (!robot || !checker || !params) {
        return false;
    }

    m_robot = robot;
    m_checker = checker;
    m_params = params;
    return true;
}

bool RobotPlanningSpace::insertHeuristic(RobotHeuristic* h)
{
    auto hit = std::find(m_heuristics.begin(), m_heuristics.end(), h);
    if (hit != m_heuristics.end()) {
        return false;
    }
    SMPL_DEBUG_NAMED(params()->graph_log, "Insert heuristic %p", h);
    m_heuristics.push_back(h);
    return true;
}

bool RobotPlanningSpace::eraseHeuristic(const RobotHeuristic* h)
{
    auto hit = std::remove(m_heuristics.begin(), m_heuristics.end(), h);
    if (hit == m_heuristics.end()) {
        return false;
    }
    SMPL_DEBUG_NAMED(params()->graph_log, "Erasure heuristic %p", h);
    m_heuristics.erase(hit, m_heuristics.end());
    return true;
}

bool RobotPlanningSpace::hasHeuristic(const RobotHeuristic* h)
{
    auto hit = std::find(m_heuristics.begin(), m_heuristics.end(), h);
    return hit != m_heuristics.end();
}

bool RobotPlanningSpace::setStart(const RobotState& start)
{
    m_start = start;
    notifyStartChanged(start);
    return true;
}

bool RobotPlanningSpace::setGoal(const GoalConstraint& goal)
{
    m_goal = goal;
    notifyGoalChanged(goal);
    return true;
}

/// Add an observer to the list of observers if it is not already observing
void RobotPlanningSpace::insertObserver(RobotPlanningSpaceObserver* obs)
{
    if (std::find(m_obs.begin(), m_obs.end(), obs) == m_obs.end()) {
        SMPL_DEBUG_NAMED(params()->graph_log, "Insert observer %p", obs);
        m_obs.push_back(obs);
    }
}

/// Remove an observer from the list of observers
void RobotPlanningSpace::eraseObserver(RobotPlanningSpaceObserver* obs)
{
    auto it = std::remove(m_obs.begin(), m_obs.end(), obs);
    if (it != m_obs.end()) {
        SMPL_DEBUG_NAMED(params()->graph_log, "Erase observer %p", obs);
        m_obs.erase(it, m_obs.end());
    }
}

/// Return whether an observer is in the list of observers
bool RobotPlanningSpace::hasObserver(RobotPlanningSpaceObserver* obs) const
{
    return std::find(m_obs.begin(), m_obs.end(), obs) != m_obs.end();
}

void RobotPlanningSpace::notifyStartChanged(const RobotState& state)
{
    for (RobotPlanningSpaceObserver* obs : m_obs) {
        SMPL_DEBUG_NAMED(params()->graph_log, "Notify %p of start change", obs);
        obs->updateStart(state);
    }
}

void RobotPlanningSpace::notifyGoalChanged(const GoalConstraint& goal)
{
    for (RobotPlanningSpaceObserver* obs : m_obs) {
        SMPL_DEBUG_NAMED(params()->graph_log, "Notify %p of goal change", obs);
        obs->updateGoal(goal);
    }
}

int RobotPlanningSpace::GetGoalHeuristic(int state_id)
{
    if (numHeuristics() == 0) {
        return 0;
    }
    return heuristic(0)->GetGoalHeuristic(state_id);
}

int RobotPlanningSpace::GetStartHeuristic(int state_id)
{
    if (numHeuristics() == 0) {
        return 0;
    }
    return heuristic(0)->GetStartHeuristic(state_id);
}

int RobotPlanningSpace::GetFromToHeuristic(int from_id, int to_id)
{
    if (numHeuristics() == 0) {
        return 0;
    }
    return heuristic(0)->GetFromToHeuristic(from_id, to_id);
}

void RobotPlanningSpace::GetLazySuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs,
    std::vector<bool>* true_costs)
{
    GetSuccs(state_id, succs, costs);
    true_costs->assign(succs->size(), true);
}

int RobotPlanningSpace::GetTrueCost(int parent_id, int child_id)
{
    std::vector<int> succs;
    std::vector<int> costs;
    GetSuccs(parent_id, &succs, &costs);
    auto sit = std::find(succs.begin(), succs.end(), child_id);
    if (sit == succs.end()) {
        return -1;
    }

    return costs[std::distance(succs.begin(), sit)];
}

} // namespace motion
} // namespace sbpl
