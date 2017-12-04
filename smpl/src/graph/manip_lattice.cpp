////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
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

#include <smpl/graph/manip_lattice.h>

// standard includes
#include <sstream>

// system includes
#include <Eigen/Dense>
#include <leatherman/viz.h>
#include <leatherman/print.h>
#include <sbpl/planners/planner.h>

#include <smpl/angles.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/debug/visualize.h>
#include "../profiling.h"

auto std::hash<sbpl::motion::ManipLatticeState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

namespace sbpl {
namespace motion {

ManipLattice::ManipLattice(
    RobotModel* robot_model,
    CollisionChecker* checker,
    PlanningParams* _params)
:
    Extension(),
    RobotPlanningSpace(robot_model, checker, _params),
    PoseProjectionExtension(),
    ExtractRobotStateExtension(),
    m_fk_iface(nullptr),
    m_min_limits(),
    m_max_limits(),
    m_continuous(),
    m_goal_state_id(-1),
    m_start_state_id(-1),
    m_states(),
    m_expanded_states(),
    m_near_goal(false),
    m_t_start(),
    m_viz_frame_id()
{
    m_fk_iface = robot()->getExtension<ForwardKinematicsInterface>();

    m_min_limits.resize(robot()->jointVariableCount());
    m_max_limits.resize(robot()->jointVariableCount());
    m_continuous.resize(robot()->jointVariableCount());
    m_bounded.resize(robot()->jointVariableCount());
    for (int jidx = 0; jidx < robot()->jointVariableCount(); ++jidx) {
        m_min_limits[jidx] = robot_model->minPosLimit(jidx);
        m_max_limits[jidx] = robot_model->maxPosLimit(jidx);
        m_continuous[jidx] = robot_model->isContinuous(jidx);
        m_bounded[jidx] = robot_model->hasPosLimit(jidx);
    }

    m_goal_state_id = reserveHashEntry();
    ROS_DEBUG_NAMED(params()->graph_log, "  goal state has state ID %d", m_goal_state_id);

    // compute the cost per cell to be used by heuristic
    computeCostPerCell();
}

ManipLattice::~ManipLattice()
{
    for (size_t i = 0; i < m_states.size(); i++) {
        delete m_states[i];
        m_states[i] = nullptr;
    }
    m_states.clear();
    m_state_to_id.clear();
}

bool ManipLattice::init(const std::vector<double>& var_res)
{
    ROS_DEBUG_NAMED(params()->graph_log, "Initialize Manip Lattice");
    if (var_res.size() != robot()->jointVariableCount()) {
        ROS_ERROR_NAMED(params()->graph_log, "Insufficient variable resolutions for robot model");
        return false;
    }

    std::vector<int> discretization(robot()->jointVariableCount());
    std::vector<double> deltas(robot()->jointVariableCount());
    for (size_t vidx = 0; vidx < robot()->jointVariableCount(); ++vidx) {
        if (m_continuous[vidx]) {
            discretization[vidx] = (int)std::round((2.0 * M_PI) / var_res[vidx]);
            deltas[vidx] = (2.0 * M_PI) / (double)discretization[vidx];
        } else if (m_bounded[vidx]) {
            const double span = std::fabs(m_max_limits[vidx] - m_min_limits[vidx]);
            discretization[vidx] = std::max(1, (int)std::round(span / var_res[vidx]));
            deltas[vidx] = span / (double)discretization[vidx];
        } else {
            discretization[vidx] = std::numeric_limits<int>::max();
            deltas[vidx] = var_res[vidx];
        }
    }

    ROS_DEBUG_STREAM_NAMED(params()->graph_log, "  coord vals: " << discretization);
    ROS_DEBUG_STREAM_NAMED(params()->graph_log, "  coord deltas: " << deltas);

    m_coord_vals = std::move(discretization);
    m_coord_deltas = std::move(deltas);
    return true;
}

void ManipLattice::PrintState(int stateID, bool verbose, FILE* fout)
{
    assert(stateID >= 0 && stateID < (int)m_states.size());

    if (!fout) {
        fout = stdout;
    }

    ManipLatticeState* entry = m_states[stateID];

    std::stringstream ss;

    if (stateID == m_goal_state_id) {
        ss << "<goal state: { ";
        switch (goal().type) {
        case GoalType::XYZ_GOAL:
        case GoalType::XYZ_RPY_GOAL:
            ss << "pose: " << to_string(goal().tgt_off_pose);
            break;
        case GoalType::JOINT_STATE_GOAL:
            ss << "state: " << to_string(goal().angles);
            break;
        }
        ss << " }>";
    } else {
        ss << "{ ";
        for (size_t i = 0; i < entry->state.size(); ++i) {
            ss << std::setprecision(3) << entry->state[i];
            if (i != entry->state.size() - 1) {
                ss << ", ";
            }
        }
        ss << " }";
    }

    if (fout == stdout) {
        ROS_DEBUG_NAMED(params()->graph_log, "%s", ss.str().c_str());
    } else if (fout == stderr) {
        ROS_WARN("%s", ss.str().c_str());
    } else {
        fprintf(fout, "%s\n", ss.str().c_str());
    }
}

void ManipLattice::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    assert(state_id >= 0 && state_id < m_states.size());

    succs->clear();
    costs->clear();

    ROS_DEBUG_NAMED(params()->expands_log, "expanding state %d", state_id);

    ActionSpacePtr action_space = actionSpace();
    if (!action_space) {
        return;
    }

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    ManipLatticeState* parent_entry = m_states[state_id];

    assert(parent_entry);
    assert(parent_entry->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    ROS_DEBUG_NAMED(params()->expands_log, "  coord: %s", to_string(parent_entry->coord).c_str());
    ROS_DEBUG_NAMED(params()->expands_log, "  angles: %s", to_string(parent_entry->state).c_str());
    ROS_DEBUG_NAMED(params()->expands_log, "  heur: %d", GetGoalHeuristic(state_id));

    SV_SHOW_INFO(getStateVisualization(parent_entry->state, "expansion"));

    int goal_succ_count = 0;

    std::vector<Action> actions;
    if (!action_space->apply(parent_entry->state, actions)) {
        ROS_WARN("Failed to get actions");
        return;
    }

    ROS_DEBUG_NAMED(params()->expands_log, "  actions: %zu", actions.size());

    // check actions for validity
    RobotCoord succ_coord(robot()->jointVariableCount(), 0);
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        ROS_DEBUG_NAMED(params()->expands_log, "    action %zu:", i);
        ROS_DEBUG_NAMED(params()->expands_log, "      waypoints: %zu", action.size());

        double dist;
        if (!checkAction(parent_entry->state, action, dist)) {
            continue;
        }

        // compute destination coords
        stateToCoord(action.back(), succ_coord);

        // get the successor

        // get pose of planning link
        std::vector<double> tgt_off_pose;
        if (!computePlanningFrameFK(action.back(), tgt_off_pose)) {
            ROS_WARN("Failed to compute FK for planning frame");
            continue;
        }

        // check if hash entry already exists, if not then create one
        int succ_state_id = getOrCreateState(succ_coord, action.back());
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);

        // check if this state meets the goal criteria
        const bool is_goal_succ = isGoal(action.back(), tgt_off_pose);
        if (is_goal_succ) {
            // update goal state
            ++goal_succ_count;
        }

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_state_id);
        }
        costs->push_back(cost(parent_entry, succ_entry, is_goal_succ));

        // log successor details
        ROS_DEBUG_NAMED(params()->expands_log, "      succ: %zu", i);
        ROS_DEBUG_NAMED(params()->expands_log, "        id: %5i", succ_state_id);
        ROS_DEBUG_NAMED(params()->expands_log, "        coord: %s", to_string(succ_coord).c_str());
        ROS_DEBUG_NAMED(params()->expands_log, "        state: %s", to_string(succ_entry->state).c_str());
        ROS_DEBUG_NAMED(params()->expands_log, "        pose: %s", to_string(tgt_off_pose).c_str());
        ROS_DEBUG_NAMED(params()->expands_log, "        heur: %2d", GetGoalHeuristic(succ_state_id));
        ROS_DEBUG_NAMED(params()->expands_log, "        cost: %5d", cost(parent_entry, succ_entry, is_goal_succ));
    }

    if (goal_succ_count > 0) {
        ROS_DEBUG_NAMED(params()->expands_log, "Got %d goal successors!", goal_succ_count);
    }
    //ROS_ERROR_STREAM("Expanded node iD is "<<state_id);
    m_expanded_states.push_back(state_id);
    //std::getchar();
}

Stopwatch GetLazySuccsStopwatch("GetLazySuccs", 10);

void ManipLattice::GetLazySuccs(
    int SourceStateID,
    std::vector<int>* SuccIDV,
    std::vector<int>* CostV,
    std::vector<bool>* isTrueCost)
{
    GetLazySuccsStopwatch.start();
    PROFAUTOSTOP(GetLazySuccsStopwatch);

    assert(SourceStateID >= 0 && SourceStateID < m_states.size());

    SuccIDV->clear();
    CostV->clear();
    isTrueCost->clear();

    ROS_DEBUG_NAMED(params()->expands_log, "expand state %d", SourceStateID);

    ActionSpacePtr action_space = actionSpace();
    if (!action_space) {
        return;
    }

    // goal state should be absorbing
    if (SourceStateID == m_goal_state_id) {
        return;
    }

    ManipLatticeState* state_entry = m_states[SourceStateID];

    assert(state_entry);
    assert(state_entry->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    ROS_DEBUG_NAMED(params()->expands_log, "  coord: %s", to_string(state_entry->coord).c_str());
    ROS_DEBUG_NAMED(params()->expands_log, "  angles: %s", to_string(state_entry->state).c_str());
    ROS_DEBUG_NAMED(params()->expands_log, "  heur: %d", GetGoalHeuristic(SourceStateID));

    const RobotState& source_angles = state_entry->state;
    SV_SHOW_DEBUG(getStateVisualization(source_angles, "expansion"));

    std::vector<Action> actions;
    if (!action_space->apply(source_angles, actions)) {
        ROS_WARN("Failed to get successors");
        return;
    }

    ROS_DEBUG_NAMED(params()->expands_log, "  actions: %zu", actions.size());

    int goal_succ_count = 0;
    RobotCoord succ_coord(robot()->jointVariableCount());
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        ROS_DEBUG_NAMED(params()->expands_log, "    action %zu:", i);
        ROS_DEBUG_NAMED(params()->expands_log, "      waypoints: %zu", action.size());

        stateToCoord(action.back(), succ_coord);

        std::vector<double> tgt_off_pose;
        if (!computePlanningFrameFK(action.back(), tgt_off_pose)) {
            ROS_WARN("Failed to compute FK for planning frame");
            continue;
        }

        const bool succ_is_goal_state = isGoal(action.back(), tgt_off_pose);
        if (succ_is_goal_state) {
            ++goal_succ_count;
        }

        int succ_state_id = getOrCreateState(succ_coord, action.back());
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);

        if (succ_is_goal_state) {
            SuccIDV->push_back(m_goal_state_id);
        } else {
            SuccIDV->push_back(succ_state_id);
        }
        CostV->push_back(cost(state_entry, succ_entry, succ_is_goal_state));
        isTrueCost->push_back(false);

        // log successor details
        ROS_DEBUG_NAMED(params()->expands_log, "      succ: %zu", i);
        ROS_DEBUG_NAMED(params()->expands_log, "        id: %5i", succ_state_id);
        ROS_DEBUG_NAMED(params()->expands_log, "        coord: %s", to_string(succ_coord).c_str());
        ROS_DEBUG_NAMED(params()->expands_log, "        state: %s", to_string(succ_entry->state).c_str());
        ROS_DEBUG_NAMED(params()->expands_log, "        pose: %s", to_string(tgt_off_pose).c_str());
        ROS_DEBUG_NAMED(params()->expands_log, "        heur: %2d", GetGoalHeuristic(succ_state_id));
        ROS_DEBUG_NAMED(params()->expands_log, "        cost: %5d", cost(state_entry, succ_entry, succ_is_goal_state));
    }

    if (goal_succ_count > 0) {
        ROS_DEBUG_NAMED(params()->expands_log, "Got %d goal successors!", goal_succ_count);
    }

    m_expanded_states.push_back(SourceStateID);
}

Stopwatch GetTrueCostStopwatch("GetTrueCost", 10);

int ManipLattice::GetTrueCost(int parentID, int childID)
{
    GetTrueCostStopwatch.start();
    PROFAUTOSTOP(GetTrueCostStopwatch);

    ROS_DEBUG_NAMED(params()->expands_log, "evaluating cost of transition %d -> %d", parentID, childID);

    assert(parentID >= 0 && parentID < (int)m_states.size());
    assert(childID >= 0 && childID < (int)m_states.size());

    ManipLatticeState* parent_entry = m_states[parentID];
    ManipLatticeState* child_entry = m_states[childID];
    assert(parent_entry && parent_entry->coord.size() >= robot()->jointVariableCount());
    assert(child_entry && child_entry->coord.size() >= robot()->jointVariableCount());

    const RobotState& parent_angles = parent_entry->state;
    SV_SHOW_DEBUG(getStateVisualization(parent_angles, "expansion"));

    ActionSpacePtr action_space = actionSpace();
    if (!action_space) {
        return -1;
    }

    std::vector<Action> actions;
    if (!action_space->apply(parent_angles, actions)) {
        ROS_WARN("Failed to get actions");
        return -1;
    }

    const bool goal_edge = (childID == m_goal_state_id);

    size_t num_actions = 0;

    // check actions for validity and find the valid action with the least cost
    RobotCoord succ_coord(robot()->jointVariableCount());
    int best_cost = std::numeric_limits<int>::max();
    for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
        const Action& action = actions[aidx];

        stateToCoord(action.back(), succ_coord);

        // check whether this action leads to the child state
        if (goal_edge) {
            std::vector<double> tgt_off_pose;
            if (!computePlanningFrameFK(action.back(), tgt_off_pose)) {
                ROS_WARN("Failed to compute FK for planning frame");
                continue;
            }

            // skip actions which don't end up at a goal state
            if (!isGoal(action.back(), tgt_off_pose)) {
                continue;
            }
        } else {
            // skip actions which don't end up at the child state
            if (succ_coord != child_entry->coord) {
                continue;
            }
        }

        ROS_DEBUG_NAMED(params()->expands_log, "    action %zu:", num_actions++);
        ROS_DEBUG_NAMED(params()->expands_log, "      waypoints %zu:", action.size());

        double dist;
        if (!checkAction(parent_angles, action, dist)) {
            continue;
        }

        // get the unique state
        int succ_state_id = goal_edge ? getHashEntry(succ_coord) : childID;
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);
        assert(succ_entry);

        const int edge_cost = cost(parent_entry, succ_entry, goal_edge);
        if (edge_cost < best_cost) {
            best_cost = edge_cost;
        }
    }

    if (best_cost != std::numeric_limits<int>::max()) {
        return best_cost;
    } else {
        return -1;
    }
}

const RobotState& ManipLattice::extractState(int state_id)
{
    return m_states[state_id]->state;
}

bool ManipLattice::projectToPose(int state_id, Eigen::Affine3d& pose)
{
    if (state_id == getGoalStateID()) {
        assert(goal().tgt_off_pose.size() >= 6);
        Eigen::Matrix3d R;
        angles::from_euler_zyx(
                goal().tgt_off_pose[5],
                goal().tgt_off_pose[4],
                goal().tgt_off_pose[3],
                R);
        pose =
                Eigen::Translation3d(
                        goal().tgt_off_pose[0],
                        goal().tgt_off_pose[1],
                        goal().tgt_off_pose[2]) *
                Eigen::Affine3d(R);
        return true;
    }

    std::vector<double> vpose;
    if (!computePlanningFrameFK(m_states[state_id]->state, vpose)) {
        ROS_WARN("Failed to compute fk for state %d", state_id);
        return false;
    }

    Eigen::Matrix3d R;
    angles::from_euler_zyx(vpose[5], vpose[4], vpose[3], R);
    pose =
            Eigen::Translation3d(vpose[0], vpose[1], vpose[2]) *
            Eigen::Affine3d(R);
    return true;
}

void ManipLattice::GetPreds(
    int TargetStateID,
    std::vector<int>* PredIDV,
    std::vector<int>* CostV)
{
    ROS_WARN("GetPreds unimplemented");
}

// angles are counterclockwise from 0 to 360 in radians, 0 is the center of bin
// 0, ...
inline void ManipLattice::coordToState(
    const RobotCoord& coord,
    RobotState& state) const
{
    assert((int)state.size() == robot()->jointVariableCount() &&
            (int)coord.size() == robot()->jointVariableCount());

    for (size_t i = 0; i < coord.size(); ++i) {
        if (m_continuous[i]) {
            state[i] = coord[i] * m_coord_deltas[i];
        } else if (!m_bounded[i]) {
            state[i] = (double)coord[i] * m_coord_deltas[i];
        } else {
            state[i] = m_min_limits[i] + coord[i] * m_coord_deltas[i];
        }
    }
}

inline void ManipLattice::stateToCoord(
    const RobotState& state,
    RobotCoord& coord) const
{
    assert((int)state.size() == robot()->jointVariableCount() &&
            (int)coord.size() == robot()->jointVariableCount());

    for (size_t i = 0; i < state.size(); ++i) {
        if (m_continuous[i]) {
            double pos_angle = angles::normalize_angle_positive(state[i]);

            coord[i] = (int)((pos_angle + m_coord_deltas[i] * 0.5) / m_coord_deltas[i]);

            if (coord[i] == m_coord_vals[i]) {
                coord[i] = 0;
            }
        } else if (!m_bounded[i]) {
            if (state[i] >= 0.0) {
                coord[i] = (int)(state[i] / m_coord_deltas[i] + 0.5);
            } else {
                coord[i] = (int)(state[i] / m_coord_deltas[i] - 0.5);
            }
        } else {
            coord[i] = (int)(((state[i] - m_min_limits[i]) / m_coord_deltas[i]) + 0.5);
        }
    }
}

ManipLatticeState* ManipLattice::getHashEntry(int state_id) const
{
    if (state_id < 0 || state_id >= (int)m_states.size()) {
        return nullptr;
    }

    return m_states[state_id];
}

/// Return the state id of the state with the given coordinate or -1 if the
/// state has not yet been allocated.
int ManipLattice::getHashEntry(const RobotCoord& coord)
{
    ManipLatticeState state;
    state.coord = coord;
    auto sit = m_state_to_id.find(&state);
    if (sit == m_state_to_id.end()) {
        return -1;
    }
    return sit->second;
}

int ManipLattice::createHashEntry(
    const RobotCoord& coord,
    const RobotState& state)
{
    int state_id = reserveHashEntry();
    ManipLatticeState* entry = getHashEntry(state_id);

    entry->coord = coord;
    entry->state = state;

    // map state -> state id
    m_state_to_id[entry] = state_id;

    return state_id;
}

int ManipLattice::getOrCreateState(
    const RobotCoord& coord,
    const RobotState& state)
{
    int state_id = getHashEntry(coord);
    if (state_id < 0) {
        state_id = createHashEntry(coord, state);
    }
    return state_id;
}

int ManipLattice::reserveHashEntry()
{
    ManipLatticeState* entry = new ManipLatticeState;
    int state_id = (int)m_states.size();

    // map state id -> state
    m_states.push_back(entry);

    // map planner state -> graph state
    int* pinds = new int[NUMOFINDICES_STATEID2IND];
    std::fill(pinds, pinds + NUMOFINDICES_STATEID2IND, -1);
    StateID2IndexMapping.push_back(pinds);

    return state_id;
}

/// NOTE: const although RobotModel::computePlanningLinkFK used underneath may
/// not be
bool ManipLattice::computePlanningFrameFK(
    const RobotState& state,
    std::vector<double>& pose) const
{
    assert(state.size() == robot()->jointVariableCount());

    if (!m_fk_iface || !m_fk_iface->computePlanningLinkFK(state, pose)) {
        return false;
    }

    pose = getTargetOffsetPose(pose);

    assert(pose.size() == 6);
    return true;
}

int ManipLattice::cost(
    ManipLatticeState* HashEntry1,
    ManipLatticeState* HashEntry2,
    bool bState2IsGoal) const
{
    const int DefaultCostMultiplier = 1000;
    return DefaultCostMultiplier;
}

bool ManipLattice::checkAction(
    const RobotState& state,
    const Action& action,
    double& dist)
{
    std::uint32_t violation_mask = 0x00000000;
    int plen = 0;
    int nchecks = 0;
    dist = 0.0;

    // check intermediate states for collisions
    for (size_t iidx = 0; iidx < action.size(); ++iidx) {
        const RobotState& istate = action[iidx];
        ROS_DEBUG_NAMED(params()->expands_log, "        %zu: %s", iidx, to_string(istate).c_str());

        // check joint limits
        if (!robot()->checkJointLimits(istate)) {
            ROS_DEBUG_NAMED(params()->expands_log, "        -> violates joint limits");
            violation_mask |= 0x00000001;
            break;
        }

        // TODO/NOTE: this can result in an unnecessary number of collision
        // checks per each action; leaving commented here as it might hint at
        // an optimization where actions are checked at a coarse resolution as
        // a way of speeding up overall collision checking; in that case, the
        // isStateToStateValid function on CollisionChecker would have semantics
        // meaning "collision check a waypoint path without including the
        // endpoints".
//        // check for collisions
//        if (!collisionChecker()->isStateValid(istate, params()->verbose_collisions_, false, dist))
//        {
//            ROS_DEBUG_NAMED(params()->expands_log_, "        -> in collision (dist: %0.3f)", dist);
//            violation_mask |= 0x00000002;
//            break;
//        }
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions along path from parent to first waypoint
    if (!collisionChecker()->isStateToStateValid(state, action[0], plen, nchecks, dist)) {
        ROS_DEBUG_NAMED(params()->expands_log, "        -> path to first waypoint in collision (dist: %0.3f, path_length: %d)", dist, plen);
        violation_mask |= 0x00000004;
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions between waypoints
    for (size_t j = 1; j < action.size(); ++j) {
        const RobotState& prev_istate = action[j - 1];
        const RobotState& curr_istate = action[j];
        if (!collisionChecker()->isStateToStateValid(
                prev_istate, curr_istate, plen, nchecks, dist))
        {
            ROS_DEBUG_NAMED(params()->expands_log, "        -> path between waypoints %zu and %zu in collision (dist: %0.3f, path_length: %d)", j - 1, j, dist, plen);
            violation_mask |= 0x00000008;
            break;
        }
    }

    if (violation_mask) {
        return false;
    }

    return true;
}

bool ManipLattice::isGoal(
    const RobotState& state,
    const std::vector<double>& pose)
{
    switch (goal().type) {
    case GoalType::JOINT_STATE_GOAL:
    {
        for (int i = 0; i < goal().angles.size(); i++) {
            if (fabs(state[i] - goal().angles[i]) > goal().angle_tolerances[i]) {
                return false;
            }
        }
        return true;
    }   break;
    case GoalType::XYZ_RPY_GOAL:
    {
        const double dx = fabs(pose[0] - goal().tgt_off_pose[0]);
        const double dy = fabs(pose[1] - goal().tgt_off_pose[1]);
        const double dz = fabs(pose[2] - goal().tgt_off_pose[2]);
        if (dx <= goal().xyz_tolerance[0] &&
            dy <= goal().xyz_tolerance[1] &&
            dz <= goal().xyz_tolerance[2])
        {
            // log the amount of time required for the search to get close to the goal
            if (!m_near_goal) {
                using namespace std::chrono;
                auto time_to_goal_region = clock::now() - m_t_start;
                auto time_to_goal_s =
                        duration_cast<duration<double>>(time_to_goal_region);
                m_near_goal = true;
                ROS_INFO_NAMED(params()->expands_log, "Search is at %0.2f %0.2f %0.2f, within %0.3fm of the goal (%0.2f %0.2f %0.2f) after %0.4f sec. (after %zu expansions)",
                        pose[0], pose[1], pose[2],
                        goal().xyz_tolerance[0],
                        goal().tgt_off_pose[0], goal().tgt_off_pose[1], goal().tgt_off_pose[2],
                        time_to_goal_s.count(),
                        m_expanded_states.size());
            }
            Eigen::Quaterniond qg(
                    Eigen::AngleAxisd(goal().tgt_off_pose[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(goal().tgt_off_pose[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(goal().tgt_off_pose[3], Eigen::Vector3d::UnitX()));
            Eigen::Quaterniond q(
                    Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()));
            if (q.dot(qg) < 0.0) {
                qg = Eigen::Quaterniond(-qg.w(), -qg.x(), -qg.y(), -qg.z());
            }

//            const double theta = angles::normalize_angle(Eigen::AngleAxisd(qg.conjugate() * q).angle());
            const double theta = angles::normalize_angle(2.0 * acos(q.dot(qg)));
            if (theta < goal().rpy_tolerance[0]) {
                return true;
            }
        }
    }   break;
    case GoalType::XYZ_GOAL:
    {
        if (fabs(pose[0] - goal().tgt_off_pose[0]) <= goal().xyz_tolerance[0] &&
            fabs(pose[1] - goal().tgt_off_pose[1]) <= goal().xyz_tolerance[1] &&
            fabs(pose[2] - goal().tgt_off_pose[2]) <= goal().xyz_tolerance[2])
        {
            return true;
        }
    }   break;
    default:
    {
        ROS_ERROR_NAMED(params()->graph_log, "Unknown goal type.");
    }   break;
    }

    return false;
}

visualization_msgs::MarkerArray ManipLattice::getStateVisualization(
    const RobotState& vars,
    const std::string& ns)
{
    auto ma = collisionChecker()->getCollisionModelVisualization(vars);
    for (auto& marker : ma.markers) {
        marker.ns = ns;
    }
    return ma;
}

bool ManipLattice::setStart(const RobotState& state)
{
    ROS_DEBUG_NAMED(params()->graph_log, "set the start state");

    if ((int)state.size() < robot()->jointVariableCount()) {
        ROS_ERROR_NAMED(params()->graph_log, "start state does not contain enough joint positions");
        return false;
    }

    ROS_DEBUG_NAMED(params()->graph_log, "  state: %s", to_string(state).c_str());

    // get joint positions of starting configuration
    std::vector<double> pose(6, 0.0);
    if (!computePlanningFrameFK(state, pose)) {
        ROS_WARN(" -> unable to compute forward kinematics");
        return false;
    }
    ROS_DEBUG_NAMED(params()->graph_log, "  planning link pose: { x: %0.3f, y: %0.3f, z: %0.3f, R: %0.3f, P: %0.3f, Y: %0.3f }", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

    // check joint limits of starting configuration
    if (!robot()->checkJointLimits(state, true)) {
        ROS_WARN(" -> violates the joint limits");
        return false;
    }

    // check if the start configuration is in collision
    double dist = 0.0;
    if (!collisionChecker()->isStateValid(state, true, false, dist)) {
        ROS_WARN(" -> in collision (distance to nearest obstacle %0.3fm)", dist);
        return false;
    }

    SV_SHOW_INFO(getStateVisualization(state, "start_config"));

    // get arm position in environment
    RobotCoord start_coord(robot()->jointVariableCount());
    stateToCoord(state, start_coord);
    ROS_DEBUG_NAMED(params()->graph_log, "  coord: %s", to_string(start_coord).c_str());

    m_start_state_id = getOrCreateState(start_coord, state);

    // notify observers of updated start state
    return RobotPlanningSpace::setStart(state);
}

bool ManipLattice::setGoal(const GoalConstraint& goal)
{
    switch (goal.type) {
    case GoalType::XYZ_GOAL:
    case GoalType::XYZ_RPY_GOAL: {
        return setGoalPose(goal);
    }   break;
    case GoalType::JOINT_STATE_GOAL:
        return setGoalConfiguration(goal);
    default:
        return false;
    }
}

void ManipLattice::getExpandedStates(std::vector<RobotState>& states) const
{
    RobotState state(robot()->jointVariableCount(), 0);

    for (size_t i = 0; i < m_expanded_states.size(); ++i) {
        const ManipLatticeState* entry = getHashEntry(m_expanded_states[i]);
        if (entry) {
            states.push_back(entry->state);
        }
        states.push_back(state);
    }
}

void ManipLattice::setVisualizationFrameId(const std::string& frame_id)
{
    m_viz_frame_id = frame_id;
}

const std::string& ManipLattice::visualizationFrameId() const
{
    return m_viz_frame_id;
}

void ManipLattice::computeCostPerCell()
{
    ROS_WARN("yeah...");
}

bool ManipLattice::extractPath(
    const std::vector<int>& idpath,
    std::vector<RobotState>& path, std::vector<geometry_msgs::PoseStamped>& eePath)
{
    if (idpath.empty()) {
        return true;
    }

    std::vector<RobotState> opath;

    // attempt to handle paths of length 1...do any of the sbpl planners still
    // return a single-point path in some cases?
    if (idpath.size() == 1) {
        const int state_id = idpath[0];

        if (state_id == getGoalStateID()) {
            const ManipLatticeState* entry = getHashEntry(getStartStateID());
            if (!entry) {
                ROS_ERROR_NAMED(params()->graph_log, "Failed to get state entry for state %d", getStartStateID());
                return false;
            }
            opath.push_back(entry->state);
        } else {
            const ManipLatticeState* entry = getHashEntry(state_id);
            if (!entry) {
                ROS_ERROR_NAMED(params()->graph_log, "Failed to get state entry for state %d", state_id);
                return false;
            }
            opath.push_back(entry->state);
        }

        SV_SHOW_INFO(getStateVisualization(opath.back(), "goal_state"));
        return true;
    }

    if (idpath[0] == getGoalStateID()) {
        ROS_ERROR_NAMED(params()->graph_log, "Cannot extract a non-trivial path starting from the goal state");
        return false;
    }

    // grab the first point
    {
        const ManipLatticeState* entry = getHashEntry(idpath[0]);
        if (!entry) {
            ROS_ERROR_NAMED(params()->graph_log, "Failed to get state entry for state %d", idpath[0]);
            return false;
        }
        opath.push_back(entry->state);
    }

    ActionSpacePtr action_space = actionSpace();
    if (!action_space) {
        ROS_ERROR_NAMED(params()->graph_log, "No action space available for path extraction");
        return false;
    }

    // grab the rest of the points
    for (size_t i = 1; i < idpath.size(); ++i) {
        const int prev_id = idpath[i - 1];
        const int curr_id = idpath[i];
        ROS_DEBUG_NAMED(params()->graph_log, "Extract motion from state %d to state %d", prev_id, curr_id);

        if (prev_id == getGoalStateID()) {
            ROS_ERROR_NAMED(params()->graph_log, "Cannot determine goal state predecessor state during path extraction");
            return false;
        }

        if (curr_id == getGoalStateID()) {
            ROS_DEBUG_NAMED(params()->graph_log, "Search for transition to goal state");

            ManipLatticeState* prev_entry = m_states[prev_id];
            const RobotState& prev_state = prev_entry->state;

            std::vector<Action> actions;
            if (!action_space->apply(prev_state, actions)) {
                ROS_ERROR_NAMED(params()->graph_log, "Failed to get actions while extracting the path");
                return false;
            }

            // find the goal state corresponding to the cheapest valid action
            ManipLatticeState* best_goal_state = nullptr;
            RobotCoord succ_coord(robot()->jointVariableCount());
            int best_cost = std::numeric_limits<int>::max();
            for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
                const Action& action = actions[aidx];

                std::vector<double> tgt_off_pose;
                if (!computePlanningFrameFK(action.back(), tgt_off_pose)) {
                    ROS_WARN("Failed to compute FK for planning frame");
                    continue;
                }

                // skip non-goal states
                if (!isGoal(action.back(), tgt_off_pose)) {
                    continue;
                }

                // check the validity of this transition
                double dist;
                if (!checkAction(prev_state, action, dist)) {
                    continue;
                }

                stateToCoord(action.back(), succ_coord);
                int succ_state_id = getHashEntry(succ_coord);
                ManipLatticeState* succ_entry = getHashEntry(succ_state_id);
                assert(succ_entry);

                const int edge_cost = cost(prev_entry, succ_entry, true);
                if (edge_cost < best_cost) {
                    best_cost = edge_cost;
                    best_goal_state = succ_entry;
                }
            }

            if (!best_goal_state) {
                ROS_ERROR_NAMED(params()->graph_log, "Failed to find valid goal successor from state %s during path extraction", to_string(prev_entry->state).c_str());
                return false;
            }

            opath.push_back(best_goal_state->state);
        } else {
            const ManipLatticeState* entry = getHashEntry(curr_id);
            if (!entry) {
                ROS_ERROR_NAMED(params()->graph_log, "Failed to get state entry state %d", curr_id);
                return false;
            }

            ROS_DEBUG_NAMED(params()->graph_log, "Extract successor state %s", to_string(entry->state).c_str());
            opath.push_back(entry->state);
        }
    }

    // we made it!
    path = std::move(opath);
    SV_SHOW_INFO(getStateVisualization(path.back(), "goal_state"));
    return true;
}

Extension* ManipLattice::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotPlanningSpace>() ||
        class_code == GetClassCode<PointProjectionExtension>() ||
        class_code == GetClassCode<ExtractRobotStateExtension>() ||
        class_code == GetClassCode<ManipLattice>() ||
        class_code == GetClassCode<PoseProjectionExtension>())
    {
        return this;
    }
    return nullptr;
}

/// \brief Return the ID of the goal state or -1 if no goal has been set.
int ManipLattice::getGoalStateID() const
{
    return m_goal_state_id;
}

/// \brief Return the ID of the start state or -1 if no start has been set.
///
/// This returns the reserved id corresponding to all states which are goal
/// states and not the state id of any particular unique state.
int ManipLattice::getStartStateID() const
{
    return m_start_state_id;
}

/// \brief Get the (heuristic) distance from the planning frame position to the
///     start
RobotState ManipLattice::getStartConfiguration() const
{
    if (m_start_state_id >= 0) {
        return getHashEntry(m_start_state_id)->state;
    } else {
        return RobotState();
    }
}

/// Set a 6-dof goal pose for the planning link
bool ManipLattice::setGoalPose(const GoalConstraint& gc)
{
    // check arguments
    if (gc.pose.size() != 6) {
        ROS_ERROR_NAMED(params()->graph_log, "Goal pose has incorrect format");
        return false;
    }

    if (gc.tgt_off_pose.size() != 6) {
        ROS_ERROR_NAMED(params()->graph_log, "Goal target offset pose has incorrect format");
        return false;
    }

    SV_SHOW_INFO(::viz::getPosesMarkerArray({ gc.tgt_off_pose }, m_viz_frame_id, "target_goal"));

    using namespace std::chrono;
    auto now = clock::now();
    auto now_s = duration_cast<duration<double>>(now.time_since_epoch());
    ROS_DEBUG_NAMED(params()->graph_log, "time: %f", now_s.count());
    ROS_DEBUG_NAMED(params()->graph_log, "A new goal has been set.");
    ROS_DEBUG_NAMED(params()->graph_log, "    xyz (meters): (%0.2f, %0.2f, %0.2f)", gc.pose[0], gc.pose[1], gc.pose[2]);
    ROS_DEBUG_NAMED(params()->graph_log, "    tol (meters): %0.3f", gc.xyz_tolerance[0]);
    ROS_DEBUG_NAMED(params()->graph_log, "    rpy (radians): (%0.2f, %0.2f, %0.2f)", gc.pose[3], gc.pose[4], gc.pose[5]);
    ROS_DEBUG_NAMED(params()->graph_log, "    tol (radians): %0.3f", gc.rpy_tolerance[0]);

    startNewSearch();

    // set the (modified) goal
    return RobotPlanningSpace::setGoal(gc);
}

/// \brief Set a full joint configuration goal.
bool ManipLattice::setGoalConfiguration(const GoalConstraint& goal)
{
    // compute the goal pose
    std::vector<double> pose;
    if (!computePlanningFrameFK(goal.angles, pose)) {
        ROS_WARN("Could not compute planning link FK for given goal configuration!");
        return false;
    }

    startNewSearch();

    // notify observers of updated goal
    return RobotPlanningSpace::setGoal(goal);
}

// Reset any variables that should be set just before a new search is started.
void ManipLattice::startNewSearch()
{
    m_expanded_states.clear();
    m_near_goal = false;
    m_t_start = clock::now();
}

/// \brief Return the 6-dof goal pose for the offset from the tip link.
std::vector<double> ManipLattice::getTargetOffsetPose(
    const std::vector<double>& tip_pose) const
{
    // pose represents T_planning_eef
    Eigen::Affine3d T_planning_tipoff = // T_planning_eef * T_eef_tipoff
            Eigen::Translation3d(tip_pose[0], tip_pose[1], tip_pose[2]) *
            Eigen::AngleAxisd(tip_pose[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(tip_pose[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(tip_pose[3], Eigen::Vector3d::UnitX()) *
            Eigen::Translation3d(
                    goal().xyz_offset[0],
                    goal().xyz_offset[1],
                    goal().xyz_offset[2]);
    const Eigen::Vector3d voff(T_planning_tipoff.translation());
    return { voff.x(), voff.y(), voff.z(), tip_pose[3], tip_pose[4], tip_pose[5] };
}

} // namespace motion
} // namespace sbpl
