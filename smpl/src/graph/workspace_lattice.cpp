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

#include <smpl/graph/workspace_lattice.h>

// system includes
#include <boost/functional/hash.hpp>
#include <leatherman/print.h>
#include <leatherman/viz.h>

// project includes
#include <smpl/angles.h>
#include <smpl/debug/visualize.h>
#include <smpl/heuristic/robot_heuristic.h>

auto std::hash<sbpl::motion::WorkspaceLatticeState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

namespace sbpl {
namespace motion {

std::ostream& operator<<(std::ostream& o, const WorkspaceLatticeState& s)
{
    o << "{ coord: " << s.coord << ", state: " << s.state << ", h: " << s.h << " }";
    return o;
}

template <
    class InputIt,
    class Equal = std::equal_to<typename std::iterator_traits<InputIt>::value_type>>
bool all_equal(InputIt first, InputIt last, typename std::iterator_traits<InputIt>::reference val)
{
    typedef typename std::iterator_traits<InputIt>::reference reference;
    return std::all_of(
            first, last,
            [&val](reference a) { return Equal()(a, val); });
}

WorkspaceLattice::WorkspaceLattice(
    RobotModel* robot,
    CollisionChecker* checker,
    const PlanningParams* params)
:
    WorkspaceLatticeBase(robot, checker, params),
    m_goal_entry(nullptr),
    m_goal_state_id(-1),
    m_start_entry(nullptr),
    m_start_state_id(-1),
    m_state_to_id(),
    m_states(),
    m_t_start(),
    m_expanded_states(),
    m_near_goal(false)
{
    // this should serve as a reasonable dummy state since no valid state should
    // have an empty coordinate vector
    WorkspaceCoord fake_coord;
    m_goal_state_id = createState(fake_coord);
    m_goal_entry = getState(m_goal_state_id);
    ROS_DEBUG_NAMED(params->graph_log, "  goal state has id %d", m_goal_state_id);

    m_ik_amp_enabled = true;
    m_ik_amp_thresh = 0.2;
}

WorkspaceLattice::~WorkspaceLattice()
{
    for (size_t i = 0; i < m_states.size(); i++) {
        delete m_states[i];
        m_states[i] = nullptr;
    }
    m_states.clear();

    // NOTE: StateID2IndexMapping cleared by DiscreteSpaceInformation
}

void WorkspaceLattice::setVisualizationFrameId(const std::string& frame_id)
{
    m_viz_frame_id = frame_id;
}

const std::string& WorkspaceLattice::visualizationFrameId() const
{
    return m_viz_frame_id;
}

bool WorkspaceLattice::init(const Params& _params)
{
    if (!WorkspaceLatticeBase::init(_params)) {
        return false;
    }

    ROS_DEBUG_NAMED(params()->graph_log, "initialize environment");

    if (!initMotionPrimitives()) {
        return false;
    }

    return true;
}

bool WorkspaceLattice::projectToPose(int state_id, Eigen::Affine3d& pose)
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

    WorkspaceLatticeState* state = getState(state_id);

    double p[6];
    poseCoordToWorkspace(&state->coord[0], &p[0]);

    pose = Eigen::Translation3d(p[0], p[1], p[2]) *
            Eigen::AngleAxisd(p[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(p[3], Eigen::Vector3d::UnitX());
    return true;
}

bool WorkspaceLattice::setStart(const RobotState& state)
{
    if (!initialized()) {
        ROS_ERROR_NAMED(params()->graph_log, "cannot set start state on uninitialized lattice");
        return false;
    }

    ROS_DEBUG_NAMED(params()->graph_log, "set the start state");
    if (state.size() < robot()->jointVariableCount()) {
        ROS_ERROR_NAMED(params()->graph_log, "start state contains insufficient coordinate positions");
        return false;
    }

    ROS_DEBUG_NAMED(params()->graph_log, "  angles: %s", to_string(state).c_str());

    if (!robot()->checkJointLimits(state)) {
        ROS_ERROR_NAMED(params()->graph_log, "start state violates joint limits");
        return false;
    }

    double dist;
    if (!collisionChecker()->isStateValid(state, true, false, dist)) {
        ROS_WARN("start state is in collision");
        return false;
    }

    SV_SHOW_INFO(getStateVisualization(state, "start_config"));
    WorkspaceCoord start_coord;
    stateRobotToCoord(state, start_coord);

    m_start_state_id = createState(start_coord);
    m_start_entry = getState(m_start_state_id);
    m_start_entry->state = state;

    return RobotPlanningSpace::setStart(state);
}

bool WorkspaceLattice::setGoal(const GoalConstraint& goal)
{
    bool res = false;
    if (goal.type == GoalType::XYZ_RPY_GOAL) {
        return setGoalPose(goal);
    } else {
        // TODO: set other goals here
        return false;
    }

    return false;
}

int WorkspaceLattice::getStartStateID() const
{
    if (m_start_state_id >= 0 && m_goal_state_id >= 0) {
        WorkspaceLatticeState* start_state = getState(m_start_state_id);
        WorkspaceState cont_state;
        stateCoordToWorkspace(start_state->coord, cont_state);
        if (isGoal(cont_state)) {
            return m_goal_state_id;
        }
    }
    return m_start_state_id;
}

int WorkspaceLattice::getGoalStateID() const
{
    return m_goal_state_id;
}

/*bool WorkspaceLattice::extractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path)
{
    std::vector<geometry_msgs::PoseStamped> eePath;
    return extractPath(ids, path, eePath);
}*/

bool WorkspaceLattice::extractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path, std::vector<geometry_msgs::PoseStamped>& eePath)
{
    path.clear();

    

    if (ids.empty()) {
        return true;
    }

    if (ids.size() == 1) {
        const int state_id = ids[0];
        if (state_id == getGoalStateID()) {
            const WorkspaceLatticeState* entry = getState(m_start_state_id);
            if (!entry) {
                ROS_ERROR_NAMED(params()->graph_log, "Failed to get state entry for state %d", m_start_state_id);
                return false;
            }
            path.push_back(entry->state);
        } else {
            const WorkspaceLatticeState* entry = getState(state_id);
            if (!entry) {
                ROS_ERROR_NAMED(params()->graph_log, "Failed to get state entry for state %d", state_id);
                return false;
            }
            geometry_msgs::PoseStamped eePose;
            WorkspaceState state;
            stateCoordToWorkspace(entry->coord,state);
            extractEEPose(state,eePose);
            path.push_back(entry->state); 
            eePath.push_back(eePose);
        }

        SV_SHOW_INFO(getStateVisualization(path.back(), "goal_state"));
        return true;
    }

    WorkspaceLatticeState* start_entry = getState(ids[0]);
    path.push_back(start_entry->state);

    for (size_t i = 1; i < ids.size(); ++i) {
        const int prev_id = ids[i - 1];
        const int curr_id = ids[i];

        if (prev_id == getGoalStateID()) {
            ROS_ERROR_NAMED(params()->graph_log, "cannot determine goal state successors during path extraction");
            return false;
        }
        //ROS_WARN_STREAM("ID "<<prev_id);
        if (curr_id == getGoalStateID()) {
            // TODO: variant of get succs that returns unique state ids
            WorkspaceLatticeState* prev_entry = getState(prev_id);
            std::vector<Action> actions;
            getActions(*prev_entry, actions);

            WorkspaceLatticeState* best_goal_entry = nullptr;
            int best_cost = std::numeric_limits<int>::max();

            for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
                const Action& action = actions[aidx];

                const WorkspaceState& final_state = action.back();
                if (!isGoal(final_state)) {
                    continue;
                }

                double dist;
                if (!checkAction(prev_entry->state, action, dist)) {
                    continue;
                }

                WorkspaceCoord goal_coord;
                stateWorkspaceToCoord(final_state, goal_coord);

                int goal_id = createState(goal_coord);
                WorkspaceLatticeState* goal_state = getState(goal_id);

                // shouldn't have created a new state, so no need to set the
                // continuous state counterpart
                assert(goal_state->state.size() == robot()->jointVariableCount());

                best_cost = 30; // Hardcoded primitive value in GetSuccs
                best_goal_entry = goal_state;
                break;
            }

            if (!best_goal_entry) {
                ROS_ERROR_NAMED(params()->graph_log, "failed to find valid goal successor during path extraction");
                return false;
            }
            geometry_msgs::PoseStamped eePose;
            WorkspaceState state;
            stateCoordToWorkspace(best_goal_entry->coord,state);
            extractEEPose(state,eePose);
            path.push_back(best_goal_entry->state);
            eePath.push_back(eePose);
        } else {
            WorkspaceLatticeState* state_entry = getState(curr_id);
            geometry_msgs::PoseStamped eePose;
            WorkspaceState state;
            stateCoordToWorkspace(state_entry->coord,state);
            extractEEPose(state,eePose);
            path.push_back(state_entry->state);
            eePath.push_back(eePose);
        }
        
    }
    return true;
}

void WorkspaceLattice::extractEEPose(WorkspaceState currntState, geometry_msgs::PoseStamped& eePose)
{
    eePose.header.frame_id = "world";
    eePose.header.stamp = ros::Time::now();
    eePose.pose.position.x = currntState[0];
    eePose.pose.position.y = currntState[1];
    eePose.pose.position.z = currntState[2];
    eePose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(currntState[3],currntState[4],currntState[5]);
}

Extension* WorkspaceLattice::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<WorkspaceLattice>() ||
        class_code == GetClassCode<RobotPlanningSpace>() ||
        class_code == GetClassCode<PoseProjectionExtension>() ||
        class_code == GetClassCode<PointProjectionExtension>())
    {
        return this;
    }
    return nullptr;
}

void WorkspaceLattice::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    assert(state_id >= 0 && state_id < m_states.size());

    // clear the successor arrays
    succs->clear();
    costs->clear();

    ROS_DEBUG_NAMED(params()->expands_log, "Expand state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    WorkspaceLatticeState* parent_entry = getState(state_id);

    assert(parent_entry);
    assert(parent_entry->coord.size() == m_dof_count);

    ROS_DEBUG_NAMED(params()->expands_log, "  coord: %s", to_string(parent_entry->coord).c_str());
    ROS_DEBUG_NAMED(params()->expands_log, "  state: %s", to_string(parent_entry->state).c_str());

    SV_SHOW_INFO(getStateVisualization(parent_entry->state, "expansion"));

    std::vector<Action> actions;
    getActions(*parent_entry, actions);

    ROS_DEBUG_NAMED(params()->expands_log, "  actions: %zu", actions.size());

    // iterate through successors of source state
    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        ROS_DEBUG_NAMED(params()->expands_log, "    action %zu", i);
        ROS_DEBUG_NAMED(params()->expands_log, "      waypoints: %zu", action.size());

        double dist;
        RobotState final_rstate;
        if (!checkAction(parent_entry->state, action, dist, &final_rstate)) {
            continue;
        }

        
        const WorkspaceState& final_state = action.back();
        WorkspaceCoord succ_coord;
        stateWorkspaceToCoord(final_state, succ_coord);

        // check if hash entry already exists, if not then create one
        int succ_id = createState(succ_coord);
        WorkspaceLatticeState* succ_state = getState(succ_id);
        succ_state->state = final_rstate;

        // check if this state meets the goal criteria
        const bool is_goal_succ = isGoal(final_state);

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_id);
        }

        const int edge_cost = 30;
        costs->push_back(edge_cost);

        ROS_DEBUG_NAMED(params()->expands_log, "      succ: %d", succ_id);
        ROS_DEBUG_NAMED(params()->expands_log, "        coord: %s", to_string(succ_state->coord).c_str());
        ROS_DEBUG_NAMED(params()->expands_log, "        state: %s", to_string(succ_state->state).c_str());
        ROS_DEBUG_NAMED(params()->expands_log, "        cost: %5d", edge_cost);
    }

    m_expanded_states.push_back(state_id);
    //ROS_ERROR_STREAM("Expanded node iD is "<<state_id);
}

void WorkspaceLattice::GetPreds(
    int state_id,
    std::vector<int>* preds,
    std::vector<int>* costs)
{
}

void WorkspaceLattice::PrintState(int state_id, bool verbose, FILE* fout)
{
    assert(state_id >= 0 && state_id < (int)m_states.size());
    if (!fout) {
        fout = stdout;
    }

    WorkspaceLatticeState* state = getState(state_id);

    std::stringstream ss;
    ss << *state;

    if (fout == stdout) {
        ROS_DEBUG_NAMED(params()->graph_log, "%s", ss.str().c_str());
    } else if (fout == stderr) {
        ROS_WARN_NAMED(params()->graph_log, "%s", ss.str().c_str());
    } else {
        fprintf(fout, "%s\n", ss.str().c_str());
    }
}

void WorkspaceLattice::GetLazySuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs,
    std::vector<bool>* true_costs)
{
    assert(state_id >= 0 && state_id < m_states.size());

    succs->clear();
    costs->clear();

    ROS_DEBUG_NAMED(params()->expands_log, "Expand state %d", state_id);

    // goal state should be absorbing
    if (state_id == m_goal_state_id) {
        return;
    }

    WorkspaceLatticeState* state_entry = getState(state_id);

    assert(state_entry);
    assert(state_entry->coord.size() == m_dof_count);

    ROS_DEBUG_NAMED(params()->expands_log, "  coord: %s", to_string(state_entry->coord).c_str());
    ROS_DEBUG_NAMED(params()->expands_log, "  state: %s", to_string(state_entry->state).c_str());

    SV_SHOW_DEBUG(getStateVisualization(state_entry->state, "expansion"));

    std::vector<Action> actions;
    getActions(*state_entry, actions);

    ROS_DEBUG_NAMED(params()->expands_log, "  actions: %zu", actions.size());

    for (size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        ROS_DEBUG_NAMED(params()->expands_log, "    action %zu", i);
        ROS_DEBUG_NAMED(params()->expands_log, "      waypoints: %zu", action.size());

        double dist;
        RobotState final_rstate;
        if (!checkLazyAction(state_entry->state, action, dist, &final_rstate)) {
            continue;
        }

        const WorkspaceState& final_state = action.back();
        WorkspaceCoord succ_coord;
        stateWorkspaceToCoord(final_state, succ_coord);

        // check if hash entry already exists, if not then create one
        int succ_id = createState(succ_coord);
        WorkspaceLatticeState* succ_state = getState(succ_id);
        succ_state->state = final_rstate;

        // check if this state meets the goal criteria
        const bool is_goal_succ = isGoal(final_state);

        // put successor on successor list with the proper cost
        if (is_goal_succ) {
            succs->push_back(m_goal_state_id);
        } else {
            succs->push_back(succ_id);
        }

        const int edge_cost = 30;
        costs->push_back(edge_cost);

        true_costs->push_back(false);

        ROS_DEBUG_NAMED(params()->expands_log, "      succ: %d", succ_id);
        ROS_DEBUG_NAMED(params()->expands_log, "        coord: %s", to_string(succ_state->coord).c_str());
        ROS_DEBUG_NAMED(params()->expands_log, "        state: %s", to_string(succ_state->state).c_str());
        ROS_DEBUG_NAMED(params()->expands_log, "        cost: %5d", 30);
    }
}


void WorkspaceLattice::getExpandedStates(std::vector<RobotState>& states) const 
{
    RobotState state(robot()->jointVariableCount(), 0);

    for (size_t i = 0; i < m_expanded_states.size(); ++i) {
        const WorkspaceLatticeState* entry = getState(m_expanded_states[i]);
        if (entry) {
            states.push_back(entry->state);
        }
        states.push_back(state);
    }
}

int WorkspaceLattice::GetTrueCost(int parent_id, int child_id)
{
    ROS_DEBUG_NAMED(params()->expands_log, "Evaluate cost of transition %d -> %d", parent_id, child_id);

    assert(parent_id >= 0 && parent_id < (int)m_states.size());
    assert(child_id >= 0 && child_id < (int)m_states.size());

    WorkspaceLatticeState* parent_entry = getState(parent_id);
    WorkspaceLatticeState* child_entry = getState(child_id);
    assert(parent_entry && parent_entry->coord.size() == m_dof_count);
    assert(child_entry && child_entry->coord.size() == m_dof_count);

    std::vector<Action> actions;
    getActions(*parent_entry, actions);

    const bool goal_edge = (child_id == m_goal_state_id);

    WorkspaceCoord succ_coord;
    int best_cost = std::numeric_limits<int>::max();
    for (size_t aidx = 0; aidx < actions.size(); ++aidx) {
        const Action& action = actions[aidx];

        stateWorkspaceToCoord(action.back(), succ_coord);

        if (goal_edge) {
            if (!isGoal(action.back())) {
                continue;
            }
        } else {
            if (succ_coord != child_entry->coord) {
                continue;
            }
        }

        double dist;
        if (!checkAction(parent_entry->state, action, dist, nullptr)) {
            continue;
        }

        const int edge_cost = 30;
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


bool WorkspaceLattice::initMotionPrimitives()
{
    m_prims.clear();

    MotionPrimitive prim;

    // create 26-connected position motions
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }

                std::vector<double> d(m_dof_count, 0.0);
                d[0] = m_res[0] * dx;
                d[1] = m_res[1] * dy;
                d[2] = m_res[2] * dz;
                prim.type = MotionPrimitive::Type::LONG_DISTANCE;
                prim.action.clear();
                prim.action.push_back(std::move(d));

                m_prims.push_back(prim);
            }
        }
    }

    // create 2-connected motions for rotation and free angle motions
    for (int a = 3; a < m_dof_count; ++a) {
        std::vector<double> d(m_dof_count, 0.0);

        d[a] = m_res[a] * -1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(d);
        m_prims.push_back(prim);

        d[a] = m_res[a] * 1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(d);
        m_prims.push_back(prim);
    }

    return true;
}

bool WorkspaceLattice::setGoalPose(const GoalConstraint& goal)
{
    if (!initialized()) {
        ROS_ERROR_NAMED(params()->graph_log, "cannot set goal pose on uninitialized lattice");
        return false;
    }

    if (goal.pose.size() != 6) {
        ROS_ERROR("goal element has incorrect format");
        return false;
    }

    if (goal.tgt_off_pose.size() != 6) {
        ROS_ERROR_NAMED(params()->graph_log, "Goal target offset pose has incorrect format");
        return false;
    }

    SV_SHOW_INFO(::viz::getPosesMarkerArray({ goal.tgt_off_pose }, m_viz_frame_id, "target_goal"));

    ROS_DEBUG_NAMED(params()->graph_log, "set the goal state");

    // check if an IK solution exists for the goal pose before we do
    // the search we plan even if there is no solution
    RobotState seed(robot()->jointVariableCount(), 0);
    RobotState ik_solution;
    if (!m_ik_iface->computeIK(goal.pose, seed, ik_solution)) {
        ROS_WARN("No valid IK solution for the goal pose.");
    }

    ROS_DEBUG_NAMED(params()->graph_log, "  xyz (meters): (%0.2f, %0.2f, %0.2f)", goal.pose[0], goal.pose[1], goal.pose[2]);
    ROS_DEBUG_NAMED(params()->graph_log, "  tol (meters): (%0.3f, %0.3f, %0.3f)", goal.xyz_tolerance[0], goal.xyz_tolerance[1], goal.xyz_tolerance[2]);
    ROS_DEBUG_NAMED(params()->graph_log, "  rpy (radians): (%0.2f, %0.2f, %0.2f)", goal.pose[3], goal.pose[4], goal.pose[5]);
    ROS_DEBUG_NAMED(params()->graph_log, "  tol (radians): (%0.3f, %0.3f, %0.3f)", goal.rpy_tolerance[0], goal.rpy_tolerance[1], goal.rpy_tolerance[2]);

    m_expanded_states.clear();
    m_near_goal = false;
    m_t_start = clock::now();

    return RobotPlanningSpace::setGoal(goal);
}

bool WorkspaceLattice::setGoalPoses(const std::vector<PoseGoal>& goals)
{
    return false;
}

/// \brief Create a state entry for a given coordinate and return its id
///
/// If an entry already exists for the coordinate, the id corresponding to that
/// entry is returned; otherwise, a new entry is created and its id returned.
int WorkspaceLattice::createState(const WorkspaceCoord& coord)
{
    WorkspaceLatticeState state;
    state.coord = coord;
    auto sit = m_state_to_id.find(&state);
    if (sit != m_state_to_id.end()) {
        return sit->second;
    }

    int new_id = (int)m_states.size();

    // create a new entry
    WorkspaceLatticeState* state_entry = new WorkspaceLatticeState(state);

    // map id <-> state
    m_states.push_back(state_entry);
    m_state_to_id[state_entry] = new_id;

    int* indices = new int[NUMOFINDICES_STATEID2IND];
    std::fill(indices, indices + NUMOFINDICES_STATEID2IND, -1);
    StateID2IndexMapping.push_back(indices);

    return new_id;
}

/// \brief Retrieve a state by its id.
///
/// The id is not checked for validity and the state is assumed to have already
/// been created, either by GetSuccs during a search or by designating a new
/// start or goal state.
WorkspaceLatticeState* WorkspaceLattice::getState(int state_id) const
{
    assert(state_id >= 0 && state_id < m_states.size());
    return m_states[state_id];
}

bool WorkspaceLattice::isGoal(const WorkspaceState& state) const
{
    // check position
    switch (goal().type) {
    case GoalType::JOINT_STATE_GOAL:
        ROS_WARN_ONCE("WorkspaceLattice joint-space goals not implemented");
        return false;
    case GoalType::XYZ_RPY_GOAL: {
        double dx = std::fabs(state[0] - goal().tgt_off_pose[0]);
        double dy = std::fabs(state[1] - goal().tgt_off_pose[1]);
        double dz = std::fabs(state[2] - goal().tgt_off_pose[2]);
        if (dx <= goal().xyz_tolerance[0] &&
            dy <= goal().xyz_tolerance[1] &&
            dz <= goal().xyz_tolerance[2])
        {
            // log the amount of time required for the search to get close to the goal
            if (!m_near_goal) {
                auto now = clock::now();
                double time_to_goal_region =
                        std::chrono::duration<double>(now - m_t_start).count();
                m_near_goal = true;
                ROS_INFO("search is at the goal position after %0.3f sec", time_to_goal_region);
            }

            Eigen::Quaterniond qg(
                    Eigen::AngleAxisd(goal().tgt_off_pose[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(goal().tgt_off_pose[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(goal().tgt_off_pose[3], Eigen::Vector3d::UnitX()));
            Eigen::Quaterniond q(
                    Eigen::AngleAxisd(state[5], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(state[4], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(state[3], Eigen::Vector3d::UnitX()));

            if (q.dot(qg) < 0.0) {
                qg = Eigen::Quaterniond(-qg.w(), -qg.x(), -qg.y(), -qg.z());
            }

//            const double theta = angles::normalize_angle(Eigen::AngleAxisd(qg.conjugate() * q).angle());
            const double theta = angles::normalize_angle(2.0 * acos(q.dot(qg)));
            if (theta < goal().rpy_tolerance[0]) {
                return true;
            }
        }
        return false;
    }   break;
    case GoalType::XYZ_GOAL: {
        ROS_WARN_ONCE("WorkspaceLattice xyz goals not implemented");
        return false;
    }   break;
    default:
        return false;
    }
}

visualization_msgs::MarkerArray WorkspaceLattice::getStateVisualization(
    const RobotState& state,
    const std::string& ns)
{
    auto ma = collisionChecker()->getCollisionModelVisualization(state);
    for (auto& marker : ma.markers) {
        marker.ns = ns;
    }
    return ma;
}

void WorkspaceLattice::getActions(
    const WorkspaceLatticeState& entry,
    std::vector<Action>& actions)
{
    actions.clear();
    actions.reserve(m_prims.size());

    WorkspaceState cont_state;
    stateCoordToWorkspace(entry.coord, cont_state);

    ROS_DEBUG_STREAM_NAMED(params()->expands_log, "  create actions for state: " << cont_state);

    for (size_t pidx = 0; pidx < m_prims.size(); ++pidx) {
        const auto& prim = m_prims[pidx];

        Action action;
        action.reserve(prim.action.size());

        WorkspaceState final_state = cont_state;
        for (const RobotState& delta_state : prim.action) {
            // increment the state
            for (size_t d = 0; d < m_dof_count; ++d) {
                final_state[d] += delta_state[d];
            }

            normalizeEulerAngles(&final_state[3]);

            action.push_back(final_state);
        }

        actions.push_back(std::move(action));
    }

    if (m_ik_amp_enabled && numHeuristics() > 0) {
        RobotHeuristic* h = heuristic(0);
        double goal_dist = h->getMetricGoalDistance(
                cont_state[0], cont_state[1], cont_state[2]);
        if (goal_dist < m_ik_amp_thresh) {
            std::vector<double> ik_sol;
            if (m_ik_iface->computeIK(goal().tgt_off_pose, entry.state, ik_sol)) {
                WorkspaceState final_state;
                stateRobotToWorkspace(ik_sol, final_state);
                Action action(1);
                action[0] = final_state;
                actions.push_back(std::move(action));
            }
        }
    }
}

bool WorkspaceLattice::checkAction(
    const RobotState& state,
    const Action& action,
    double& dist,
    RobotState* final_rstate)
{
    std::vector<RobotState> wptraj;
    wptraj.reserve(action.size());

    std::uint32_t violation_mask = 0x00000000;

    // check waypoints for ik solutions and joint limits
    for (size_t widx = 0; widx < action.size(); ++widx) {
        const WorkspaceState& istate = action[widx];

        ROS_DEBUG_NAMED(params()->expands_log, "        %zu: %s", widx, to_string(istate).c_str());

        RobotState irstate;
        if (!stateWorkspaceToRobot(istate, state, irstate)) {
            ROS_DEBUG_NAMED(params()->expands_log, "         -> failed to find ik solution");
            violation_mask |= 0x00000001;
            break;
        }

        wptraj.push_back(irstate);

        if (!robot()->checkJointLimits(irstate)) {
            ROS_DEBUG_NAMED(params()->expands_log, "        -> violates joint limits");
            violation_mask |= 0x00000002;
            break;
        }
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions between the waypoints
    assert(wptraj.size() == action.size());

    int plen = 0;
    int nchecks = 0;
    if (!collisionChecker()->isStateToStateValid(state, wptraj[0], plen, nchecks, dist)) {
        ROS_DEBUG_NAMED(params()->expands_log, "        -> path to first waypoint in collision");
        violation_mask |= 0x00000004;
    }

    if (violation_mask) {
        return false;
    }

    for (size_t widx = 1; widx < wptraj.size(); ++widx) {
        const RobotState& prev_istate = wptraj[widx - 1];
        const RobotState& curr_istate = wptraj[widx];
        if (!collisionChecker()->isStateToStateValid(prev_istate, curr_istate, plen, nchecks, dist)) {
            ROS_DEBUG_NAMED(params()->expands_log, "        -> path between waypoints in collision");
            violation_mask |= 0x00000008;
            break;
        }
    }

    if (violation_mask) {
        return false;
    }

    if (final_rstate) {
        *final_rstate = wptraj.back();
    }
    return true;
}

bool WorkspaceLattice::checkLazyAction(
    const RobotState& state,
    const Action& action,
    double& dist,
    RobotState* final_rstate)
{
    std::vector<RobotState> wptraj;
    wptraj.reserve(action.size());

    std::uint32_t violation_mask = 0x00000000;

    // check waypoints for ik solutions and joint limits
    for (size_t widx = 0; widx < action.size(); ++widx) {
        const WorkspaceState& istate = action[widx];

        ROS_DEBUG_NAMED(params()->expands_log, "        %zu: %s", widx, to_string(istate).c_str());

        RobotState irstate;
        if (!stateWorkspaceToRobot(istate, state, irstate)) {
            ROS_DEBUG_NAMED(params()->expands_log, "         -> failed to find ik solution");
            violation_mask |= 0x00000001;
            break;
        }

        wptraj.push_back(irstate);

        if (!robot()->checkJointLimits(irstate)) {
            ROS_DEBUG_NAMED(params()->expands_log, "        -> violates joint limits");
            violation_mask |= 0x00000002;
            break;
        }
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions between the waypoints
    assert(wptraj.size() == action.size());

    if (final_rstate) {
        *final_rstate = wptraj.back();
    }
    return true;
}

#if !BROKEN

bool WorkspaceLattice::getMotionPrimitive(
    WorkspaceLatticeState* parent,
    MotionPrimitive& mp)
{
    if (mp.type == SNAP_TO_RPY) {
        if (parent->h > prms_.cost_per_cell * 6) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_RPY, parent, mp);
    } else if (mp.type == SNAP_TO_XYZRPY) {
        if (parent->h > prms_.cost_per_cell * 10) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_XYZRPY, parent, mp);
    } else if (mp.type == SNAP_TO_XYZ_THEN_TO_RPY) {
        if (parent->h > prms_.cost_per_cell * 15) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_XYZ_THEN_TO_RPY, parent, mp);
    } else if (mp.type == SNAP_TO_RPY_THEN_TO_XYZ) {
        if (parent->h > prms_.cost_per_cell * 15) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_RPY_THEN_TO_XYZ, parent, mp);
    } else if (mp.type == SNAP_TO_RPY_AT_START) {
        if (parent->h < prms_.cost_per_cell * 3) {
            return false;
        }
        getAdaptiveMotionPrim(SNAP_TO_RPY_AT_START, parent, mp);
    } else if (mp.type == RETRACT_THEN_SNAP_TO_RPY_THEN_TO_XYZ) {
        if (parent->h > prms_.cost_per_cell * 20) {
            return false;
        }
        int x, y, z;
        if (!getDistanceGradient(x, y, z)) {
            return false;
        }
        getAdaptiveMotionPrim(RETRACT_THEN_SNAP_TO_RPY_THEN_TO_XYZ, parent, mp);
    } else if (mp.type == RETRACT_THEN_TOWARDS_RPY_THEN_TOWARDS_XYZ) {
        if (m_goal_entry->coord[3] == parent->coord[3] &&
            m_goal_entry->coord[4] == parent->coord[4] &&
            m_goal_entry->coord[5] == parent->coord[5])
        {
            ROS_DEBUG_NAMED(params()->expands_log, "Already at goal rpy. Not doing the retract motion.");
            return false;
        }

        int x, y, z;
        if (!getDistanceGradient(x, y, z)) {
            ROS_ERROR("Zero GRadient");
            return false;
        }

        getAdaptiveMotionPrim(RETRACT_THEN_TOWARDS_RPY_THEN_TOWARDS_XYZ, parent, mp);
    } else {
        ROS_WARN("Invalid motion primitive type");
    }

    if (all_equal(mp.coord.begin(), mp.coord.end(), 0)) {
        ROS_DEBUG_NAMED(params()->graph_log, "Not using adaptive mprim cause its all zeros. We should be at goal??? (type: %s)", to_string(mp.type).c_str());
        return false;
    }

    return true;
}

void WorkspaceLattice::getAdaptiveMotionPrim(
    int type,
    WorkspaceLatticeState* parent,
    MotionPrimitive& mp)
{
    mp.m.clear();
    mp.coord.clear();
    mp.coord.resize(m_dof_count, 0);

    // set rpy to motion required to satisfy goal orientation
    if (type == SNAP_TO_RPY) {
        mp.m.resize(1, std::vector<double>(m_dof_count, 0.0));
        mp.m[0][3] = angles::normalize_angle(double(m_goal_entry->coord[3] - parent->coord[3]) * m_res[3]);
        mp.m[0][4] = angles::normalize_angle(double(m_goal_entry->coord[4] - parent->coord[4]) * m_res[4]);
        mp.m[0][5] = angles::normalize_angle(double(m_goal_entry->coord[5] - parent->coord[5]) * m_res[5]);
        mp.coord[3] = m_goal_entry->coord[3] - parent->coord[3];
        mp.coord[4] = m_goal_entry->coord[4] - parent->coord[4];
        mp.coord[5] = m_goal_entry->coord[5] - parent->coord[5];
    } else if (type == SNAP_TO_XYZRPY) {
        // snap to goal pose in one motion
        mp.m.resize(1, std::vector<double>(m_dof_count, 0.0));
        mp.m[0][0] = double(m_goal_entry->coord[0] - parent->coord[0]) * m_res[0];
        mp.m[0][1] = double(m_goal_entry->coord[1] - parent->coord[1]) * m_res[1];
        mp.m[0][2] = double(m_goal_entry->coord[2] - parent->coord[2]) * m_res[2];
        mp.m[0][3] = angles::normalize_angle(double(m_goal_entry->coord[3] - parent->coord[3]) * m_res[3]);
        mp.m[0][4] = angles::normalize_angle(double(m_goal_entry->coord[4] - parent->coord[4]) * m_res[4]);
        mp.m[0][5] = angles::normalize_angle(double(m_goal_entry->coord[5] - parent->coord[5]) * m_res[5]);
        mp.coord[0] = m_goal_entry->coord[0] - parent->coord[0];
        mp.coord[1] = m_goal_entry->coord[1] - parent->coord[1];
        mp.coord[2] = m_goal_entry->coord[2] - parent->coord[2];
        mp.coord[3] = m_goal_entry->coord[3] - parent->coord[3];
        mp.coord[4] = m_goal_entry->coord[4] - parent->coord[4];
        mp.coord[5] = m_goal_entry->coord[5] - parent->coord[5];
    } else if (type == SNAP_TO_RPY_THEN_TO_XYZ) {
        // first satisfy goal orientation in place, then move into the goal position
        mp.m.resize(2, std::vector<double>(m_dof_count, 0.0));
        mp.m[0][3] = angles::normalize_angle(double(m_goal_entry->coord[3] - parent->coord[3]) * m_res[3]);
        mp.m[0][4] = angles::normalize_angle(double(m_goal_entry->coord[4] - parent->coord[4]) * m_res[4]);
        mp.m[0][5] = angles::normalize_angle(double(m_goal_entry->coord[5] - parent->coord[5]) * m_res[5]);
        mp.m[1][0] = double(m_goal_entry->coord[0] - parent->coord[0]) * m_res[0];
        mp.m[1][1] = double(m_goal_entry->coord[1] - parent->coord[1]) * m_res[1];
        mp.m[1][2] = double(m_goal_entry->coord[2] - parent->coord[2]) * m_res[2];
        mp.coord[0] = m_goal_entry->coord[0] - parent->coord[0];
        mp.coord[1] = m_goal_entry->coord[1] - parent->coord[1];
        mp.coord[2] = m_goal_entry->coord[2] - parent->coord[2];
        mp.coord[3] = m_goal_entry->coord[3] - parent->coord[3];
        mp.coord[4] = m_goal_entry->coord[4] - parent->coord[4];
        mp.coord[5] = m_goal_entry->coord[5] - parent->coord[5];
    } else if (type == SNAP_TO_XYZ_THEN_TO_RPY) {
        // first move to the goal position, then rotate into the goal orientation
        mp.m.resize(2, std::vector<double>(m_dof_count, 0.0));
        mp.m[0][0] = double(m_goal_entry->coord[0] - parent->coord[0]) * m_res[0];
        mp.m[0][1] = double(m_goal_entry->coord[1] - parent->coord[1]) * m_res[1];
        mp.m[0][2] = double(m_goal_entry->coord[2] - parent->coord[2]) * m_res[2];
        mp.m[1][3] = angles::normalize_angle(double(m_goal_entry->coord[3] - parent->coord[3]) * m_res[3]);
        mp.m[1][4] = angles::normalize_angle(double(m_goal_entry->coord[4] - parent->coord[4]) * m_res[4]);
        mp.m[1][5] = angles::normalize_angle(double(m_goal_entry->coord[5] - parent->coord[5]) * m_res[5]);
        mp.coord[0] = m_goal_entry->coord[0] - parent->coord[0];
        mp.coord[1] = m_goal_entry->coord[1] - parent->coord[1];
        mp.coord[2] = m_goal_entry->coord[2] - parent->coord[2];
        mp.coord[3] = m_goal_entry->coord[3] - parent->coord[3];
        mp.coord[4] = m_goal_entry->coord[4] - parent->coord[4];
        mp.coord[5] = m_goal_entry->coord[5] - parent->coord[5];
    } else if (type == SNAP_TO_RPY_AT_START) {
        mp.m.resize(1, std::vector<double>(m_dof_count, 0.0));
        getVector(
                m_goal_entry->coord[0], m_goal_entry->coord[1],
                m_goal_entry->coord[2], parent->coord[0], parent->coord[1], parent->coord[2], mp.coord[0],
                mp.coord[1], mp.coord[2], 5);
        getVector(
                m_goal_entry->coord[3], m_goal_entry->coord[4],
                m_goal_entry->coord[5], parent->coord[3], parent->coord[4], parent->coord[5], mp.coord[3],
                mp.coord[4], mp.coord[5], 10);

        mp.m[0][0] = double(mp.coord[0]) * m_res[0];
        mp.m[0][1] = double(mp.coord[1]) * m_res[1];
        mp.m[0][2] = double(mp.coord[2]) * m_res[2];
        mp.m[0][3] = angles::normalize_angle(double(mp.coord[3]) * m_res[3]);
        mp.m[0][4] = angles::normalize_angle(double(mp.coord[4]) * m_res[4]);
        mp.m[0][5] = angles::normalize_angle(double(mp.coord[5]) * m_res[5]);
        ROS_DEBUG_NAMED(params()->expands_log, "     [snap_to_rpy_at_start] xyz-coord: %d %d %d  rpy-coord: %d %d %d  fa-coord: %d", mp.coord[0], mp.coord[1], mp.coord[2], mp.coord[3], mp.coord[4], mp.coord[5], mp.coord[6]);
        ROS_DEBUG_NAMED(params()->expands_log, "     [snap_to_rpy_at_start] xyz: %0.3f %0.3f %0.3f    rpy: %0.3f %0.3f %0.3f   fa: %0.3f   (parent-dist: %d)", mp.m[0][0], mp.m[0][1], mp.m[0][2], mp.m[0][3], mp.m[0][4], mp.m[0][5], mp.m[0][6], parent->h);
    } else if (type == RETRACT_THEN_SNAP_TO_RPY_THEN_TO_XYZ) {
        int x, y, z;
        std::vector<int> icoord(m_dof_count, 0);
        if (!getDistanceGradient(x, y, z)) {
            ROS_ERROR("I shouldn't be here...");
        }

        // get xyz for retracted pose
        getVector(
                parent->coord[0] + x, parent->coord[1] + y, parent->coord[2] + z,
                parent->coord[0], parent->coord[1], parent->coord[2],
                icoord[0], icoord[1], icoord[2],
                6, false);
        mp.m.resize(3, std::vector<double>(m_dof_count, 0.0));
        mp.m[0][0] = double(icoord[0]) * m_res[0];
        mp.m[0][1] = double(icoord[1]) * m_res[1];
        mp.m[0][2] = double(icoord[2]) * m_res[2];
        mp.m[1][3] = angles::normalize_angle(double(m_goal_entry->coord[3] - parent->coord[3]) * m_res[3]);
        mp.m[1][4] = angles::normalize_angle(double(m_goal_entry->coord[4] - parent->coord[4]) * m_res[4]);
        mp.m[1][5] = angles::normalize_angle(double(m_goal_entry->coord[5] - parent->coord[5]) * m_res[5]);
        mp.m[2][0] = double(m_goal_entry->coord[0] - parent->coord[0]) * m_res[0];
        mp.m[2][1] = double(m_goal_entry->coord[1] - parent->coord[1]) * m_res[1];
        mp.m[2][2] = double(m_goal_entry->coord[2] - parent->coord[2]) * m_res[2];
        mp.coord[0] = m_goal_entry->coord[0] - parent->coord[0];
        mp.coord[1] = m_goal_entry->coord[1] - parent->coord[1];
        mp.coord[2] = m_goal_entry->coord[2] - parent->coord[2];
        mp.coord[3] = m_goal_entry->coord[3] - parent->coord[3];
        mp.coord[4] = m_goal_entry->coord[4] - parent->coord[4];
        mp.coord[5] = m_goal_entry->coord[5] - parent->coord[5];

        // debugging
        if (mp.m.size() == 3) {
            std::vector<double> wcoord(m_dof_count, 0), gwcoord(m_dof_count, 0);
            coordToWorldPose(parent->coord, wcoord);
            coordToWorldPose(m_goal_entry->coord, gwcoord);
            ROS_DEBUG_NAMED(params()->expands_log, " [distance_gradiant] %d %d %d", x, y, z);
            ROS_DEBUG_NAMED(params()->expands_log, "          [parent]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", wcoord[0], wcoord[1], wcoord[2], wcoord[3], wcoord[4], wcoord[5], wcoord[6], parent->h);
            ROS_DEBUG_NAMED(params()->expands_log, "    [diff-to-goal]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", gwcoord[0] - wcoord[0], gwcoord[1] - wcoord[1], gwcoord[2] - wcoord[2], gwcoord[3] - wcoord[3], gwcoord[4] - wcoord[4], gwcoord[5] - wcoord[5], gwcoord[6] - wcoord[6], parent->h);
            ROS_DEBUG_NAMED(params()->expands_log, "         [retract]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[0][0], mp.m[0][1], mp.m[0][2], mp.m[0][3], mp.m[0][4], mp.m[0][5], mp.m[0][6], parent->h);
            ROS_DEBUG_NAMED(params()->expands_log, "     [towards-rpy]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[1][0], mp.m[1][1], mp.m[1][2], mp.m[1][3], mp.m[1][4], mp.m[1][5], mp.m[1][6], parent->h);
            ROS_DEBUG_NAMED(params()->expands_log, "     [towards-xyz]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[2][0], mp.m[2][1], mp.m[2][2], mp.m[2][3], mp.m[2][4], mp.m[2][5], mp.m[2][6], parent->h);
        } else {
            std::vector<double> wcoord(m_dof_count, 0), gwcoord(m_dof_count, 0);
            coordToWorldPose(parent->coord, wcoord);
            coordToWorldPose(m_goal_entry->coord, gwcoord);
            ROS_DEBUG_NAMED(params()->expands_log, " [distance_gradiant] %d %d %d", x, y, z);
            ROS_DEBUG_NAMED(params()->expands_log, "          [parent]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", wcoord[0], wcoord[1], wcoord[2], wcoord[3], wcoord[4], wcoord[5], wcoord[6], parent->h);
            ROS_DEBUG_NAMED(params()->expands_log, "    [diff-to-goal]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", gwcoord[0] - wcoord[0], gwcoord[1] - wcoord[1], gwcoord[2] - wcoord[2], gwcoord[3] - wcoord[3], gwcoord[4] - wcoord[4], gwcoord[5] - wcoord[5], gwcoord[6] - wcoord[6], parent->h);
            ROS_DEBUG_NAMED(params()->expands_log, "         [retract]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[0][0], mp.m[0][1], mp.m[0][2], mp.m[0][3], mp.m[0][4], mp.m[0][5], mp.m[0][6], parent->h);
            ROS_DEBUG_NAMED(params()->expands_log, "     [towards-rpy]  -- ");
            ROS_DEBUG_NAMED(params()->expands_log, "     [towards-xyz]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[1][0], mp.m[1][1], mp.m[1][2], mp.m[1][3], mp.m[1][4], mp.m[1][5], mp.m[1][6], parent->h);
        }
    } else if (type == RETRACT_THEN_TOWARDS_RPY_THEN_TOWARDS_XYZ) {
        int x, y, z;
        std::vector<int> icoord(m_dof_count, 0);
        if (!getDistanceGradient(x, y, z)) {
            ROS_ERROR("I shouldn't be here...");
        }

        // get xyz for retracted pose
        getVector(
                parent->coord[0] + x, parent->coord[1] + y, parent->coord[2] + z,
                parent->coord[0], parent->coord[1],
                parent->coord[2], icoord[0], icoord[1],
                icoord[2], 4, false);

        // rotate towards rpy
        getVector(
                m_goal_entry->coord[3], m_goal_entry->coord[4], m_goal_entry->coord[5],
                parent->coord[3], parent->coord[4], parent->coord[5],
                mp.coord[3], mp.coord[4], mp.coord[5], 15);

        std::vector<double> im(m_dof_count, 0);
        mp.m.resize(1, std::vector<double>(m_dof_count, 0.0));
        mp.m[0][0] = double(icoord[0]) * m_res[0];
        mp.m[0][1] = double(icoord[1]) * m_res[1];
        mp.m[0][2] = double(icoord[2]) * m_res[2];

        if (mp.coord[3] != 0 || mp.coord[4] != 0 || mp.coord[5] != 0) {
            im[3] = angles::normalize_angle(double(mp.coord[3]) * m_res[3]);
            im[4] = angles::normalize_angle(double(mp.coord[4]) * m_res[4]);
            im[5] = angles::normalize_angle(double(mp.coord[5]) * m_res[5]);
            mp.m.push_back(im);
        }
        std::vector<double> wcoord(m_dof_count, 0), gwcoord(m_dof_count, 0);
        coordToWorldPose(parent->coord, wcoord);
        coordToWorldPose(m_goal_entry->coord, gwcoord);
        ROS_DEBUG_NAMED(params()->expands_log, " [distance_gradiant] %d %d %d", x, y, z);
        ROS_DEBUG_NAMED(params()->expands_log, "          [parent]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", wcoord[0], wcoord[1], wcoord[2], wcoord[3], wcoord[4], wcoord[5], wcoord[6], parent->h);
        ROS_DEBUG_NAMED(params()->expands_log, "    [diff-to-goal]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", gwcoord[0] - wcoord[0], gwcoord[1] - wcoord[1], gwcoord[2] - wcoord[2], gwcoord[3] - wcoord[3], gwcoord[4] - wcoord[4], gwcoord[5] - wcoord[5], gwcoord[6] - wcoord[6], parent->h);
        ROS_DEBUG_NAMED(params()->expands_log, "         [retract]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[0][0], mp.m[0][1], mp.m[0][2], mp.m[0][3], mp.m[0][4], mp.m[0][5], mp.m[0][6], parent->h);
        ROS_DEBUG_NAMED(params()->expands_log, "     [towards-rpy]  xyz: %2.2f %2.2f %2.2f    rpy: %2.2f %2.2f %2.2f   fa: %2.2f   (parent-dist: %d)", mp.m[1][0], mp.m[1][1], mp.m[1][2], mp.m[1][3], mp.m[1][4], mp.m[1][5], mp.m[1][6], parent->h);
    } else {
        ROS_WARN("Invalid adaptive motion primitive type.");
    }
}

void WorkspaceLattice::getVector(
    int x1, int y1, int z1,
    int x2, int y2, int z2,
    int& xout, int& yout, int& zout,
    int multiplier,
    bool snap)
{
    double dx = x1 - x2;
    double dy = y1 - y2;
    double dz = z1 - z2;
    double length = sqrt(dx * dx + dy * dy + dz * dz);

    if (!snap) {
        if (dx != 0) {
            xout = (dx * multiplier) / length + 0.5;
        } else {
            xout = 0;
        }

        if (dy != 0) {
            yout = (dy * multiplier) / length + 0.5;
        } else {
            yout = 0;
        }

        if (dz != 0) {
            zout = (dz * multiplier) / length + 0.5;
        } else {
            zout = 0;
        }
    } else {
        if (dx > 0) {
            xout = min((dx * multiplier) / length + 0.5, dx);
        } else if (dx < 0) {
            xout = max((dx * multiplier) / length + 0.5, dx);
        } else {
            xout = 0;
        }

        if (dy > 0) {
            yout = min((dy * multiplier) / length + 0.5, dy);
        } else if (dy < 0) {
            yout = max((dy * multiplier) / length + 0.5, dy);
        } else {
            yout = 0;
        }

        if (dz > 0) {
            zout = min((dz * multiplier) / length + 0.5, dz);
        } else if (dz < 0) {
            zout = max((dz * multiplier) / length + 0.5, dz);
        } else {
            zout = 0;
        }
    }
    ROS_DEBUG_NAMED(params()->graph_log, "xyz1: %d %d %d    xyz2: %d %d %d", x1, y1, z1, x2, y2, z2);
    ROS_DEBUG_NAMED(params()->graph_log, "dx: %1.2f dy: %1.2f dz: %1.2f length: %1.2f multiplier: %d unit_double{%1.2f %1.2f %1.2f}  unit_int{%d %d %d}", dx, dy, dz, length, multiplier, dx / length, dy / length, dz / length, xout, yout, zout);
}

bool WorkspaceLattice::getDistanceGradient(int& x, int& y, int& z)
{
    mp_gradient_[0] = mp_dist_[0] - mp_dist_[1];
    mp_gradient_[1] = mp_dist_[2] - mp_dist_[3];
    mp_gradient_[2] = mp_dist_[4] - mp_dist_[5];

    x = mp_gradient_[0];
    y = mp_gradient_[1];
    z = mp_gradient_[2];

    if (mp_dist_[0] == -1 || mp_dist_[1] == -1) {
        mp_gradient_[0] = 0;
        return false;
    }
    if (mp_dist_[2] == -1 || mp_dist_[3] == -1) {
        mp_gradient_[1] = 0;
        return false;
    }
    if (mp_dist_[4] == -1 || mp_dist_[5] == -1) {
        mp_gradient_[2] = 0;
        return false;
    }
    if (mp_gradient_[0] == 0 && mp_gradient_[1] == 0 && mp_gradient_[2] == 0) {
        return false;
    }

    double norm = sqrt(mp_gradient_[0] * mp_gradient_[0] + mp_gradient_[1] * mp_gradient_[1] + mp_gradient_[2] * mp_gradient_[2]);
    ROS_INFO("gradient_x: %2.2f   gradient_y: %2.2f   gradient_z: %2.2f  norm: %2.2f", mp_gradient_[0], mp_gradient_[1], mp_gradient_[2], norm);
    return true;
}

#endif // BROKEN

} // namespace motion
} // namespace sbpl
