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

#include <smpl/graph/adaptive_workspace_lattice.h>

// system includes
#include <boost/functional/hash.hpp>
#include <sbpl/planners/planner.h>

// project includes
#include <smpl/angles.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/heuristic/robot_heuristic.h>

auto std::hash<sbpl::motion::AdaptiveGridState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, std::hash<decltype(s.gx)>()(s.gx));
    boost::hash_combine(seed, std::hash<decltype(s.gy)>()(s.gy));
    boost::hash_combine(seed, std::hash<decltype(s.gz)>()(s.gz));
    return seed;
}

auto std::hash<sbpl::motion::AdaptiveWorkspaceState>::operator()(
    const argument_type& s) const -> result_type
{
    size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.coord.begin(), s.coord.end()));
    return seed;
}

namespace sbpl {
namespace motion {

std::ostream& operator<<(std::ostream& o, const AdaptiveGridState& s)
{
    o << "{ x: " << s.gx << ", y: " << s.gy << ", z: " << s.gy << " }";
    return o;
}

std::ostream& operator<<(std::ostream& o, const AdaptiveWorkspaceState& s)
{
    o << "{ coord: " << s.coord << ", state: " << s.state << " }";
    return o;
}

AdaptiveWorkspaceLattice::~AdaptiveWorkspaceLattice()
{
    for (AdaptiveState* state : m_states) {
        if (state->hid) {
            AdaptiveWorkspaceState* hi_state = (AdaptiveWorkspaceState*)state;
            delete hi_state;
        } else {
            AdaptiveGridState* lo_state = (AdaptiveGridState*)state;
            delete lo_state;
        }
    }

    // NOTE: StateID2IndexMapping cleared by DiscreteSpaceInformation
}

bool AdaptiveWorkspaceLattice::init(
    RobotModel* _robot,
    CollisionChecker* checker,
    const PlanningParams* pp,
    const Params& _params,
    const OccupancyGrid* grid)
{
    if (!WorkspaceLatticeBase::init(_robot, checker, pp, _params)) {
        return false;
    }

    m_grid = grid;

    m_dim_grid.assign(
            m_grid->numCellsX(),
            m_grid->numCellsY(),
            m_grid->numCellsZ(),
            AdaptiveGridCell());

    m_goal_state_id = reserveHashEntry(true);
    m_goal_state = getHashEntry(m_goal_state_id);
    SMPL_DEBUG_NAMED(pp->graph_log, " goal state has state ID %d", m_goal_state_id);

    if (!initMotionPrimitives()) {
        return false;
    }

    return true;
}

bool AdaptiveWorkspaceLattice::projectToPoint(
    int state_id,
    Eigen::Vector3d& pos)
{
    AdaptiveState* state = m_states[state_id];
    if (state_id == m_goal_state_id) {
        assert(goal().tgt_off_pose.size() >= 3);
        pos.x() = goal().tgt_off_pose[0];
        pos.y() = goal().tgt_off_pose[1];
        pos.z() = goal().tgt_off_pose[2];
    } else if (state->hid) {
        AdaptiveWorkspaceState* hi_state = (AdaptiveWorkspaceState*)state;
        WorkspaceState state;
        stateCoordToWorkspace(hi_state->coord, state);
        pos.x() = state[0];
        pos.y() = state[1];
        pos.z() = state[2];
    } else {
        AdaptiveGridState* lo_state = (AdaptiveGridState*)state;
        pos.x() = lo_state->x;
        pos.y() = lo_state->y;
        pos.z() = lo_state->z;
    }

    return true;
}

bool AdaptiveWorkspaceLattice::addHighDimRegion(int state_id)
{
    AdaptiveState* state = m_states[state_id];

    Eigen::Vector3i gp;

    if (state_id == m_goal_state_id) {
        SMPL_INFO_NAMED(params()->graph_log, "Skip adding high-dimensional region around goal");
        m_grid->worldToGrid(
                goal().tgt_off_pose[0], goal().tgt_off_pose[1], goal().tgt_off_pose[2],
                gp.x(), gp.y(), gp.z());
    } else if (state->hid) {
        SMPL_INFO_NAMED(params()->graph_log, "Grow high-dimensional region around hi state %d", state_id);
        AdaptiveWorkspaceState* hi_state = (AdaptiveWorkspaceState*)state;
        WorkspaceState work_state;
        stateCoordToWorkspace(hi_state->coord, work_state);
        m_grid->worldToGrid(
                work_state[0], work_state[1], work_state[2],
                gp.x(), gp.y(), gp.z());
    } else {
        SMPL_INFO_NAMED(params()->graph_log, "Add high-dimensional region around lo state %d", state_id);
        // add/grow hd region around ld state
        AdaptiveGridState* lo_state = (AdaptiveGridState*)state;
        m_grid->worldToGrid(lo_state->x, lo_state->y, lo_state->z, gp.x(), gp.y(), gp.z());
    }

    SMPL_INFO_NAMED(params()->graph_log, "Region center: (%d, %d, %d)", gp.x(), gp.y(), gp.z());

    if (!m_grid->isInBounds(gp.x(), gp.y(), gp.z())) {
        SMPL_INFO_NAMED(params()->graph_log, " -> out of bounds");
        return false;
    }

    ++m_dim_grid(gp.x(), gp.y(), gp.z()).grow_count;
    const int radius = m_region_radius * m_dim_grid(gp.x(), gp.y(), gp.z()).grow_count;
    SMPL_INFO_NAMED(params()->graph_log, "  radius: %d", radius);

    // TODO: mark cells as in high-dimensional region
    int marked = 0;
    for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
    for (int dz = -radius; dz <= radius; ++dz) {
        Eigen::Vector3i p = gp + Eigen::Vector3i(dx, dy, dz);
        if (m_grid->isInBounds(p.x(), p.y(), p.z())) {
            m_dim_grid(p.x(), p.y(), p.z()).plan_hd = true;
            ++marked;
        }
    }
    }
    }

    SMPL_INFO_NAMED(params()->graph_log, "Marked %d cells as high-dimensional", marked);

    return true;
}

bool AdaptiveWorkspaceLattice::setTunnel(const std::vector<int>& states)
{
    // clear the tunnel grid
    // TODO: retain the list of points in the tunnel and clear only those points
    for (auto it = m_dim_grid.begin(); it != m_dim_grid.end(); ++it) {
        it->trak_hd = false;
    }

    std::vector<Eigen::Vector3i> tunnel;
    for (int state_id : states) {
        double px, py, pz;

        AdaptiveState* state = m_states[state_id];

        if (state_id == m_goal_state_id) {
            px = goal().tgt_off_pose[0];
            py = goal().tgt_off_pose[1];
            pz = goal().tgt_off_pose[2];
        }
        else if (state->hid) {
            AdaptiveWorkspaceState* hi_state = (AdaptiveWorkspaceState*)state;
            WorkspaceState wstate;
            stateCoordToWorkspace(hi_state->coord, wstate);
            px = wstate[0];
            py = wstate[1];
            pz = wstate[2];
        } else {
            AdaptiveGridState* lo_state = (AdaptiveGridState*)state;
            px = lo_state->x;
            py = lo_state->y;
            pz = lo_state->z;
        }

        int gx, gy, gz;
        m_grid->worldToGrid(px, py, pz, gx, gy, gz);

        if (!m_grid->isInBounds(gx, gy, gz)) {
            SMPL_ERROR_NAMED(params()->graph_log, "Failed to create tunnel. State (%d, %d, %d) out of bounds", gx, gy, gz);
            return false;
        }

        tunnel.emplace_back(gx, gy, gz);
    }

    // TODO: dijkstra/breadth-first search out from tunnel to fill states at
    // tunnel-width away
    int marked = 0;
    for (const Eigen::Vector3i& gp : tunnel) {
        const int radius = m_tunnel_radius;

        for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
        for (int dz = -radius; dz <= radius; ++dz) {
            Eigen::Vector3i p = gp + Eigen::Vector3i(dx, dy, dz);
            if (m_grid->isInBounds(p.x(), p.y(), p.z())) {
                m_dim_grid(p.x(), p.y(), p.z()).trak_hd = true;
                ++marked;
            }
        }
        }
        }
    }
    SMPL_INFO_NAMED(params()->graph_log, "Marked %d cells as tunnel cells", marked);

    return true;
}

bool AdaptiveWorkspaceLattice::isExecutable(
    const std::vector<int>& states) const
{
    for (int state_id : states) {
        AdaptiveState* state = getHashEntry(state_id);
        if (!state->hid) {
            return false;
        }
    }
    return true;
}

bool AdaptiveWorkspaceLattice::setTrackMode(const std::vector<int>& tunnel)
{
    if (!setTunnel(tunnel)) {
        return false;
    }
    m_plan_mode = false;
    SV_SHOW_INFO(getAdaptiveGridVisualization(m_plan_mode));
    return true;
}

bool AdaptiveWorkspaceLattice::setPlanMode()
{
    m_plan_mode = true;
    SV_SHOW_INFO(getAdaptiveGridVisualization(m_plan_mode));
    return true;
}

int AdaptiveWorkspaceLattice::getStartStateID() const
{
    return m_start_state_id;
}

int AdaptiveWorkspaceLattice::getGoalStateID() const
{
    return m_goal_state_id;
}

bool AdaptiveWorkspaceLattice::extractPath(
    const std::vector<int>& ids,
    std::vector<RobotState>& path) //, std::vector<geometry_msgs::PoseStamped>& eePath)
{
    path.clear();

    if (ids.empty()) {
        return true;
    }

    AdaptiveWorkspaceState* start_state = getHiHashEntry(ids[0]);
    if (!start_state) {
        SMPL_ERROR_NAMED(params()->graph_log, "Start state is not high-dimensional");
        return false;
    }

    path.push_back(start_state->state);

    for (std::size_t i = 1; i < ids.size(); ++i) {
        const int prev_id = ids[i - 1];
        const int curr_id = ids[i];

        if (prev_id == getGoalStateID()) {
            SMPL_ERROR_NAMED(params()->graph_log, "Cannot determine goal state successors during path extraction");
            return false;
        }

        if (curr_id == getGoalStateID()) {
            AdaptiveWorkspaceState* prev_state = getHiHashEntry(prev_id);
            if (!prev_state) {
                SMPL_ERROR_NAMED(params()->graph_log, "Intermediate state is not high-dimensional");
                return false;
            }

            std::vector<Action> actions;
            getActions(*prev_state, actions);

            AdaptiveWorkspaceState* best_goal_state = nullptr;
            int best_cost = std::numeric_limits<int>::max();

            for (std::size_t aidx = 0; aidx < actions.size(); ++aidx) {
                const Action& action = actions[aidx];

                const WorkspaceState& final_state = action.back();
                if (!isGoal(final_state)) {
                    continue;
                }

                if (!checkAction(prev_state->state, action)) {
                    continue;
                }

                WorkspaceCoord goal_coord;
                stateWorkspaceToCoord(final_state, goal_coord);

                int goal_id = getHiHashEntry(goal_coord);
                if (goal_id < 0) {
                    SMPL_WARN_NAMED(params()->graph_log, "Successor state for goal transition was not generated");
                    continue;
                }

                AdaptiveWorkspaceState* goal_state = getHiHashEntry(goal_id);
                if (!goal_state) {
                    SMPL_ERROR_NAMED(params()->graph_log, "Target goal state is not high-dimensional");
                    return false;
                }

                best_cost = 30;
                best_goal_state = goal_state;
            }

            if (!best_goal_state) {
                SMPL_ERROR_NAMED(params()->graph_log, "Failed to find valid goal successor during path extraction");
                return false;
            }

            path.push_back(best_goal_state->state);
        } else {
            AdaptiveWorkspaceState* state = getHiHashEntry(curr_id);
            if (!state) {
                SMPL_ERROR_NAMED(params()->graph_log, "Intermediate state is not high-dimensional");
                return false;
            }
            path.push_back(state->state);
        }
    }

    return true;
}

bool AdaptiveWorkspaceLattice::setStart(const RobotState& state)
{
    if (!initialized()) {
        SMPL_ERROR_NAMED(params()->graph_log, "cannot set start state on uninitialized lattice");
        return false;
    }

    SMPL_DEBUG_NAMED(params()->graph_log, "set the start state");
    if (state.size() < robot()->jointVariableCount()) {
        SMPL_ERROR_NAMED(params()->graph_log, "start state contains insufficient coordinate positions");
        return false;
    }

    SMPL_DEBUG_STREAM_NAMED(params()->graph_log, "  angles: " << state);

    if (!robot()->checkJointLimits(state)) {
        SMPL_ERROR_NAMED(params()->graph_log, "start state violates joint limits");
        return false;
    }

    if (!collisionChecker()->isStateValid(state, true)) {
        SV_SHOW_WARN(collisionChecker()->getCollisionModelVisualization(state));
        SMPL_WARN("start state is in collision");
        return false;
    }

    SV_SHOW_INFO(getStateVisualization(state, "start_config"));
    WorkspaceCoord start_coord;
    stateRobotToCoord(state, start_coord);

    m_start_state_id = getHiHashEntry(start_coord);
    if (m_start_state_id < 0) {
        m_start_state_id = createHiState(start_coord, state);
    }
    m_start_state = getHashEntry(m_start_state_id);

    return RobotPlanningSpace::setStart(state);
}

bool AdaptiveWorkspaceLattice::setGoal(const GoalConstraint& goal)
{
    switch (goal.type) {
    case GoalType::XYZ_GOAL:
    case GoalType::XYZ_RPY_GOAL:
        setGoalPose(goal);
        return true;
    case GoalType::JOINT_STATE_GOAL:
    default:
        return false;
    }
}

Extension* AdaptiveWorkspaceLattice::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotPlanningSpace>() ||
        class_code == GetClassCode<PointProjectionExtension>() ||
        class_code == GetClassCode<AdaptiveGraphExtension>())
    {
        return this;
    }
    return nullptr;
}

void AdaptiveWorkspaceLattice::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    assert(state_id >= 0 && state_id < (int)m_states.size());

    SMPL_DEBUG_NAMED(params()->expands_log, "Expand state %d", state_id);

    if (state_id == m_goal_state_id) {
        return;
    }

    AdaptiveState* state = m_states[state_id];
    if (state->hid) {
        AdaptiveWorkspaceState* hi_state = (AdaptiveWorkspaceState*)state;
        GetSuccs(*hi_state, succs, costs);
    } else {
        AdaptiveGridState* lo_state = (AdaptiveGridState*)state;
        GetSuccs(*lo_state, succs, costs);
    }
    SMPL_DEBUG_NAMED(params()->expands_log, " -> %zu successors", succs->size());
}

void AdaptiveWorkspaceLattice::GetPreds(
    int state_id,
    std::vector<int>* preds,
    std::vector<int>* costs)
{
    // TODO: implement
}

void AdaptiveWorkspaceLattice::PrintState(
    int state_id,
    bool verbose,
    FILE* f)
{
    if (!f) {
        f = stdout;
    }

    AdaptiveState* state = getHashEntry(state_id);
    std::stringstream ss;
    if (state->hid) {
        AdaptiveWorkspaceState* hi_state = (AdaptiveWorkspaceState*)state;
        ss << *hi_state;
    } else {
        AdaptiveGridState* lo_state = (AdaptiveGridState*)state;
        ss << *lo_state;
    }

    if (f == stdout) {
        SMPL_DEBUG_NAMED(params()->graph_log, "%s", ss.str().c_str());
    } else if (f == stderr) {
        SMPL_WARN_NAMED(params()->graph_log, "%s", ss.str().c_str());
    } else {
        fprintf(f, "%s\n", ss.str().c_str());
    }
}


/*void AdaptiveWorkspaceLattice:: getExpandedStates(std::vector<RobotState>& states) const 
{
    ROS_WARN("getExpandedStates not implemented for Adaptive Workspace Lattice!");
}*/

bool AdaptiveWorkspaceLattice::initMotionPrimitives()
{
    // TODO: Factor out copy-pasta from WorkspaceLattice?
    m_hi_prims.clear();

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

                m_hi_prims.push_back(prim);
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
        m_hi_prims.push_back(prim);

        d[a] = m_res[a] * 1;
        prim.type = MotionPrimitive::Type::LONG_DISTANCE;
        prim.action.clear();
        prim.action.push_back(d);
        m_hi_prims.push_back(prim);
    }

    // create 26-connected ld position motions
    m_lo_prims.clear();
    for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
    for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
            continue;
        }

         m_lo_prims.emplace_back(m_res[0] * dx, m_res[1] * dy, m_res[2] * dz);
    }
    }
    }

    return true;
}

bool AdaptiveWorkspaceLattice::setGoalPose(const GoalConstraint& goal)
{
    if (!initialized()) {
        SMPL_ERROR_NAMED(params()->graph_log, "cannot set goal pose on uninitialized lattice");
        return false;
    }

    if (goal.pose.size() != 6) {
        SMPL_ERROR("goal element has incorrect format");
        return false;
    }

    if (goal.tgt_off_pose.size() != 6) {
        SMPL_ERROR_NAMED(params()->graph_log, "Goal target offset pose has incorrect format");
        return false;
    }

    Eigen::Affine3d goal_pose(
            Eigen::Translation3d(
                    goal.tgt_off_pose[0],
                    goal.tgt_off_pose[1],
                    goal.tgt_off_pose[2]) *
            Eigen::AngleAxisd(goal.tgt_off_pose[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(goal.tgt_off_pose[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(goal.tgt_off_pose[3], Eigen::Vector3d::UnitX()));
    auto markers = visual::MakePoseMarkers(goal_pose, m_grid->getReferenceFrame(), "target_goal");
    SV_SHOW_INFO(markers);

    SMPL_DEBUG_NAMED(params()->graph_log, "set the goal state");

    // check if an IK solution exists for the goal pose before we do
    // the search we plan even if there is no solution
    RobotState seed(robot()->jointVariableCount(), 0);
    RobotState ik_solution;
    if (!m_ik_iface->computeIK(goal.pose, seed, ik_solution)) {
        SMPL_WARN("No valid IK solution for the goal pose.");
    }

    SMPL_DEBUG_NAMED(params()->graph_log, "  xyz (meters): (%0.2f, %0.2f, %0.2f)", goal.pose[0], goal.pose[1], goal.pose[2]);
    SMPL_DEBUG_NAMED(params()->graph_log, "  tol (meters): (%0.3f, %0.3f, %0.3f)", goal.xyz_tolerance[0], goal.xyz_tolerance[1], goal.xyz_tolerance[2]);
    SMPL_DEBUG_NAMED(params()->graph_log, "  rpy (radians): (%0.2f, %0.2f, %0.2f)", goal.pose[3], goal.pose[4], goal.pose[5]);
    SMPL_DEBUG_NAMED(params()->graph_log, "  tol (radians): (%0.3f, %0.3f, %0.3f)", goal.rpy_tolerance[0], goal.rpy_tolerance[1], goal.rpy_tolerance[2]);

    m_near_goal = false;
    m_t_start = clock::now();

    return RobotPlanningSpace::setGoal(goal);
}

void AdaptiveWorkspaceLattice::GetSuccs(
    const AdaptiveGridState& state,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    SMPL_DEBUG_NAMED(params()->expands_log, "  coord: (%d, %d, %d), state: (%0.3f, %0.3f, %0.3f)", state.gx, state.gy, state.gz, state.x, state.y, state.z);

    auto m = visual::MakeSphereMarker(
            state.x, state.y, state.z,
            m_grid->resolution(),
            30,
            m_grid->getReferenceFrame(),
            "expansion_lo",
            0);

    SV_SHOW_INFO(m);

    SMPL_DEBUG_NAMED(params()->successors_log, "  actions: %zu", m_lo_prims.size());
    for (size_t aidx = 0; aidx < m_lo_prims.size(); ++aidx) {
        const Eigen::Vector3d& dx = m_lo_prims[aidx];

        SMPL_DEBUG_NAMED(params()->successors_log, "    action %zu", aidx);

        double succ_pos[3] =
        {
            state.x + dx.x(),
            state.y + dx.y(),
            state.z + dx.z()
        };

        int tgx, tgy, tgz;
        m_grid->worldToGrid(
                succ_pos[0], succ_pos[1], succ_pos[2],
                tgx, tgy, tgz);

        if (!m_grid->isInBounds(tgx, tgy, tgz)) {
            SMPL_DEBUG_NAMED(params()->successors_log, "      -> out of bounds (%d, %d, %d)", tgx, tgy, tgz);
            continue;
        }

        const double sphere_radius = 0.02;
        if (m_grid->getDistance(tgx, tgy, tgz) < sphere_radius) {
            SMPL_DEBUG_NAMED(params()->successors_log, "      -> in collision (%d, %d, %d)", tgx, tgy, tgz);
            continue;
        }

        if (isHighDimensional(tgx, tgy, tgz)) {
            SMPL_DEBUG_NAMED(params()->successors_log, "      -> high-dimensional");
            // TODO: high-dimensional transitions
            // sample roll, pitch, yaw, and free angles

            WorkspaceState succ_state(m_dof_count);
            succ_state[0] = succ_pos[0];
            succ_state[1] = succ_pos[1];
            succ_state[2] = succ_pos[2];
            int yaw_samples = 4;
            int pitch_samples = 3;
            int roll_samples = 4;
            // TODO: sampling of free angle vector
//            std::vector<double> fav_samples(freeAngleCount, 4);
            for (int y = 0; y < yaw_samples; ++y) {
            for (int p = 0; p < pitch_samples; ++p) {
            for (int r = 0; r < roll_samples; ++r) {
                double yaw = y * (2.0 * M_PI) / (double)yaw_samples;
                double pitch = p * (M_PI / (double)(pitch_samples - 1));
                double roll = r * (2.0 * M_PI) / (double)roll_samples;
                succ_state[3] = roll;
                succ_state[4] = pitch;
                succ_state[5] = yaw;

                WorkspaceCoord succ_coord;
                RobotState final_rstate;
                stateWorkspaceToCoord(succ_state, succ_coord);
                if (!stateWorkspaceToRobot(succ_state, final_rstate)) {
                    continue;
                }

                int succ_id = getHiHashEntry(succ_coord);
                if (succ_id < 0) {
                    succ_id = createHiState(succ_coord, final_rstate);
                }

                if (isGoal(succ_state)) {
                    succs->push_back(m_goal_state_id);
                } else {
                    succs->push_back(succ_id);
                }
                costs->push_back(30);
            }
            }
            }
        } else {
            SMPL_DEBUG_NAMED(params()->successors_log, "      -> low-dimensional");

            int succ_coord[3];
            posWorkspaceToCoord(succ_pos, succ_coord);

            int succ_id = getLoHashEntry(tgx, tgy, tgz);
            if (succ_id < 0) {
                succ_id = createLoState(
                    succ_coord[0], succ_coord[1], succ_coord[2],
                    succ_pos[0], succ_pos[1], succ_pos[2]);
            }

            if (isLoGoal(succ_pos[0], succ_pos[1], succ_pos[2])) {
                succs->push_back(m_goal_state_id);
            } else {
                succs->push_back(succ_id);
            }
            costs->push_back(30);
            SMPL_DEBUG_NAMED(params()->successors_log, "          succ: { id: %d, coord: (%d, %d, %d), state: (%0.3f, %0.3f, %0.3f), cost: %d }", succs->back(), succ_coord[0], succ_coord[1], succ_coord[2], succ_pos[0], succ_pos[1], succ_pos[2], costs->back());
        }
    }
}

void AdaptiveWorkspaceLattice::GetSuccs(
    const AdaptiveWorkspaceState& state,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  coord: " << state.coord);
    SMPL_DEBUG_STREAM_NAMED(params()->expands_log, "  state: " << state.state);
    SV_SHOW_DEBUG(getStateVisualization(state.state, "expansion"));

    std::vector<Action> actions;
    getActions(state, actions);

    SMPL_DEBUG_NAMED(params()->successors_log, "  actions: %zu", actions.size());
    for (std::size_t i = 0; i < actions.size(); ++i) {
        const Action& action = actions[i];

        SMPL_DEBUG_NAMED(params()->successors_log, "    action %zu", i);
        SMPL_DEBUG_NAMED(params()->successors_log, "      waypoints: %zu", action.size());

        RobotState final_rstate;
        if (!checkAction(state.state, action, &final_rstate)) {
            continue;
        }

        const WorkspaceState& final_state = action.back();

        // project state to 3d
        int gx, gy, gz;
        m_grid->worldToGrid(
                final_state[0], final_state[1], final_state[2], gx, gy, gz);

        if (!m_grid->isInBounds(gx, gy, gz)) {
            SMPL_DEBUG_NAMED(params()->successors_log, "      -> out of bounds (%d, %d, %d)", gx, gy, gz);
            continue;
        }

        WorkspaceCoord succ_coord;
        stateWorkspaceToCoord(final_state, succ_coord);
        if (isHighDimensional(gx, gy, gz)) {
            SMPL_DEBUG_NAMED(params()->successors_log, "      -> high-dimensional");
            int succ_id = getHiHashEntry(succ_coord);
            if (succ_id < 0) {
                succ_id = createHiState(succ_coord, final_rstate);
            }

            const bool is_goal_succ = isGoal(final_state);
            if (is_goal_succ) {
                succs->push_back(m_goal_state_id);
            } else {
                succs->push_back(succ_id);
            }
            costs->push_back(120);

            SMPL_DEBUG_STREAM_NAMED(params()->successors_log, "         succ: { id: " << succs->back() << ", coord: " << succ_coord << ", state: " << final_rstate << ", cost: " << costs->back() << " }");
        } else if (m_plan_mode) {
            SMPL_DEBUG_NAMED(params()->successors_log, "      -> low-dimensional");
            int succ_id = getLoHashEntry(succ_coord[0], succ_coord[1], succ_coord[2]);
            if (succ_id < 0) {
                succ_id = createLoState(
                        succ_coord[0], succ_coord[1], succ_coord[2],
                        final_state[0], final_state[1], final_state[2]);
            }
            if (isLoGoal(final_state[0], final_state[1], final_state[2])) {
                succs->push_back(m_goal_state_id);
            } else {
                succs->push_back(succ_id);
            }
            costs->push_back(120);
            SMPL_DEBUG_NAMED(params()->successors_log, "         succ: { id: %d, coord: (%d, %d, %d), state: (%0.3f, %0.3f, %0.3f), cost: %d }", succs->back(), succ_coord[0], succ_coord[1], succ_coord[2], final_state[0], final_state[1], final_state[2], costs->back());
        } else {
            SMPL_DEBUG_NAMED(params()->successors_log, "      -> outside tunnel");
        }
    }
}

int AdaptiveWorkspaceLattice::reserveHashEntry(bool hid)
{
    AdaptiveState* entry;
    if (hid) {
        entry = new AdaptiveWorkspaceState;
    } else {
        entry = new AdaptiveGridState;
    }
    entry->hid = hid;

    int state_id = (int)m_states.size();

    // map state id -> state
    m_states.push_back(entry);

    // map planner state -> graph state
    int* pinds = new int[NUMOFINDICES_STATEID2IND];
    std::fill(pinds, pinds + NUMOFINDICES_STATEID2IND, -1);
    StateID2IndexMapping.push_back(pinds);

    return state_id;
}

bool AdaptiveWorkspaceLattice::isHighDimensional(int gx, int gy, int gz) const
{
    if (m_plan_mode) {
        return m_dim_grid(gx, gy, gz).plan_hd;
    } else {
        return m_dim_grid(gx, gy, gz).trak_hd;
    }
}

AdaptiveState* AdaptiveWorkspaceLattice::getHashEntry(int state_id) const
{
    assert(state_id >= 0 && state_id < m_states.size());
    return m_states[state_id];
}

AdaptiveWorkspaceState* AdaptiveWorkspaceLattice::getHiHashEntry(
    int state_id) const
{
    AdaptiveState* state = getHashEntry(state_id);
    if (state->hid) {
        return (AdaptiveWorkspaceState*)state;
    }
    return nullptr;
}

AdaptiveGridState* AdaptiveWorkspaceLattice::getLoHashEntry(
    int state_id) const
{
    AdaptiveState* state = getHashEntry(state_id);
    if (!state->hid) {
        return (AdaptiveGridState*)state;
    }
    return nullptr;
}

int AdaptiveWorkspaceLattice::getHiHashEntry(
    const WorkspaceCoord& coord)
{
    AdaptiveWorkspaceState state;
    state.coord = coord;
    auto sit = m_hi_to_id.find(&state);
    if (sit == m_hi_to_id.end()) {
        return -1;
    }
    return sit->second;
}

int AdaptiveWorkspaceLattice::getLoHashEntry(
    int x,
    int y,
    int z)
{
    AdaptiveGridState state;
    state.gx = x;
    state.gy = y;
    state.gz = z;
    auto sit = m_lo_to_id.find(&state);
    if (sit == m_lo_to_id.end()) {
        return -1;
    }
    return sit->second;
}

int AdaptiveWorkspaceLattice::createHiState(
    const WorkspaceCoord& coord,
    const RobotState& state)
{
    int state_id = reserveHashEntry(true);
    AdaptiveWorkspaceState* hi_state = getHiHashEntry(state_id);
    hi_state->coord = coord;
    hi_state->state = state;
    m_hi_to_id[hi_state] = state_id;
    return state_id;
}

int AdaptiveWorkspaceLattice::createLoState(
    int x, int y, int z,
    double wx, double wy, double wz)
{
    int state_id = reserveHashEntry(false);
    AdaptiveGridState* lo_state = getLoHashEntry(state_id);
    lo_state->gx = x;
    lo_state->gy = y;
    lo_state->gz = z;
    lo_state->x = wx;
    lo_state->y = wy;
    lo_state->z = wz;
    return state_id;
}

void AdaptiveWorkspaceLattice::getActions(
    const AdaptiveWorkspaceState& state,
    std::vector<Action>& actions)
{
    actions.clear();
    actions.reserve(m_hi_prims.size());

    WorkspaceState cont_state;
    stateCoordToWorkspace(state.coord, cont_state);

    SMPL_DEBUG_STREAM_NAMED(params()->successors_log, "Create actions for state: " << cont_state);

    for (std::size_t pidx = 0; pidx < m_hi_prims.size(); ++pidx) {
        const auto& prim = m_hi_prims[pidx];

        Action action;
        action.reserve(prim.action.size());

        WorkspaceState final_state = cont_state;
        for (const RobotState& delta_state : prim.action) {
            for (int d = 0; d < dofCount(); ++d) {
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
            if (m_ik_iface->computeIK(goal().tgt_off_pose, state.state, ik_sol)) {
                WorkspaceState final_state;
                stateRobotToWorkspace(ik_sol, final_state);
                Action action(1);
                action[0] = final_state;
                actions.push_back(std::move(action));
            }
        }
    }
}

bool AdaptiveWorkspaceLattice::checkAction(
    const RobotState& state,
    const Action& action,
    RobotState* final_rstate)
{
    std::vector<RobotState> wptraj;
    wptraj.reserve(action.size());

    std::uint32_t violation_mask = 0x00000000;

    // check waypoints for ik solutions and joint limits
    for (size_t widx = 0; widx < action.size(); ++widx) {
        const WorkspaceState& istate = action[widx];

        SMPL_DEBUG_STREAM_NAMED(params()->successors_log, "        " << widx << ": " << istate);

        RobotState irstate;
        if (!stateWorkspaceToRobot(istate, state, irstate)) {
            SMPL_DEBUG_NAMED(params()->successors_log, "         -> failed to find ik solution");
            violation_mask |= 0x00000001;
            break;
        }

        wptraj.push_back(irstate);

        if (!robot()->checkJointLimits(irstate)) {
            SMPL_DEBUG_NAMED(params()->successors_log, "        -> violates joint limits");
            violation_mask |= 0x00000002;
            break;
        }
    }

    if (violation_mask) {
        return false;
    }

    // check for collisions between the waypoints
    assert(wptraj.size() == action.size());

    if (!collisionChecker()->isStateToStateValid(state, wptraj[0])) {
        SMPL_DEBUG_NAMED(params()->successors_log, "        -> path to first waypoint in collision");
        violation_mask |= 0x00000004;
    }

    if (violation_mask) {
        return false;
    }

    for (size_t widx = 1; widx < wptraj.size(); ++widx) {
        const RobotState& prev_istate = wptraj[widx - 1];
        const RobotState& curr_istate = wptraj[widx];
        if (!collisionChecker()->isStateToStateValid(prev_istate, curr_istate)) {
            SMPL_DEBUG_NAMED(params()->successors_log, "        -> path between waypoints in collision");
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

bool AdaptiveWorkspaceLattice::isGoal(const WorkspaceState& state) const
{
    return std::fabs(state[0] - goal().tgt_off_pose[0]) <= goal().xyz_tolerance[0] &&
            std::fabs(state[1] - goal().tgt_off_pose[1]) <= goal().xyz_tolerance[1] &&
            std::fabs(state[2] - goal().tgt_off_pose[2]) <= goal().xyz_tolerance[2] &&
            angles::shortest_angle_dist(state[3], goal().tgt_off_pose[3]) <= goal().rpy_tolerance[0] &&
            angles::shortest_angle_dist(state[4], goal().tgt_off_pose[4]) <= goal().rpy_tolerance[1] &&
            angles::shortest_angle_dist(state[5], goal().tgt_off_pose[5]) <= goal().rpy_tolerance[2];
}

bool AdaptiveWorkspaceLattice::isLoGoal(double x, double y, double z) const
{
    const double dx = std::fabs(x - goal().tgt_off_pose[0]);
    const double dy = std::fabs(y - goal().tgt_off_pose[1]);
    const double dz = std::fabs(z - goal().tgt_off_pose[2]);
    return dx <= goal().xyz_tolerance[0] &&
            dy <= goal().xyz_tolerance[1] &&
            dz <= goal().xyz_tolerance[2];
}

auto AdaptiveWorkspaceLattice::getStateVisualization(
    const RobotState& state,
    const std::string& ns) -> std::vector<visual::Marker>
{
    auto markers = collisionChecker()->getCollisionModelVisualization(state);
    for (auto& marker : markers) {
        marker.ns = ns;
    }
    return markers;
}

auto AdaptiveWorkspaceLattice::getAdaptiveGridVisualization(bool plan_mode) const
    -> visual::Marker
{
    visual::Marker m;

    std::vector<Eigen::Vector3d> points;
    for (int x = 0; x < m_grid->numCellsX(); ++x) {
    for (int y = 0; y < m_grid->numCellsY(); ++y) {
    for (int z = 0; z < m_grid->numCellsZ(); ++z) {
        if (isHighDimensional(x, y, z)) {
            Eigen::Vector3d p;
            m_grid->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            points.push_back(p);
        }
    }
    }
    }

    visual::Color color;
    if (m_plan_mode) {
        color.r = color.a = 1.0f;
        color.g = color.b = 0.0f;
    } else {
        color.b = color.a = 1.0f;
        color.r = color.g = 0.0f;
    }

    SMPL_INFO("Visualize %zu/%zu adaptive cells", points.size(), m_dim_grid.size());

    return MakeCubesMarker(
        std::move(points),
        0.5 * m_grid->resolution(),
        color,
        m_grid->getReferenceFrame(),
        plan_mode ? "adaptive_grid_plan" : "adaptive_grid_track");
}

} // namespace motion
} // namespace sbpl
