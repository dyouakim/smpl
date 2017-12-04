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

#include <smpl/graph/manip_lattice_egraph.h>

#include <fstream>

#include <boost/filesystem.hpp>
#include <leatherman/print.h>
#include <smpl/csv_parser.h>
#include <smpl/intrusive_heap.h>
#include <smpl/debug/visualize.h>
#include <smpl/graph/manip_lattice_action_space.h>

namespace sbpl {
namespace motion {

auto ManipLatticeEgraph::RobotCoordHash::operator()(const argument_type& s) const ->
    result_type
{
    std::size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.begin(), s.end()));
    return seed;
}

ManipLatticeEgraph::ManipLatticeEgraph(
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
:
    ManipLattice(robot, checker, params),
    m_coord_to_nodes(),
    m_egraph(),
    m_egraph_state_ids()
{
}

bool ManipLatticeEgraph::extractPath(
    const std::vector<int>& idpath,
    std::vector<RobotState>& path, std::vector<geometry_msgs::PoseStamped>& eePath)
{
    ROS_DEBUG_NAMED(params()->graph_log, "State ID Path: %s", to_string(idpath).c_str());
    if (idpath.empty()) {
        return true;
    }

    // attempt to handle paths of length 1...do any of the sbpl planners still
    // return a single-point path in some cases?
    if (idpath.size() == 1) {
        const int state_id = idpath[0];

        if (state_id == getGoalStateID()) {
            RobotState angles;
            const ManipLatticeState* entry = getHashEntry(getStartStateID());
            if (!entry) {
                ROS_ERROR_NAMED(params()->graph_log, "Failed to get state entry for state %d", getStartStateID());
                return false;
            }
            path.push_back(entry->state);
        } else {
            const ManipLatticeState* entry = getHashEntry(state_id);
            if (!entry) {
                ROS_ERROR_NAMED(params()->graph_log, "Failed to get state entry for state %d", state_id);
                return false;
            }
            path.push_back(entry->state);
        }

        SV_SHOW_INFO(getStateVisualization(path.back(), "goal_state"));
        return true;
    }

    if (idpath[0] == getGoalStateID()) {
        ROS_ERROR_NAMED(params()->graph_log, "Cannot extract a non-trivial path starting from the goal state");
        return false;
    }

    std::vector<RobotState> opath;

    // grab the first point
    {
        RobotState angles;
        const ManipLatticeState* entry = getHashEntry(idpath[0]);
        if (!entry) {
            ROS_ERROR_NAMED(params()->graph_log, "Failed to get state entry for state %d", idpath[0]);
            return false;
        }
        opath.push_back(entry->state);
    }

    ActionSpacePtr action_space = actionSpace();
    if (!action_space) {
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

        // find the successor state corresponding to the cheapest valid action

        ManipLatticeState* prev_entry = getHashEntry(prev_id);
        const RobotState& prev_state = prev_entry->state;

        std::vector<Action> actions;
        if (!action_space->apply(prev_state, actions)) {
            ROS_ERROR_NAMED(params()->graph_log, "Failed to get actions while extracting the path");
            return false;
        }

        ROS_DEBUG_NAMED(params()->graph_log, "Check for transition via normal successors");
        ManipLatticeState* best_state = nullptr;
        RobotCoord succ_coord(robot()->jointVariableCount());
        int best_cost = std::numeric_limits<int>::max();
        for (const Action& action : actions) {
            // check the validity of this transition
            double dist;
            if (!checkAction(prev_state, action, dist)) {
                continue;
            }

            if (curr_id == getGoalStateID()) {
                ROS_DEBUG_NAMED(params()->graph_log, "Search for transition to goal state");
                std::vector<double> tgt_off_pose;
                if (!computePlanningFrameFK(action.back(), tgt_off_pose)) {
                    ROS_WARN("Failed to compute FK for planning frame");
                    continue;
                }

                // skip non-goal states
                if (!isGoal(action.back(), tgt_off_pose)) {
                    continue;
                }

                stateToCoord(action.back(), succ_coord);
                int succ_state_id = getHashEntry(succ_coord);
                ManipLatticeState* succ_entry = getHashEntry(succ_state_id);
                assert(succ_entry);

                const int edge_cost = cost(prev_entry, succ_entry, true);
                if (edge_cost < best_cost) {
                    best_cost = edge_cost;
                    best_state = succ_entry;
                }
            } else {
                stateToCoord(action.back(), succ_coord);
                int succ_state_id = getHashEntry(succ_coord);
                ManipLatticeState* succ_entry = getHashEntry(succ_state_id);
                assert(succ_entry);
                if (succ_state_id != curr_id) {
                    continue;
                }

                const int edge_cost = cost(prev_entry, succ_entry, false);
                if (edge_cost < best_cost) {
                    best_cost = edge_cost;
                    best_state = succ_entry;
                }
            }
        }

        if (best_state) {
            ROS_DEBUG_NAMED(params()->graph_log, "Extract successor state %s", to_string(best_state->state).c_str());
            opath.push_back(best_state->state);
            continue;
        }

        bool found = false;
        // check for shortcut transition
        auto pnit = std::find(m_egraph_state_ids.begin(), m_egraph_state_ids.end(), prev_id);
        auto cnit = std::find(m_egraph_state_ids.begin(), m_egraph_state_ids.end(), curr_id);
        if (pnit != m_egraph_state_ids.end() &&
            cnit != m_egraph_state_ids.end())
        {
            ExperienceGraph::node_id pn =
                    std::distance(m_egraph_state_ids.begin(), pnit);
            ExperienceGraph::node_id cn =
                    std::distance(m_egraph_state_ids.begin(), cnit);

            ROS_INFO("Check for shortcut from %d to %d (egraph %zu -> %zu)!", prev_id, curr_id, pn, cn);

            std::vector<ExperienceGraph::node_id> node_path;
            found = findShortestExperienceGraphPath(pn, cn, node_path);
            if (found) {
                for (ExperienceGraph::node_id n : node_path) {
                    int state_id = m_egraph_state_ids[n];
                    ManipLatticeState* entry = getHashEntry(state_id);
                    assert(entry);
                    opath.push_back(entry->state);
                }
            }
        }
        if (found) {
            continue;
        }

        // check for snap transition
        ROS_DEBUG_NAMED(params()->graph_log, "Check for snap successor");
        int cost;
        if (snap(prev_id, curr_id, cost)) {
            ROS_ERROR("Snap from %d to %d with cost %d", prev_id, curr_id, cost);
            ManipLatticeState* entry = getHashEntry(curr_id);
            assert(entry);
            opath.push_back(entry->state);
            continue;
        }

        ROS_ERROR_NAMED(params()->graph_log, "Failed to find valid goal successor during path extraction");
        return false;
    }

    // we made it!
    path = std::move(opath);
    SV_SHOW_INFO(getStateVisualization(path.back(), "goal_state"));
    return true;
}

bool ManipLatticeEgraph::loadExperienceGraph(const std::string& path)
{
    ROS_INFO("Load Experience Graph at %s", path.c_str());

    boost::filesystem::path p(path);
    if (!boost::filesystem::is_directory(p)) {
        ROS_ERROR("'%s' is not a directory", path.c_str());
        return false;
    }

    for (auto dit = boost::filesystem::directory_iterator(p);
        dit != boost::filesystem::directory_iterator(); ++dit)
    {
        const std::string& filepath = dit->path().generic_string();
        std::vector<RobotState> egraph_states;
        if (!parseExperienceGraphFile(filepath, egraph_states)) {
            continue;
        }

        if (egraph_states.empty()) {
            continue;
        }

        ROS_INFO("Create hash entries for experience graph states");

        const RobotState& pp = egraph_states.front();  // previous robot state
        RobotCoord pdp(robot()->jointVariableCount()); // previous robot coord
        stateToCoord(egraph_states.front(), pdp);

        ExperienceGraph::node_id pid = m_egraph.insert_node(pp);
        m_coord_to_nodes[pdp].push_back(pid);

        int entry_id = reserveHashEntry();
        ManipLatticeState* entry = getHashEntry(entry_id);
        entry->coord = pdp;
        entry->state = pp;

        // map state id <-> experience graph state
        m_egraph_state_ids.resize(pid + 1, -1);
        m_egraph_state_ids[pid] = entry_id;
        m_state_to_node[entry_id] = pid;

        std::vector<RobotState> edge_data;
        for (size_t i = 1; i < egraph_states.size(); ++i) {
            const RobotState& p = egraph_states[i];
            RobotCoord dp(robot()->jointVariableCount());
            stateToCoord(p, dp);
            if (dp != pdp) {
                // found a new discrete state along the path

                ExperienceGraph::node_id id = m_egraph.insert_node(p);
                m_coord_to_nodes[dp].push_back(id);

                int entry_id = reserveHashEntry();
                ManipLatticeState* entry = getHashEntry(entry_id);
                entry->coord = dp;
                entry->state = p;

                m_egraph_state_ids.resize(id + 1, -1);
                m_egraph_state_ids[id] = entry_id;
                m_state_to_node[entry_id] = id;
                m_egraph.insert_edge(pid, id, edge_data);

                pdp = dp;
                pid = id;
                edge_data.clear();
            } else {
                // gather intermediate robot states
                edge_data.push_back(p);
            }
        }
    }

    ROS_INFO("Experience graph contains %zu nodes and %zu edges", m_egraph.num_nodes(), m_egraph.num_edges());
    return true;
}

void ManipLatticeEgraph::getExperienceGraphNodes(
    int state_id,
    std::vector<ExperienceGraph::node_id>& nodes)
{
    auto it = m_state_to_node.find(state_id);
    if (it != m_state_to_node.end()) {
        nodes.push_back(it->second);
    }
}

bool ManipLatticeEgraph::shortcut(
    int first_id,
    int second_id,
    int& cost)
{
    ManipLatticeState* first_entry = getHashEntry(first_id);
    ManipLatticeState* second_entry = getHashEntry(second_id);
    if (!first_entry | !second_entry) {
        ROS_WARN("No state entries for state %d or state %d", first_id, second_id);
        return false;
    }

    ROS_INFO("Shortcut %s -> %s", to_string(first_entry->state).c_str(), to_string(second_entry->state).c_str());
    SV_SHOW_INFO(getStateVisualization(first_entry->state, "shortcut_from"));
    SV_SHOW_INFO(getStateVisualization(second_entry->state, "shortcut_to"));

    ROS_INFO("  Shortcut %d -> %d!", first_id, second_id);
    cost = 1000;
    return true;
}

bool ManipLatticeEgraph::snap(
    int first_id,
    int second_id,
    int& cost)
{
    ManipLatticeState* first_entry = getHashEntry(first_id);
    ManipLatticeState* second_entry = getHashEntry(second_id);
    if (!first_entry | !second_entry) {
        ROS_WARN("No state entries for state %d or state %d", first_id, second_id);
        return false;
    }

    ROS_INFO("Snap %s -> %s", to_string(first_entry->state).c_str(), to_string(second_entry->state).c_str());
    SV_SHOW_INFO(getStateVisualization(first_entry->state, "snap_from"));
    SV_SHOW_INFO(getStateVisualization(second_entry->state, "snap_to"));

    int plen, check_count;
    double dist;
    if (!collisionChecker()->isStateToStateValid(
        first_entry->state, second_entry->state, plen, check_count, dist))
    {
        ROS_WARN("Failed snap!");
        return false;
    }

    ROS_INFO("  Snap %d -> %d!", first_id, second_id);
    cost = 1000;
    return true;
}

const ExperienceGraph* ManipLatticeEgraph::getExperienceGraph() const
{
    return &m_egraph;
}

ExperienceGraph* ManipLatticeEgraph::getExperienceGraph()
{
    return &m_egraph;
}

int ManipLatticeEgraph::getStateID(ExperienceGraph::node_id n) const
{
    if (n >= m_egraph_state_ids.size()) {
        return -1;
    } else {
        return m_egraph_state_ids[n];
    }
}

Extension* ManipLatticeEgraph::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<ExperienceGraphExtension>()) {
        return this;
    } else {
        return ManipLattice::getExtension(class_code);
    }
}

bool ManipLatticeEgraph::findShortestExperienceGraphPath(
    ExperienceGraph::node_id start_node,
    ExperienceGraph::node_id goal_node,
    std::vector<ExperienceGraph::node_id>& path)
{
    struct ExperienceGraphSearchNode : heap_element
    {
        int g;
        bool closed;
        ExperienceGraphSearchNode* bp;
        ExperienceGraphSearchNode() :
            g(std::numeric_limits<int>::max()),
            closed(false),
            bp(nullptr)
        { }
    };

    struct NodeCompare
    {
        bool operator()(
            const ExperienceGraphSearchNode& a,
            const ExperienceGraphSearchNode& b)
        {
            return a.g < b.g;
        }
    };

    typedef intrusive_heap<ExperienceGraphSearchNode, NodeCompare> heap_type;

    std::vector<ExperienceGraphSearchNode> search_nodes(m_egraph.num_nodes());

    heap_type open;

    search_nodes[start_node].g = 0;
    open.push(&search_nodes[start_node]);
    int exp_count = 0;
    while (!open.empty()) {
        ++exp_count;
        ExperienceGraphSearchNode* min = open.min();
        open.pop();
        min->closed = true;

        if (min == &search_nodes[goal_node]) {
            ROS_ERROR("Found shortest experience graph path");
            ExperienceGraphSearchNode* ps = nullptr;
            for (ExperienceGraphSearchNode* s = &search_nodes[goal_node];
                s; s = s->bp)
            {
                if (s != ps) {
                    path.push_back(std::distance(search_nodes.data(), s));
                    ps = s;
                } else {
                    ROS_ERROR("Cycle detected!");
                }
            }
            std::reverse(path.begin(), path.end());
            return true;
        }

        ExperienceGraph::node_id n = std::distance(search_nodes.data(), min);
        auto adj = m_egraph.adjacent_nodes(n);
        for (auto ait = adj.first; ait != adj.second; ++ait) {
            ExperienceGraphSearchNode& succ = search_nodes[*ait];
            if (succ.closed) {
                continue;
            }
            int new_cost = min->g + 1;
            if (new_cost < succ.g) {
                succ.g = new_cost;
                succ.bp = min;
                if (open.contains(&succ)) {
                    open.decrease(&succ);
                } else {
                    open.push(&succ);
                }
            }
        }
    }

    ROS_INFO("Expanded %d nodes looking for shortcut", exp_count);
    return false;
}

bool ManipLatticeEgraph::parseExperienceGraphFile(
    const std::string& filepath,
    std::vector<RobotState>& egraph_states) const
{
    std::ifstream fin(filepath);
    if (!fin.is_open()) {
        return false;
    }

    CSVParser parser;
    const bool with_header = true;
    if (!parser.parseStream(fin, with_header)) {
        ROS_ERROR("Failed to parse experience graph file '%s'", filepath.c_str());
        return false;
    }

    ROS_INFO("Parsed experience graph file");
    ROS_INFO("  Has Header: %s", parser.hasHeader() ? "true" : "false");
    ROS_INFO("  %zu records", parser.recordCount());
    ROS_INFO("  %zu fields", parser.fieldCount());

    const size_t jvar_count = robot()->getPlanningJoints().size();
    if (parser.fieldCount() != jvar_count) {
        ROS_ERROR("Parsed experience graph contains insufficient number of joint variables");
        return false;
    }

    egraph_states.reserve(parser.totalFieldCount());
    for (size_t i = 0; i < parser.recordCount(); ++i) {
        RobotState state(jvar_count);
        for (size_t j = 0; j < parser.fieldCount(); ++j) {
            try {
                state[j] = std::stod(parser.fieldAt(i, j));
            } catch (const std::invalid_argument& ex) {
                ROS_ERROR("Failed to parse egraph state variable (%s)", ex.what());
                return false;
            } catch (const std::out_of_range& ex) {
                ROS_ERROR("Failed to parse egraph state variable (%s)", ex.what());
                return false;
            }
        }
        egraph_states.push_back(std::move(state));
    }

    ROS_INFO("Read %zu states from experience graph file", egraph_states.size());
    return true;
}

/// An attempt to construct the discrete experience graph by discretizing all
/// input continuous states and connecting them via edges available in the
/// canonical action set. This turns out to not work very well since the points
/// are not often able to be connected by the limited action set. It also
/// necessitates imposing restrictions on the action set, since context-specific
/// actions don't make sense before an actual planning request.
void ManipLatticeEgraph::rasterizeExperienceGraph()
{
//    std::vector<RobotCoord> egraph_coords;
//    for (const RobotState& state : egraph_states) {
//        RobotCoord coord(robot()->jointVariableCount());
//        stateToCoord(state, coord);
//        egraph_coords.push_back(std::move(coord));
//    }
//
//    auto it = std::unique(egraph_coords.begin(), egraph_coords.end());
//    egraph_coords.erase(it, egraph_coords.end());
//
//    ROS_INFO("Experience contains %zu discrete states", egraph_coords.size());
//    for (const RobotCoord& coord : egraph_coords) {
//        ROS_INFO("  %s", to_string(coord).c_str());
//    }
//
//    ROS_INFO("Insert states into experience graph and map coords to experience graph nodes");
//
//    // insert all coords into egraph and initialize coord -> egraph node mapping
//    RobotState state(robot()->jointVariableCount());
//    for (auto it = egraph_coords.begin(); it != egraph_coords.end(); ++it) {
//        coordToState(*it, state);
//        m_coord_to_nodes[*it] = m_egraph.insert_node(state);
//    }
//
//    ROS_INFO("Insert experience graph edges into experience graph");
//
//    int edge_count = 0;
//    ManipLatticeActionSpace* aspace =
//            dynamic_cast<ManipLatticeActionSpace*>(actionSpace().get());
//    if (!aspace) {
//        ROS_ERROR("ManipLatticeEgraph requires action space to be a ManipLatticeActionSpace");
//        return false;
//    }
//
//    // save action space configuration
//    bool mprim_enabled_state[MotionPrimitive::NUMBER_OF_MPRIM_TYPES];
//    for (int i = 0; i < MotionPrimitive::NUMBER_OF_MPRIM_TYPES; ++i) {
//        mprim_enabled_state[i] = aspace->useAmp((MotionPrimitive::Type)i);
//    }
//    bool use_long_and_short_mprims = aspace->useLongAndShortPrims();
//
//    // disable context-specific motion primitives
//    aspace->useAmp(MotionPrimitive::SHORT_DISTANCE, true);
//    aspace->useAmp(MotionPrimitive::SNAP_TO_RPY, false);
//    aspace->useAmp(MotionPrimitive::SNAP_TO_XYZ, false);
//    aspace->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, false);
//
//    for (auto it = egraph_coords.begin(); it != egraph_coords.end(); ++it) {
//        const ExperienceGraph::node_id n = m_coord_to_nodes[*it];
//        RobotState source(robot()->jointVariableCount());
//        coordToState(*it, source);
//        std::vector<Action> actions;
//        aspace->apply(source, actions);
//        ROS_INFO("%zu actions from egraph state", actions.size());
//        for (const Action& action : actions) {
//            RobotCoord last(robot()->jointVariableCount());
//            stateToCoord(action.back(), last);
//            ROS_INFO("Check for experience graph edge %s -> %s", to_string(*it).c_str(), to_string(last).c_str());
//            auto iit = m_coord_to_nodes.find(last);
//            if (iit != m_coord_to_nodes.end() && !m_egraph.edge(n, iit->second)) {
//                m_egraph.insert_edge(n, iit->second);
//                ++edge_count;
//            }
//        }
//    }
//    ROS_INFO("Experience graph contains %d edges", edge_count);
//
//    // restore action space configuration
//    for (int i = 0; i < MotionPrimitive::NUMBER_OF_MPRIM_TYPES; ++i) {
//        aspace->useAmp((MotionPrimitive::Type)i, mprim_enabled_state[i]);
//    }
//    aspace->useLongAndShortPrims(use_long_and_short_mprims);
}

} // namespace motion
} // namespace sbpl
