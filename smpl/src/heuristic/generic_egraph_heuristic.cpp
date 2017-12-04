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

// project includes
#include <smpl/console/console.h>
#include <smpl/heuristic/generic_egraph_heuristic.h>

namespace sbpl {
namespace motion {

static const char* LOG = "heuristic.generic_egraph";

bool GenericEgraphHeuristic::init(RobotPlanningSpace* space, RobotHeuristic* h)
{
    if (!h) {
        return false;
    }

    if (!RobotHeuristic::init(space)) {
        return false;
    }

    m_orig_h = h;

    m_eg = space->getExtension<ExperienceGraphExtension>();
    if (!m_eg) {
        SMPL_WARN_NAMED(LOG, "GenericEgraphHeuristic recommends ExperienceGraphExtension");
    }

    return true;
}

void GenericEgraphHeuristic::setWeightEGraph(double w)
{
    m_eg_eps = w;
    SMPL_INFO_NAMED(LOG, "egraph_epsilon: %0.3f", m_eg_eps);
}

void GenericEgraphHeuristic::getEquivalentStates(
    int state_id,
    std::vector<int>& ids)
{
    ExperienceGraph* eg = m_eg->getExperienceGraph();
    auto nodes = eg->nodes();
    const int equiv_thresh = 100;
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        int egraph_state_id = m_eg->getStateID(*nit);
        int h = m_orig_h->GetFromToHeuristic(state_id, egraph_state_id);
        if (h <= equiv_thresh) {
            ids.push_back(egraph_state_id);
        }
    }
}

void GenericEgraphHeuristic::getShortcutSuccs(
    int state_id,
    std::vector<int>& shortcut_ids)
{
    std::vector<ExperienceGraph::node_id> egraph_nodes;
    m_eg->getExperienceGraphNodes(state_id, egraph_nodes);

    for (ExperienceGraph::node_id n : egraph_nodes) {
        const int comp_id = m_component_ids[n];
        for (ExperienceGraph::node_id nn : m_shortcut_nodes[comp_id]) {
            int egraph_state_id = m_eg->getStateID(nn);
            if (state_id != egraph_state_id) {
                shortcut_ids.push_back(egraph_state_id);
            }
        }
    }
}

double GenericEgraphHeuristic::getMetricStartDistance(
    double x, double y, double z)
{
    return m_orig_h->getMetricStartDistance(x, y, z);
}

double GenericEgraphHeuristic::getMetricGoalDistance(
    double x, double y, double z)
{
    return m_orig_h->getMetricGoalDistance(x, y, z);
}

Extension* GenericEgraphHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<ExperienceGraphHeuristicExtension>()) {
        return this;
    }
    return nullptr;
}

void GenericEgraphHeuristic::updateGoal(const GoalConstraint& goal)
{
    m_orig_h->updateGoal(goal);

    if (!m_eg) {
        return;
    }

    ExperienceGraph* eg = m_eg->getExperienceGraph();
    if (!eg) {
        SMPL_ERROR("Experience Graph Extended Planning Space has null Experience Graph");
        return;
    }

    //////////////////////////////////////////////////////////
    // Compute Connected Components of the Experience Graph //
    //////////////////////////////////////////////////////////

    int comp_count = 0;
    m_component_ids.assign(eg->num_nodes(), -1);
    auto nodes = eg->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        if (m_component_ids[*nit] != -1) {
            continue;
        }

        std::vector<ExperienceGraph::node_id> frontier;
        frontier.push_back(*nit);
        while (!frontier.empty()) {
            ExperienceGraph::node_id n = frontier.back();
            frontier.pop_back();

            m_component_ids[n] = comp_count;

            auto adj = eg->adjacent_nodes(n);
            for (auto ait = adj.first; ait != adj.second; ++ait) {
                if (m_component_ids[*ait] == -1) {
                    frontier.push_back(*ait);
                }
            }
        }

        ++comp_count;
    }

    SMPL_INFO_NAMED(LOG, "Experience graph contains %d connected components", comp_count);

    ////////////////////////////
    // Compute Shortcut Nodes //
    ////////////////////////////

    m_shortcut_nodes.assign(comp_count, std::vector<ExperienceGraph::node_id>());
    std::vector<int> shortcut_heuristics(comp_count);
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        const ExperienceGraph::node_id n = *nit;
        const int comp_id = m_component_ids[n];
        const int state_id = m_eg->getStateID(n);

        int h = m_orig_h->GetGoalHeuristic(state_id);

        if (m_shortcut_nodes[comp_id].empty()) {
            m_shortcut_nodes[comp_id].push_back(n);
            shortcut_heuristics[comp_id] = h;
        } else {
            int best_h = shortcut_heuristics[comp_id];
            if (h < best_h) {
                m_shortcut_nodes[comp_id].clear();
                m_shortcut_nodes[comp_id].push_back(n);
                shortcut_heuristics[comp_id] = h;
            } else if (h == best_h) {
                m_shortcut_nodes[comp_id].push_back(n);
            }
        }
    }

    ////////////////////////////////////////////////////////////
    // Compute Heuristic Distances for Experience Graph Nodes //
    ////////////////////////////////////////////////////////////

    m_h_nodes.assign(eg->num_nodes() + 1, HeuristicNode(Unknown));
    m_open.clear();
    m_h_nodes[0].dist = 0;
    m_open.push(&m_h_nodes[0]);
    while (!m_open.empty()) {
        HeuristicNode* s = m_open.min();
        m_open.pop();

        int nidx = std::distance(m_h_nodes.data(), s);
        if (nidx == 0) {
            // neighbors: inflated edges to all experience graph states
            // unconditionally relaxed (goal node is the first node removed)
            for (auto nit = nodes.first; nit != nodes.second; ++nit) {
                const ExperienceGraph::node_id nid = *nit;
                HeuristicNode* n = &m_h_nodes[nid + 1];
                const int state_id = m_eg->getStateID(nid);
                const int h = m_orig_h->GetGoalHeuristic(state_id);
                n->dist = (int)(m_eg_eps * h);
                m_open.push(n);
            }
        } else {
            // neighbors: inflated edges to all non-adjacent experience graph
            // states original cost edges to all adjacent experience graph
            // states
            const ExperienceGraph::node_id sid = nidx - 1;
            const int s_state_id = m_eg->getStateID(sid);
            for (auto nit = nodes.first; nit != nodes.second; ++nit) {
                const ExperienceGraph::node_id nid = *nit;
                HeuristicNode* n = &m_h_nodes[nid + 1];
                if (eg->edge(sid, nid)) {
                    const int edge_cost = 10;
                    const int new_cost = s->dist + edge_cost;
                    if (new_cost < n->dist) {
                        n->dist = new_cost;
                        if (m_open.contains(n)) {
                            m_open.decrease(n);
                        } else {
                            m_open.push(n);
                        }
                    }
                } else {
                    const int n_state_id = m_eg->getStateID(nid);
                    int h = m_orig_h->GetFromToHeuristic(s_state_id, n_state_id);
                    const int new_cost = s->dist + (int)(m_eg_eps * h);
                    if (new_cost < n->dist) {
                        n->dist = new_cost;
                        if (m_open.contains(n)) {
                            m_open.decrease(n);
                        } else {
                            m_open.push(n);
                        }
                    }
                }
            }
        }
    }
}

int GenericEgraphHeuristic::GetGoalHeuristic(int state_id)
{
    if (!m_eg) {
        return 0;
    }

    ExperienceGraph* eg = m_eg->getExperienceGraph();
    if (!eg) {
        return 0;
    }

    int best_h = (int)(m_eg_eps * m_orig_h->GetGoalHeuristic(state_id));
    auto nodes = eg->nodes();
    for (auto nit = nodes.first; nit != nodes.second; ++nit) {
        const int egraph_state_id = m_eg->getStateID(*nit);
        const int h = m_orig_h->GetFromToHeuristic(state_id, egraph_state_id);
        const int dist = m_h_nodes[*nit + 1].dist;
        const int new_h = dist + (int)(m_eg_eps * h);
        if (new_h < best_h) {
            best_h = new_h;
        }
    }

    return best_h;
}

int GenericEgraphHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int GenericEgraphHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

} // namespace motion
} // namespace sbpl
