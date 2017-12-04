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

#ifndef SMPL_GENERIC_EGRAPH_HEURISTIC_H
#define SMPL_GENERIC_EGRAPH_HEURISTIC_H

// project includes
#include <smpl/intrusive_heap.h>
#include <smpl/graph/experience_graph_extension.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/heuristic/egraph_heuristic.h>

namespace sbpl {
namespace motion {

class GenericEgraphHeuristic :
    public RobotHeuristic,
    public ExperienceGraphHeuristicExtension
{
public:

    bool init(RobotPlanningSpace* space, RobotHeuristic* h);

    double weightEGraph() const { return m_eg_eps; }
    void setWeightEGraph(double w);

    /// \name ExperienceGraphHeuristicExtension Interface
    ///@{
    void getEquivalentStates(int state_id, std::vector<int>& ids) override;
    void getShortcutSuccs(int state_id, std::vector<int>& ids) override;
    ///@}

    /// \name RobotHeuristic Interface
    ///@{
    double getMetricStartDistance(double x, double y, double z) override;
    double getMetricGoalDistance(double x, double y, double z) override;
    ///@}

    /// \name Extension Interface
    ///@{
    Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name RobotPlanningSpaceObserver Interface
    ///@{
    void updateGoal(const GoalConstraint& goal) override;
    ///@}

    /// \name Heuristic Interface
    ///@{
    int GetGoalHeuristic(int state_id) override;
    int GetStartHeuristic(int state_id) override;
    int GetFromToHeuristic(int from_id, int to_id) override;
    ///@}

private:

    static const int Unknown = std::numeric_limits<int>::max() >> 1;
    static const int Wall = std::numeric_limits<int>::max();
    static const int Infinity = Unknown;

    RobotHeuristic* m_orig_h = nullptr;

    ExperienceGraphExtension* m_eg = nullptr;

    double m_eg_eps = 1.0;

    std::vector<int> m_component_ids;
    std::vector<std::vector<ExperienceGraph::node_id>> m_shortcut_nodes;

    struct HeuristicNode : public heap_element
    {
        int dist;

        HeuristicNode() = default;
        HeuristicNode(int d) : heap_element(), dist(d) { }
    };

    struct NodeCompare
    {
        bool operator()(const HeuristicNode& a, const HeuristicNode& b) const
        {
            return a.dist < b.dist;
        }
    };

    std::vector<HeuristicNode> m_h_nodes;
    intrusive_heap<HeuristicNode, NodeCompare> m_open;
};

} // namespace motion
} // namespace sbpl

#endif
