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

#ifndef SMPL_EGRAPH_PLANNER_H
#define SMPL_EGRAPH_PLANNER_H

// system includes
#include <sbpl/planners/planner.h>

// project includes
#include <smpl/intrusive_heap.h>
#include <smpl/graph/experience_graph_extension.h>
#include <smpl/graph/robot_planning_space.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/heuristic/egraph_heuristic.h>

namespace sbpl {
namespace motion {

class ExperienceGraphPlanner : public SBPLPlanner
{
public:

    ExperienceGraphPlanner(RobotPlanningSpace* pspace, RobotHeuristic* heur);

    ~ExperienceGraphPlanner();

    /// \name Reimplemented Public Functions from SBPLPlanner
    ///@{
    int replan(std::vector<int>* solution, ReplanParams params) override;
    int replan(std::vector<int>* solution, ReplanParams params, int* cost) override;

    int     force_planning_from_scratch_and_free_memory() override;
    double  get_solution_eps() const override;
    int     get_n_expands() const override;
    double  get_initial_eps() override;
    double  get_initial_eps_planning_time() override;
    double  get_final_eps_planning_time() override;
    int     get_n_expands_init_solution() override;
    double  get_final_epsilon() override;
    void    get_search_stats(std::vector<PlannerStats>* s) override;
    void    set_initialsolution_eps(double initialsolution_eps) override;
    ///@}

    /// \name Required Public Functions from SBPLPlanner
    ///@{
    int replan(double allowed_time, std::vector<int>* solution) override;
    int replan(double allowed_time, std::vector<int>* solution, int* cost) override;
    int set_goal(int goal_state_id) override;
    int set_start(int start_state_id) override;
    int force_planning_from_scratch() override;
    int set_search_mode(bool first_solution_unbounded);
    void costs_changed(const StateChangeQuery& state_change);
    ///@}

private:

    struct SearchState : public heap_element
    {
        int state_id;
        unsigned int g;
        unsigned int h;
        unsigned int f;
        unsigned short iteration_closed;
        unsigned short call_number;
        SearchState* bp;
    };

    struct SearchStateCompare
    {
        bool operator()(const SearchState& s1, const SearchState& s2) const {
            return s1.f < s2.f;
        }
    };

    RobotPlanningSpace* m_space;
    ExperienceGraphExtension* m_ege;

    RobotHeuristic* m_heur;
    ExperienceGraphHeuristicExtension* m_egh;

    std::vector<SearchState*> m_states;
    SearchState* m_start_state;
    SearchState* m_goal_state;

    std::vector<int> m_graph_to_search_map;

    intrusive_heap<SearchState, SearchStateCompare> m_open;

    int m_call_number;
    double m_eps;

    int m_expand_count;

    SearchState* getSearchState(int state_id);
    SearchState* createState(int state_id);
    void reinitSearchState(SearchState* state);

    void extractPath(std::vector<int>& solution, int& cost) const;
};

} // namespace motion
} // namespace sbpl

#endif
