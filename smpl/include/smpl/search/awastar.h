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

#ifndef SMPL_AWASTAR_H
#define SMPL_AWASTAR_H

// system includes
#include <sbpl/heuristics/heuristic.h>
#include <sbpl/planners/planner.h>

// project includes
#include <smpl/intrusive_heap.h>
#include <smpl/time.h>

namespace sbpl {

/// An implementation of the ARA* (Anytime Repairing A*) search algorithm. This
/// algorithm runs a series of weighted A* searches with decreasing bounds on
/// suboptimality to return the best solution found within a given time bound.
/// The search intelligently reuses its search tree between successive
/// iterations for improved efficiency, rather than starting each new
/// weighted-A* iteration from scratch.
///
/// This class maintains the state of the search procedure between calls to
/// replan(), allowing the search to resume from where it left off when the
/// scenario (start, goal, and edge costs in the graph) doesn't change between
/// calls. This can be used to dedicate more time to searching in the event the
/// search fails to find a solution within the given time and to allow solutions
/// to be returned quickly and allowing the search to continue improving the
/// solution given more time. To implement this, several assumptions about the
/// implementation of the graph and heuristic are made:
///
/// * The state IDs are constant between calls to replan(). If the state ID for
///   any state the search has encountered so far (via state expansions or
///   setting the start or goal) changes, the search will be invalid.
///
/// * Changes to the goal state are reflected by changes to the goal state ID.
///   Often, many graph representations that support multiple or underdefined
///   goal states will represent the goal state given to the planner using a
///   single goal state ID. If this is the case, the caller will have to assert
///   whether or not the goal has changed, and force the planner to reinitialize
///   by calls for force_planning_from_scratch (TODO: shouldn't require full
///   reinitialization)
///
/// * The heuristics for any encountered states remain constant, unless the goal
///   state ID has changed.
struct AWAStar : public SBPLPlanner
{

    AWAStar(DiscreteSpaceInformation* space, Heuristic* heuristic);
    ~AWAStar();

    /// \name Required Functions from SBPLPlanner
    ///@{
    int replan(double allowed_time_secs, std::vector<int>* solution) override {
        return replan(allowed_time_secs, solution, nullptr);
    }

    int replan(double allowed_time_secs, std::vector<int>* solution, int* solcost) override;
    int set_goal(int state_id) override;
    int set_start(int state_id) override;
    int force_planning_from_scratch() override;
    int set_search_mode(bool bSearchUntilFirstSolution) override;
    void costs_changed(const StateChangeQuery& stateChange) override;
    ///@}

    /// \name Reimplemented Functions from SBPLPlanner
    ///@{
    int replan(std::vector<int>* solution, ReplanParams params) override;
    int replan(std::vector<int>* solution, ReplanParams params, int* solcost) override;
    int force_planning_from_scratch_and_free_memory() override;
    double get_solution_eps() const override;
    int get_n_expands() const override { return m_expands; }
    double get_initial_eps() override;
    double get_initial_eps_planning_time() override;
    double get_final_eps_planning_time() override;
    int get_n_expands_init_solution() override;
    double get_final_epsilon() override;
    void get_search_stats(std::vector<PlannerStats>* s) override;
    void set_initialsolution_eps(double eps) override { m_sus_eps = eps; }
    ///@}

    struct SearchState : public heap_element
    {
        SearchState* bp;
        enum Flags {
            Closed = (1 << 0),
            Suspended = (1 << 1),
        };

        int state_id;
        int g;     // cost-to-come
        int h;     // estimated cost-to-go
        int f;     // (g + eps * h) at time of insertion into OPEN
        int level;
        short call_number;
        std::uint8_t flags;
    };

    struct SearchStateCompare
    {
        bool operator()(const SearchState& s1, const SearchState& s2) const {
            return s1.f < s2.f;
        }
    };

    using OpenList = intrusive_heap<SearchState, SearchStateCompare>;

    DiscreteSpaceInformation*   m_space = nullptr;
    Heuristic*                  m_heur = nullptr;

    std::vector<SearchState*>   m_states;
    OpenList                    m_open;
    std::vector<SearchState*>   m_suspended;

    double                      m_curr_eps = 1.0;

    int                         m_start_state_id = -1;
    int                         m_goal_state_id = -1;

    int                         m_call_number = 0;
    int                         m_last_start_id = -1;
    int                         m_last_goal_id = -1;

    int                         m_window_size;
    int64_t                     m_sus_eps = 1;
    int                         m_cur_level;
    int                         m_min_sus;
    int                         m_expands;

    // search state (not including the values of g, f, back pointers, and
    // closed list from m_stats)

    int computeKey(SearchState* s) const;

    SearchState* getSearchState(int state_id);
    SearchState* createState(int state_id);
    void reinitSearchState(SearchState* state);

    void extractPath(
        SearchState* to_state,
        std::vector<int>& solution,
        int& cost) const;
};

} // namespace sbpl

#endif
