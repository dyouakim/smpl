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

#include <smpl/search/awastar.h>

#include <algorithm>

// system includes
#include <sbpl/utils/key.h>

// project includes
#include <smpl/time.h>
#include <smpl/console/console.h>

namespace sbpl {

static const char* SLOG = "search";
static const char* SELOG = "search.expansions";

AWAStar::AWAStar(DiscreteSpaceInformation* space, Heuristic* heur)
{
    environment_ = space;
    m_space = space;
    m_heur = heur;
    m_start_state_id = -1;
    m_goal_state_id = -1;
    m_curr_eps = 1.0;
}

AWAStar::~AWAStar()
{
    for (SearchState* s : m_states) {
        delete s;
    }
}

enum ReplanResultCode
{
    SUCCESS             =  0,
    PARTIAL_SUCCESS     = -1,
    START_NOT_SET       = -2,
    GOAL_NOT_SET        = -3,
    TIMED_OUT           = -4,
    EXHAUSTED_OPEN_LIST = -5,
};

int AWAStar::replan(
    double allowed_time_secs,
    std::vector<int>* solution,
    int* cost)
{
    auto allowed_time = to_duration(allowed_time_secs);
    SMPL_DEBUG_NAMED(SLOG, "Find path to goal");

    if (m_start_state_id < 0) {
        SMPL_ERROR_NAMED(SLOG, "Start state not set");
        return !START_NOT_SET;
    }
    if (m_goal_state_id < 0) {
        SMPL_ERROR_NAMED(SLOG, "Goal state not set");
        return !GOAL_NOT_SET;
    }

    SearchState* start_state = getSearchState(m_start_state_id);
    SearchState* goal_state = getSearchState(m_goal_state_id);

    if (m_last_start_id != m_start_state_id ||
        m_last_goal_id != m_goal_state_id)
    {
        SMPL_DEBUG_NAMED(SLOG, "Begin new search");
        m_open.clear();
        m_window_size = 0;
        m_call_number++;

        reinitSearchState(start_state);
        reinitSearchState(goal_state);

        start_state->g = 0;
        start_state->level = 0;
        start_state->f = computeKey(start_state);
        m_open.push(start_state);

//        m_sus_eps = 5;

        m_expands = 0;
    }

    // Anytime Window A* (AWA*)

    int best_sol = INFINITECOST;

    auto start_time = sbpl::clock::now();

    std::vector<int> succs;
    std::vector<int> costs;
    while (true) { // anytime loop
        while (true) { // BQWA*
            SMPL_DEBUG_NAMED(SLOG, "Begin iteration with window size = %d", m_window_size);
            m_cur_level = -1;

            // minimum f-value of all states in the suspended list
            m_min_sus = INFINITECOST;

            while (!m_open.empty()) {
                auto now = sbpl::clock::now();
                auto elapsed = now - start_time;
                if (elapsed > allowed_time) {
                    // ugh!
                    if (best_sol != INFINITECOST) {
                        extractPath(goal_state, *solution, *cost);
                        return !SUCCESS;
                    } else {
                        return !TIMED_OUT;
                    }
                }

                ++m_expands;

                SearchState* min_state = m_open.min();

                SMPL_DEBUG_NAMED(SELOG, "Expand state %d", min_state->state_id);

                m_open.pop();
                min_state->flags |= SearchState::Closed;

                // added min_sus check for BQWA*
                if (min_state->f >= best_sol ||
                    (int64_t)min_state->f >= m_sus_eps * (int64_t)m_min_sus)
                {
                    if (min_state->f < best_sol) {
                        min_state->flags &= ~SearchState::Closed;
                        m_open.push(min_state);
                    }

                    break; // update window size and backtrack
                }

                if (min_state->level <= m_cur_level - m_window_size) {
                    SMPL_DEBUG_NAMED(SELOG, "move state %d to suspend list (%d <= %d - %d)", min_state->state_id, min_state->level, m_cur_level, m_window_size);
                    min_state->flags &= ~SearchState::Closed;
                    min_state->flags |= SearchState::Suspended;
                    m_suspended.push_back(min_state);
                    m_min_sus = std::min(m_min_sus, min_state->f);
                    continue;
                }

                if (min_state->level > m_cur_level) {
                    m_cur_level = min_state->level;
                    SMPL_DEBUG_NAMED(SLOG, "increase level to %d", m_cur_level);
                }

                if (min_state == goal_state) {
                    SMPL_DEBUG_NAMED(SLOG, "Peeked the goal state");
                    // TODO: return best solution
                    best_sol = min_state->f;
                    break;
                }

                succs.clear();
                costs.clear();
                m_space->GetSuccs(min_state->state_id, &succs, &costs);
                for (size_t sidx = 0; sidx < succs.size(); ++sidx) {
                    int succ_state_id = succs[sidx];
                    int cost = costs[sidx];

                    SearchState* succ_state = getSearchState(succ_state_id);
                    reinitSearchState(succ_state);

                    int new_g = min_state->g + cost;
                    int new_f = new_g + (unsigned int)(m_curr_eps * succ_state->h);

                    if (!m_open.contains(succ_state) &&
                        !(succ_state->flags & SearchState::Closed) &&
                        !(succ_state->flags & SearchState::Suspended))
                    {
                        succ_state->bp = min_state;
                        succ_state->g = min_state->g + cost;
                        succ_state->f = new_f;
                        succ_state->level = min_state->level + 1;
                        m_open.push(succ_state);
                    } else if (m_open.contains(succ_state) || (succ_state->flags & SearchState::Suspended)) {
                        if (new_f < succ_state->f) {
                            succ_state->bp = min_state;
                            succ_state->f = new_f;
                            succ_state->level = min_state->level + 1;
                            if (m_open.contains(succ_state)) {
                                m_open.decrease(succ_state);
                            } else {
                                m_open.push(succ_state);
                            }
                        }
                    } else if (succ_state->flags & SearchState::Closed) {
                        if (new_f < succ_state->f) {
                            succ_state->bp = min_state;
                            succ_state->f = new_f;
                            succ_state->level = min_state->level + 1;
                            m_open.push(succ_state);
                        }
                    }
                }
            }

            SMPL_DEBUG_NAMED(SLOG, "open list = suspend list");
            for (auto* s : m_suspended) {
                s->flags &= ~SearchState::Suspended;
                m_open.push(s);
            }
            m_suspended.clear();
            ++m_window_size;

            if (m_open.empty()) {
                SMPL_DEBUG_NAMED(SLOG, "Found the bounded-optimal solution?");
                break;
            }
        }

        if (m_suspended.empty()) {
            SMPL_DEBUG_NAMED(SLOG, "Return the optimal solution");
            break;
        }

        // add openlist to closedlist
        SMPL_DEBUG_NAMED(SLOG, "closed list = open list");
        while (!m_open.empty()) {
            auto* s = m_open.min();
            s->flags |= SearchState::Closed;
            m_open.pop();
        }

        // move suspend list to open list
        SMPL_DEBUG_NAMED(SLOG, "open list = suspend list");
        for (auto* s : m_suspended) {
            s->flags &= ~SearchState::Suspended;
            m_open.push(s);
        }
        m_suspended.clear();

        SMPL_DEBUG_NAMED(SLOG, "Found solution with eps = %ld", m_sus_eps);
        m_sus_eps -= 1;
    }

    if (best_sol != INFINITECOST) {
        extractPath(goal_state, *solution, *cost);
        return !SUCCESS;
    }

    return !EXHAUSTED_OPEN_LIST;
}

int AWAStar::replan(std::vector<int>* solution, ReplanParams params)
{
    int cost;
    return replan(solution, params.max_time, &cost);
}

int AWAStar::replan(std::vector<int>* solution, ReplanParams params, int* cost)
{
    return replan(params.max_time, solution, cost);
}

/// Force the planner to forget previous search efforts, begin from scratch,
/// and free all memory allocated by the planner during previous searches.
int AWAStar::force_planning_from_scratch_and_free_memory()
{
    return 0;
}

/// Return the suboptimality bound of the current solution for the current search.
double AWAStar::get_solution_eps() const
{
    return 0.0;
}

/// Return the initial suboptimality bound
double AWAStar::get_initial_eps()
{
    return 0.0;
}

/// Return the time consumed by the search in progress to the initial solution.
double AWAStar::get_initial_eps_planning_time()
{
    return 0.0;
}

/// Return the time consumed by the search in progress to the final solution.
double AWAStar::get_final_eps_planning_time()
{
    return 0.0;
}

/// Return the number of expansions made in progress to the initial solution.
int AWAStar::get_n_expands_init_solution()
{
    return 0;
}

/// Return the final suboptimality bound.
double AWAStar::get_final_epsilon()
{
    return 0.0;
}

/// Return statistics for each completed search iteration.
void AWAStar::get_search_stats(std::vector<PlannerStats>* s)
{
}

/// Set the goal state.
int AWAStar::set_goal(int goal_state_id)
{
    m_goal_state_id = goal_state_id;
    return 1;
}

/// Set the start state.
int AWAStar::set_start(int start_state_id)
{
    m_start_state_id = start_state_id;
    return 1;
}

/// Force the search to forget previous search efforts and start from scratch.
int AWAStar::force_planning_from_scratch()
{
    return 0;
}

/// Set whether the number of expansions is bounded by time or total expansions
/// per call to replan().
int AWAStar::set_search_mode(bool first_solution_unbounded)
{
    return 0;
}

/// Notify the search of changes to edge costs in the graph.
void AWAStar::costs_changed(const StateChangeQuery& changes)
{
    force_planning_from_scratch();
}

int AWAStar::computeKey(SearchState* s) const
{
    return s->g + (unsigned int)(m_curr_eps * s->h);
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
AWAStar::SearchState* AWAStar::getSearchState(int state_id)
{
    if (state_id >= m_states.size()) {
        auto prev_cap = m_states.capacity();
        m_states.resize(state_id + 1, nullptr);
        auto cap = m_states.capacity();
    }

    if (m_states[state_id] == nullptr) {
        m_states[state_id] = new SearchState;
        m_states[state_id]->call_number = 0;
        m_states[state_id]->state_id = state_id;
    }

    return m_states[state_id];
}

// Lazily (re)initialize a search state.
void AWAStar::reinitSearchState(SearchState* state)
{
    if (state->call_number != m_call_number) {
//        SMPL_DEBUG_NAMED(SELOG, "Reinitialize state %d", state->state_id);
        state->g = INFINITECOST;
        state->h = m_heur->GetGoalHeuristic(state->state_id);
        state->f = INFINITECOST;
        state->flags = 0;
        state->level = -1;
        state->call_number = m_call_number;
        state->bp = nullptr;
    }
}

// Extract the path from the start state up to a new state.
void AWAStar::extractPath(
    SearchState* to_state,
    std::vector<int>& solution,
    int& cost) const
{
    for (SearchState* s = to_state; s; s = s->bp) {
        solution.push_back(s->state_id);
    }
    std::reverse(solution.begin(), solution.end());
    cost = to_state->g;
}

} // namespace sbpl
