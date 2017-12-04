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

#include <smpl/search/meta_mhastar_dts.h>

// standard includes
#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>

// system includes
#include <sbpl/utils/key.h>

// project includes
#include <smpl/time.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>

namespace sbpl {

MetaMultiHeuristicAstarDTS::MetaMultiHeuristicAstarDTS(
    DiscreteSpaceInformation* environment,
    Heuristic* hanchor,
    Heuristic** heurs,
    int hcount)
:
    SBPLPlanner(),
    m_hanchor(hanchor),
    m_heurs(heurs),
    m_hcount(hcount),
    m_params(0.0),
    m_max_expansions(0),
    m_eps(1.0),
    m_eps_satisfied((double)INFINITECOST),
    m_num_expansions(0),
    m_elapsed(0.0),
    m_call_number(0), // uninitialized
    m_start_state(nullptr),
    m_goal_state(nullptr),
    m_rng(),
    m_uniform(0.0, 1.0),
    m_search_states(),
    m_open(nullptr)
{
    SMPL_INFO("Construct Focal MHA* Search with %d heuristics", hcount);
    environment_ = environment;

    m_open = new rank_pq[hcount + 1];

    // Overwrite default members for ReplanParams to represent a single optimal
    // search
    m_params.initial_eps = 1.0;
    m_params.final_eps = 1.0;
    m_params.dec_eps = 0.0;
    m_params.return_first_solution = false;
    m_params.max_time = 0.0;
    m_params.repair_time = 0.0;

    m_best_hvals.resize(hcount);
    m_alphas.resize(hcount);
    m_betas.resize(hcount);
}

MetaMultiHeuristicAstarDTS::~MetaMultiHeuristicAstarDTS()
{
    clear();

    delete[] m_open;
}

int MetaMultiHeuristicAstarDTS::set_start(int start_state_id)
{
    SMPL_INFO("Set start to %d", start_state_id);
    m_start_state = get_state(start_state_id);
    if (!m_start_state) {
        return 0;
    } else {
        return 1;
    }
}

int MetaMultiHeuristicAstarDTS::set_goal(int goal_state_id)
{
    SMPL_INFO("Set goal to %d", goal_state_id);
    m_goal_state = get_state(goal_state_id);
    if (!m_goal_state) {
        return 0;
    } else {
        return 1;
    }
}

int MetaMultiHeuristicAstarDTS::replan(
    double allocated_time_sec,
    std::vector<int>* solution)
{
    int solcost;
    return replan(allocated_time_sec, solution, &solcost);
}

int MetaMultiHeuristicAstarDTS::replan(
    double allocated_time_sec,
    std::vector<int>* solution,
    int* solcost)
{
    ReplanParams params = m_params;
    params.max_time = allocated_time_sec;
    return replan(solution, params, solcost);
}

int MetaMultiHeuristicAstarDTS::replan(
    std::vector<int>* solution,
    ReplanParams params)
{
    int solcost;
    return replan(solution, params, &solcost);
}

int MetaMultiHeuristicAstarDTS::replan(
    std::vector<int>* solution,
    ReplanParams params,
    int* solcost)
{
    SMPL_INFO("Call replan");
    if (!check_params(params)) { // errors printed within
        SMPL_WARN(" -> Parameters invalid");
        return 0;
    }

    m_params = params;

    SBPL_INFO("Generic Search parameters:");
    SBPL_INFO("  Initial Epsilon: %0.3f", m_params.initial_eps);
    SBPL_INFO("  Final Epsilon: %0.3f", m_params.final_eps);
    SBPL_INFO("  Delta Epsilon: %0.3f", m_params.dec_eps);
    SBPL_INFO("  Return First Solution: %s", m_params.return_first_solution ? "true" : "false");
    SBPL_INFO("  Max Time: %0.3f", m_params.max_time);
    SBPL_INFO("  Repair Time: %0.3f", m_params.repair_time);
    SBPL_INFO("MHA Search parameters:");
    SBPL_INFO("  Max Expansions: %d", m_max_expansions);

    environment_->EnsureHeuristicsUpdated(true); // TODO: support backwards search

    // TODO: pick up from where last search left off and detect lazy
    // reinitializations
    reinit_search();

    m_eps = m_params.initial_eps;
    m_eps_satisfied = (double)INFINITECOST;

    // reset time limits
    m_num_expansions = 0;
    m_elapsed = 0.0;

    auto start_time = sbpl::clock::now();

    ++m_call_number;
    reinit_state(m_goal_state);
    reinit_state(m_start_state);
    m_start_state->g = 0;

    SMPL_INFO("Insert start state into OPEN and PSET");

    // insert start state into OPEN with g(s) + h(s) as the priority
    // insert start state into PSET and place in all RANK lists
    m_start_state->od[0].f = compute_key(m_start_state, 0);
    m_open[0].push(&m_start_state->od[0]);
    for (int hidx = 1; hidx < num_heuristics(); ++hidx) {
        m_best_hvals[hidx - 1] = m_start_state->od[hidx].h;
        m_start_state->od[hidx].f = compute_key(m_start_state, hidx);
        m_open[hidx].push(&m_start_state->od[hidx]);
        SMPL_INFO("Inserted start state %d into search %d with f = %d", m_start_state->state_id, hidx, m_start_state->od[hidx].f);
    }
    std::fill(m_alphas.begin(), m_alphas.end(), 1.0);
    std::fill(m_betas.begin(), m_betas.end(), 1.0);
    m_rng.seed(0);

    auto end_time = sbpl::clock::now();
    m_elapsed += to_seconds(end_time - start_time);

    const int anchor_freq = m_hcount;
    int iter_count = 0;

    while (!m_open[0].empty() && !time_limit_reached()) {
        auto start_time = sbpl::clock::now();

        // check termination criteria
        if (m_goal_state->g <= m_eps * get_minf(m_open[0])) {
            m_eps_satisfied = m_eps;
            extract_path(solution, solcost);
            return 1;
        }

        if (m_hcount == 0) {
            // degrade weighted A*, expand from the anchor queue
            continue;
        }

        int hidx = choose_search();

        if (!m_open[hidx].empty()) {
            MHASearchState* s = select_state(hidx);
            expand(s, hidx);
            s->closed_in_add = true;

            update_meta_method(hidx);
        }

        ++iter_count;

        // expand from the anchor queue
        if (!m_open[0].empty() && (iter_count % anchor_freq) == 0) {
            if (m_goal_state->g <= m_eps * get_minf(m_open[0])) {
                m_eps_satisfied = m_eps;
                extract_path(solution, solcost);
                return 1;
            }

            MHASearchState* s = state_from_open_state(m_open[0].min());
            expand(s, 0);
            s->closed_in_anc = true;
        }

        auto end_time = sbpl::clock::now();
        m_elapsed += to_seconds(end_time - start_time);
    }

    if (m_open[0].empty()) {
        SMPL_INFO("Anchor search exhausted");
    }
    if (time_limit_reached()) {
        SMPL_INFO("Time limit reached");
    }

    return 0;
}

int MetaMultiHeuristicAstarDTS::force_planning_from_scratch()
{
    return 0;
}

int MetaMultiHeuristicAstarDTS::force_planning_from_scratch_and_free_memory()
{
    return 0;
}

void MetaMultiHeuristicAstarDTS::costs_changed(StateChangeQuery const & stateChange)
{
}

int MetaMultiHeuristicAstarDTS::set_search_mode(bool bSearchUntilFirstSolution)
{
    return m_params.return_first_solution = bSearchUntilFirstSolution;
}

void MetaMultiHeuristicAstarDTS::set_initialsolution_eps(double eps)
{
    m_params.initial_eps = eps;
}

double MetaMultiHeuristicAstarDTS::get_initial_eps()
{
    return m_params.initial_eps;
}

double MetaMultiHeuristicAstarDTS::get_solution_eps() const
{
    return m_eps_satisfied;
}

double MetaMultiHeuristicAstarDTS::get_final_epsilon()
{
    return m_eps_satisfied;
}

double MetaMultiHeuristicAstarDTS::get_final_eps_planning_time()
{
    return m_elapsed;
}

double MetaMultiHeuristicAstarDTS::get_initial_eps_planning_time()
{
    return m_elapsed;
}

int MetaMultiHeuristicAstarDTS::get_n_expands() const
{
    return m_num_expansions;
}

int MetaMultiHeuristicAstarDTS::get_n_expands_init_solution()
{
    return m_num_expansions;
}

void MetaMultiHeuristicAstarDTS::get_search_stats(std::vector<PlannerStats>* s)
{
}

void MetaMultiHeuristicAstarDTS::set_final_eps(double eps)
{
    m_params.final_eps = eps;
}

void MetaMultiHeuristicAstarDTS::set_dec_eps(double eps)
{
    m_params.dec_eps = eps;
}

void MetaMultiHeuristicAstarDTS::set_max_expansions(int expansion_count)
{
    m_max_expansions = expansion_count;
}

void MetaMultiHeuristicAstarDTS::set_max_time(double max_time)
{
    m_params.max_time = max_time;
}

double MetaMultiHeuristicAstarDTS::get_final_eps() const
{
    return m_params.final_eps;
}

double MetaMultiHeuristicAstarDTS::get_dec_eps() const
{
    return m_params.dec_eps;
}

int MetaMultiHeuristicAstarDTS::get_max_expansions() const
{
    return m_max_expansions;
}

double MetaMultiHeuristicAstarDTS::get_max_time() const
{
    return m_params.max_time;
}

bool MetaMultiHeuristicAstarDTS::check_params(const ReplanParams& params)
{
    if (params.initial_eps < 1.0) {
        SBPL_ERROR("Initial Epsilon must be greater than or equal to 1");
        return false;
    }

    if (params.final_eps > params.initial_eps) {
        SBPL_ERROR("Final Epsilon must be less than or equal to initial epsilon");
        return false;
    }

    if (params.dec_eps <= 0.0) {
        SBPL_ERROR("Delta epsilon must be strictly positive");
        return false;
    }

    if (params.return_first_solution &&
        params.max_time <= 0.0 &&
        m_max_expansions <= 0)
    {
        SBPL_ERROR("Max Time or Max Expansions must be positive");
        return false;
    }

    return true;
}

bool MetaMultiHeuristicAstarDTS::time_limit_reached() const
{
    if (m_params.return_first_solution) {
        return false;
    } else if (m_params.max_time > 0.0 && m_elapsed >= m_params.max_time) {
        return true;
    } else if (m_max_expansions > 0 && m_num_expansions >= m_max_expansions) {
        return true;
    } else {
        return false;
    }
}

MHASearchState* MetaMultiHeuristicAstarDTS::get_state(int state_id)
{
    if (m_graph_to_search_state.size() < state_id + 1) {
        m_graph_to_search_state.resize(state_id + 1, -1);
    }

    if (m_graph_to_search_state[state_id] == -1) {
        // overallocate search state for appropriate heuristic information
        const size_t state_size =
                sizeof(MHASearchState) +
                sizeof(MHASearchState::HeapData) * (m_hcount);
        MHASearchState* s = (MHASearchState*)malloc(state_size);

        // force construction to correctly initialize heap position to null
        new (s) MHASearchState;
        for (int i = 1; i < num_heuristics(); ++i) {
            new (&s->od[i]) MHASearchState::HeapData;
        }

        const size_t mha_state_idx = m_search_states.size();
        init_state(s, state_id);

        // map graph state to search state
        m_graph_to_search_state[state_id] = mha_state_idx;
        m_search_states.push_back(s);

        return s;
    } else {
        int ssidx = m_graph_to_search_state[state_id];
        return m_search_states[ssidx];
    }
}

void MetaMultiHeuristicAstarDTS::clear()
{
    clear_open_lists();

    // free states
    for (size_t i = 0; i < m_search_states.size(); ++i) {
        // unmap graph to search state
        MHASearchState* search_state = m_search_states[i];

        // free search state
        free(m_search_states[i]);
    }

    // empty state table
    m_search_states.clear();

    m_start_state = nullptr;
    m_goal_state = nullptr;
}

void MetaMultiHeuristicAstarDTS::init_state(
    MHASearchState* state,
    int state_id)
{
    state->call_number = 0; // not initialized for any iteration
    state->state_id = state_id;
    state->closed_in_anc = false;
    state->closed_in_add = false;
    for (int i = 0; i < num_heuristics(); ++i) {
        state->od[i].f = compute_heuristic(state->state_id, i);
        state->od[i].me = state;
    }

    SMPL_DEBUG_STREAM("Initialized state: " << *state);
    for (int i = 0; i < num_heuristics(); ++i) {
        SMPL_DEBUG("  me[%d]: %p", i, state->od[i].me);
        SMPL_DEBUG("  h[%d]: %d", i, state->od[i].h);
        SMPL_DEBUG("  f[%d]: %d", i, state->od[i].f);
    }
}

// Reinitialize the state for a new search. Maintains the state id. Resets the
// cost-to-go to infinity. Removes the state from both closed lists. Recomputes
// all heuristics for the state. Does NOT remove from the OPEN or PSET lists.
void MetaMultiHeuristicAstarDTS::reinit_state(MHASearchState* state)
{
    if (state->call_number != m_call_number) {
        state->call_number = m_call_number;
        state->g = INFINITECOST;
        state->bp = nullptr;

        state->closed_in_anc = false;
        state->closed_in_add = false;

        for (int i = 0; i < num_heuristics(); ++i) {
            state->od[i].h = compute_heuristic(state->state_id, i);
            state->od[i].f = INFINITECOST;
        }

        SMPL_DEBUG_STREAM("Reinitialized state: " << *state);
        for (int i = 0; i < num_heuristics(); ++i) {
            SMPL_DEBUG("  me[%d]: %p", i, state->od[i].me);
            SMPL_DEBUG("  h[%d]: %d", i, state->od[i].h);
            SMPL_DEBUG("  f[%d]: %d", i, state->od[i].f);
        }
    }
}

void MetaMultiHeuristicAstarDTS::reinit_search()
{
    clear_open_lists();
}

void MetaMultiHeuristicAstarDTS::clear_open_lists()
{
    for (int i = 0; i < num_heuristics(); ++i) {
        m_open[i].clear();
    }
}

int MetaMultiHeuristicAstarDTS::compute_key(MHASearchState* state, int hidx)
{
    if (hidx == 0) {
        return state->g + state->od[hidx].h;
    } else {
        return state->od[hidx].h;
    }
}

int MetaMultiHeuristicAstarDTS::choose_search()
{
    std::vector<double> r(m_hcount);
    for (int hidx = 0; hidx < m_hcount; ++hidx) {
        boost::math::beta_distribution<double> dist(m_alphas[hidx], m_betas[hidx]);
        r[hidx] = quantile(dist, m_uniform(m_rng));
    }
    SMPL_INFO_STREAM("Choose Search from: " << r);
    return std::distance(r.begin(), std::max_element(r.begin(), r.end())) + 1;
}

void MetaMultiHeuristicAstarDTS::update_meta_method(int hidx)
{
    SMPL_INFO("Update Meta Method:");
    SMPL_INFO_STREAM("  pre-alphas: " << m_alphas);
    SMPL_INFO_STREAM("  pre-betas: " << m_betas);
    int new_best_hval = m_open[hidx].min()->h;
    if (new_best_hval < m_best_hvals[hidx - 1]) {
        SMPL_INFO("Update[%d] Good! :D", hidx);
        m_best_hvals[hidx - 1] = new_best_hval;
        m_alphas[hidx - 1] += 1.0;
    } else {
        SMPL_INFO("Update[%d] Bad! D:", hidx);
        m_betas[hidx - 1] += 1.0;
    }
    SMPL_INFO_STREAM("  alphas: " << m_alphas);
    SMPL_INFO_STREAM("  betas: " << m_betas);

    const double C = 100.0;
    if (m_alphas[hidx - 1] + m_betas[hidx - 1] > C) {
        double mod = C / (C + 1);
        SMPL_INFO("Renormalize! a: %0.3f, b: %0.3f -> a: %0.3f, b: %0.3f", m_alphas[hidx - 1], m_betas[hidx - 1], m_alphas[hidx - 1] * mod, m_betas[hidx - 1] * mod);
        m_alphas[hidx - 1] *= mod;
        m_betas[hidx - 1] *= mod;
    }
}

void MetaMultiHeuristicAstarDTS::expand(MHASearchState* state, int hidx)
{
    SMPL_INFO("Expanding state %d in search %d", state->state_id, hidx);

    assert(!closed_in_add_search(state) || !closed_in_anc_search(state));

    ++m_num_expansions;

    // remove s from OPEN and all P-SETs
    for (int hidx = 0; hidx < num_heuristics(); ++hidx) {
        if (m_open[hidx].contains(&state->od[hidx])) {
            m_open[hidx].erase(&state->od[hidx]);
        }
    }

    std::vector<int> succ_ids;
    std::vector<int> costs;
    environment_->GetSuccs(state->state_id, &succ_ids, &costs);
    assert(succ_ids.size() == costs.size());

    for (size_t sidx = 0; sidx < succ_ids.size(); ++sidx)  {
        const int cost = costs[sidx];
        MHASearchState* succ_state = get_state(succ_ids[sidx]);
        reinit_state(succ_state);

        int new_g = state->g + costs[sidx];
        if (new_g < succ_state->g) {
            succ_state->g = new_g;
            succ_state->bp = state;
            if (!closed_in_anc_search(succ_state)) {
                succ_state->od[0].f = compute_key(succ_state, 0);
                insert_or_update(succ_state, 0);

                // unless it's been closed in an inadmissible search...
                if (closed_in_add_search(succ_state)) {
                    continue;
                }

                // insert into the P-SET for each heuristic
                for (int hidx = 1; hidx < num_heuristics(); ++hidx) {
                    succ_state->od[hidx].f = compute_key(succ_state, hidx);
                    insert_or_update(succ_state, hidx);
                }
            }
        }
    }
}

MHASearchState* MetaMultiHeuristicAstarDTS::state_from_open_state(
    MHASearchState::HeapData* open_state)
{
    return open_state->me;
}

int MetaMultiHeuristicAstarDTS::compute_heuristic(int state_id, int hidx)
{
    if (hidx == 0) {
        return m_hanchor->GetGoalHeuristic(state_id);
    } else {
        return m_heurs[hidx - 1]->GetGoalHeuristic(state_id);
    }
}

int MetaMultiHeuristicAstarDTS::get_minf(rank_pq& pq) const
{
    return pq.min()->f;
}

void MetaMultiHeuristicAstarDTS::insert_or_update(MHASearchState* state, int hidx)
{
    if (m_open[hidx].contains(&state->od[hidx])) {
        m_open[hidx].update(&state->od[hidx]);
    } else {
        m_open[hidx].push(&state->od[hidx]);
    }
}

MHASearchState* MetaMultiHeuristicAstarDTS::select_state(int hidx)
{
    MHASearchState* state = state_from_open_state(m_open[hidx].min());
    MHASearchState::HeapData* min_open = m_open[0].min();
    if (state->od[0].f <= m_eps * min_open->f) {
        return state;
    } else {
        for (auto it = std::next(m_open[hidx].begin()); it != m_open[hidx].end(); ++it) {
            state = state_from_open_state(*it);
            if (state->od[0].f <= m_eps * min_open->f) {
                return state;
            }
        }
    }
    return nullptr;
}

void MetaMultiHeuristicAstarDTS::extract_path(std::vector<int>* solution_path, int* solcost)
{
    SMPL_INFO("Extracting path");
    solution_path->clear();
    *solcost = 0;
    for (MHASearchState* state = m_goal_state; state; state = state->bp)
    {
        solution_path->push_back(state->state_id);
        if (state->bp) {
            *solcost += (state->g - state->bp->g);
        }
    }

    // TODO: special cases for backward search
    std::reverse(solution_path->begin(), solution_path->end());
}

bool MetaMultiHeuristicAstarDTS::closed_in_anc_search(MHASearchState* state) const
{
    return state->closed_in_anc;
}

bool MetaMultiHeuristicAstarDTS::closed_in_add_search(MHASearchState* state) const
{
    return state->closed_in_add;
}

bool MetaMultiHeuristicAstarDTS::closed_in_any_search(MHASearchState* state) const
{
    return state->closed_in_anc || state->closed_in_add;
}

} // namespace sbpl
