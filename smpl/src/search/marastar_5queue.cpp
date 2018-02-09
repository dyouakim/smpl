
#include <smpl/search/marastar.h>

#include <algorithm>

// system includes
#include <sbpl/utils/key.h>

// project includes
#include <smpl/time.h>

namespace sbpl {

static const char* SLOG = "search";
static const char* SELOG = "search.expansions";

MARAStar::MARAStar(
    DiscreteSpaceInformation* space,
    Heuristic* heur)
:
    SBPLPlanner(),
    m_space((sbpl::motion::ManipLattice*)space),
    m_heur(heur),
    m_time_params(),
    m_initial_eps(1.0),
    m_final_eps(1.0),
    m_delta_eps(1.0),
    m_allow_partial_solutions(false),
    m_states(),
    m_start_state_id(-1),
    m_goal_state_id(-1),
    m_incons(),
    m_curr_eps(1.0),
    m_open(),
    m_iteration(1),
    m_call_number(0),
    m_last_start_state_id(-1),
    m_last_goal_state_id(-1),
    m_last_eps(1.0),
    m_expand_count_init(0),
    m_expand_count(0),
    m_search_time_init(clock::duration::zero()),
    m_search_time(clock::duration::zero()),
    m_satisfied_eps(std::numeric_limits<double>::infinity())
{
    environment_ = space;

    m_time_params.bounded = true;
    m_time_params.improve = true;
    m_time_params.type = TimeParameters::TIME;
    m_time_params.max_expansions_init = 0;
    m_time_params.max_expansions = 0;
    m_time_params.max_allowed_time_init = clock::duration::zero();
    m_time_params.max_allowed_time = clock::duration::zero();
    min_heuristic_found[0] = -1;
    min_heuristic_found[1] = -1;
}

MARAStar::~MARAStar()
{
    for (SearchState* s : m_states) {
        if (s != NULL) {
            delete s;
        }
    }
}

enum ReplanResultCode
{
    SUCCESS = 0,
    PARTIAL_SUCCESS,
    START_NOT_SET,
    GOAL_NOT_SET,
    TIMED_OUT,
    EXHAUSTED_OPEN_LIST
};

int MARAStar::replan(
    const TimeParameters& params,
    std::vector<int>* solution,
    int* cost)
{
    SMPL_DEBUG_NAMED(SLOG, "Find path to goal");

    if (m_start_state_id < 0) {
        SMPL_ERROR_NAMED(SLOG, "Start state not set");
        return !START_NOT_SET;
    }
    if (m_goal_state_id < 0) {
        SMPL_ERROR_NAMED(SLOG, "Goal state not set");
        return !GOAL_NOT_SET;
    }

    m_time_params = params;

    SearchState* start_state = getSearchState(m_start_state_id);
    SearchState* goal_state = getSearchState(m_goal_state_id);

    if (m_start_state_id != m_last_start_state_id) {
        SMPL_DEBUG_NAMED(SLOG, "Reinitialize search");
        m_open.clear(sbpl::motion::BaseGroupHeuristic::B1);
        m_open.clear(sbpl::motion::BaseGroupHeuristic::B2);
        m_open.clear(sbpl::motion::BaseGroupHeuristic::B3);
        m_open.clear(sbpl::motion::BaseGroupHeuristic::B4);
        m_open.clear(sbpl::motion::GroupType::ARM);
        m_incons.clear();
        ++m_call_number; // trigger state reinitializations

        reinitSearchState(start_state);
        reinitSearchState(goal_state);
        SMPL_WARN_STREAM("Goal state f is "<<goal_state->f);
        /*reinitSearchState(goal_state,sbpl::motion::GroupType::ARM, sbpl::motion::BaseGroupHeuristic::NONE);
        reinitSearchState(goal_state,sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B1);
        reinitSearchState(goal_state,sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B2);
        reinitSearchState(goal_state,sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B3);
        reinitSearchState(goal_state,sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B4);
*/
        start_state->g = 0;
        start_state->f[0] = computeKey(start_state, sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B1);
        m_open.push(start_state,sbpl::motion::BaseGroupHeuristic::B1);
        start_state->f[1] = computeKey(start_state, sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B2);
        m_open.push(start_state,sbpl::motion::BaseGroupHeuristic::B2);
        start_state->f[2] = computeKey(start_state, sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B3);
        m_open.push(start_state,sbpl::motion::BaseGroupHeuristic::B3);
        start_state->f[3] = computeKey(start_state, sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B4); 
        m_open.push(start_state,sbpl::motion::BaseGroupHeuristic::B4);
        start_state->f[4] = computeKey(start_state, sbpl::motion::GroupType::ARM, sbpl::motion::BaseGroupHeuristic::NONE);
        m_open.push(start_state,sbpl::motion::GroupType::ARM);

        m_iteration = 1; // 0 reserved for "not closed on any iteration"

        m_expand_count_init = 0;
        m_search_time_init = clock::duration::zero();

        m_expand_count = 0;
        m_search_time = clock::duration::zero();

        m_curr_eps = m_initial_eps;

        m_satisfied_eps = std::numeric_limits<double>::infinity();

        m_last_start_state_id = m_start_state_id;
    }

    if (m_goal_state_id != m_last_goal_state_id) {
        SMPL_DEBUG_NAMED(SLOG, "Refresh heuristics, keys, and reorder open list");
        recomputeHeuristics();
        reorderOpen();

        m_last_goal_state_id = m_goal_state_id;
    }

    auto start_time = clock::now();
    int num_expansions = 0;
    clock::duration elapsed_time = clock::duration::zero();

    int err;
    while (m_satisfied_eps > m_final_eps) {

        //TODO check how to handle this for the multi search thing
        /*if (m_curr_eps == m_satisfied_eps) {
            if (!m_time_params.improve) {
                break;
            }
            // begin a new search iteration
            ++m_iteration;
            m_curr_eps -= m_delta_eps;
            m_curr_eps = std::max(m_curr_eps, m_final_eps);
            
            for (SearchState* s : m_incons) {
                s->incons = false;
                m_open.push(s,sbpl::motion::GroupType::BASE);
                m_open.push(s,sbpl::motion::GroupType::ARM);
            }
            reorderOpen();
            m_incons.clear();
            SMPL_DEBUG_NAMED(SLOG, "Begin new search iteration %d with epsilon = %0.3f", m_iteration, m_curr_eps);
        }*/
         err = improvePath(start_time, goal_state, num_expansions, elapsed_time);
        if (m_curr_eps == m_initial_eps) {
            m_expand_count_init += num_expansions;
            m_search_time_init += elapsed_time;
        }
        if (err) {
            break;
        }
        SMPL_DEBUG_NAMED(SLOG, "Improved solution");
        m_satisfied_eps = m_curr_eps;
    }

    m_search_time += elapsed_time;
    m_expand_count += num_expansions;

    if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
        if (m_allow_partial_solutions && !m_open.empty()) {
            SearchState* next_state = m_open.min();
            extractPath(next_state, *solution, *cost);
            return !SUCCESS;
        }
        return !err;
    }

    extractPath(goal_state, *solution, *cost);
    return !SUCCESS;
}

int MARAStar::replan(
    double allowed_time,
    std::vector<int>* solution)
{
    int cost;
    return replan(allowed_time, solution, &cost);
}

// decide whether to start the search from scratch
//
// if start changed
//     reset the search to its initial state
// if goal changed
//     reevaluate heuristics
//     reorder the open list
//
// case scenario_hasnt_changed (start and goal the same)
//   case have solution for previous epsilon
//       case epsilon lowered
//           reevaluate heuristics and reorder the open list
//       case epsilon raised
//           pass
//   case dont have solution
//       case epsilon lowered
//           reevaluate heuristics and reorder the open list
//       case epsilon raised
//           reevaluate heuristics and reorder the open list
// case scenario_changed
int MARAStar::replan(
    double allowed_time,
    std::vector<int>* solution,
    int* cost)
{
    TimeParameters tparams = m_time_params;
    
    if (tparams.max_allowed_time_init == tparams.max_allowed_time) {
        // NOTE/TODO: this may lead to awkward behavior, if the caller sets the
        // allowed time to the current repair time, the repair time will begin
        // to track the allowed time for further calls to replan. perhaps set
        // an explicit flag for using repair time or an indicator value as is
        // done with ReplanParams
        tparams.max_allowed_time_init = to_duration(allowed_time);
        tparams.max_allowed_time = to_duration(allowed_time);
    } else {
        tparams.max_allowed_time_init = to_duration(allowed_time);
        // note: retain original allowed improvement time
    }
    
    return replan(tparams, solution, cost);
}

int MARAStar::replan(
    std::vector<int>* solution,
    ReplanParams params)
{
    int cost;
    return replan(solution, params, &cost);
}

int MARAStar::replan(
    std::vector<int>* solution,
    ReplanParams params,
    int* cost)
{
    // note: if replan fails before internal time parameters are updated (this
    // happens if the start or goal has not been set), then the internal
    // epsilons may be affected by this set of ReplanParams for future calls to
    // replan where ReplanParams is not used and epsilon parameters haven't been
    // set back to their desired values.
    TimeParameters tparams;
    convertReplanParamsToTimeParams(params, tparams);
    return replan(tparams, solution, cost);
}

/// Force the planner to forget previous search efforts, begin from scratch,
/// and free all memory allocated by the planner during previous searches.
int MARAStar::force_planning_from_scratch_and_free_memory()
{
    force_planning_from_scratch();
    m_open.clear(sbpl::motion::GroupType::BASE);
    m_open.clear(sbpl::motion::GroupType::ARM);

    for (SearchState* s : m_states) {
        if (s != NULL) {
            delete s;
        }
    }
    m_states.clear();
    m_states.shrink_to_fit();
    return 0;
}

/// Return the suboptimality bound of the current solution for the current search.
double MARAStar::get_solution_eps() const
{
    return m_satisfied_eps;
}

/// Return the number of expansions made in progress to the final solution.
int MARAStar::get_n_expands() const
{
    return m_expand_count/2;
}

/// Return the initial suboptimality bound
double MARAStar::get_initial_eps()
{
    return m_initial_eps;
}

/// Return the time consumed by the search in progress to the initial solution.
double MARAStar::get_initial_eps_planning_time()
{
    return to_seconds(m_search_time_init);
}

/// Return the time consumed by the search in progress to the final solution.
double MARAStar::get_final_eps_planning_time()
{
    return to_seconds(m_search_time);
}

/// Return the number of expansions made in progress to the initial solution.
int MARAStar::get_n_expands_init_solution()
{
    return m_expand_count_init;
}

/// Return the final suboptimality bound.
double MARAStar::get_final_epsilon()
{
    return m_final_eps;
}

/// Return statistics for each completed search iteration.
void MARAStar::get_search_stats(std::vector<PlannerStats>* s)
{
    PlannerStats stats;
    stats.eps = m_curr_eps;
    stats.cost;
    stats.expands = m_expand_count;
    stats.time = to_seconds(m_search_time);
    s->push_back(stats);
}

/// Set the desired suboptimality bound for the initial solution.
void MARAStar::set_initialsolution_eps(double eps)
{
    m_initial_eps = eps;
}

/// Set the goal state.
int MARAStar::set_goal(int goal_state_id)
{
    m_goal_state_id = goal_state_id;
    return 1;
}

/// Set the start state.
int MARAStar::set_start(int start_state_id)
{
    m_start_state_id = start_state_id;
    return 1;
}

/// Force the search to forget previous search efforts and start from scratch.
int MARAStar::force_planning_from_scratch()
{
    m_last_start_state_id = -1;
    m_last_goal_state_id = -1;
    return 0;
}

/// Set whether the number of expansions is bounded by time or total expansions
/// per call to replan().
int MARAStar::set_search_mode(bool first_solution_unbounded)
{
    m_time_params.bounded = !first_solution_unbounded;
    return 0;
}

/// Notify the search of changes to edge costs in the graph.
void MARAStar::costs_changed(const StateChangeQuery& changes)
{
    force_planning_from_scratch();
}

// Recompute heuristics for all states.
void MARAStar::recomputeHeuristics()
{
    for (SearchState* s : m_states) {
        if (s != NULL) {
            s->h[0] = m_heur->GetGoalHeuristic(s->state_id, sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B1);
            s->h[1] = m_heur->GetGoalHeuristic(s->state_id, sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B2);
            s->h[2] = m_heur->GetGoalHeuristic(s->state_id, sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B3);
            s->h[3] = m_heur->GetGoalHeuristic(s->state_id, sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B4);
            s->h[4] = m_heur->GetGoalHeuristic(s->state_id, sbpl::motion::GroupType::ARM, sbpl::motion::BaseGroupHeuristic::NONE);
        }
    }
}

// Convert TimeParameters to ReplanParams. Uses the current epsilon values
// to fill in the epsilon fields.
void MARAStar::convertTimeParamsToReplanParams(
    const TimeParameters& t,
    ReplanParams& r) const
{
    r.max_time = to_seconds(t.max_allowed_time_init);
    r.return_first_solution = !t.bounded && !t.improve;
    if (t.max_allowed_time_init == t.max_allowed_time) {
        r.repair_time = -1.0;
    } else {
        r.repair_time = to_seconds(t.max_allowed_time);
    }

    r.initial_eps = m_initial_eps;
    r.final_eps = m_final_eps;
    r.dec_eps = m_delta_eps;
}

// Convert ReplanParams to TimeParameters. Sets the current initial, final, and
// delta eps from ReplanParams.
void MARAStar::convertReplanParamsToTimeParams(
    const ReplanParams& r,
    TimeParameters& t)
{
    t.type = TimeParameters::TIME;

    t.bounded = !r.return_first_solution;
    t.improve = !r.return_first_solution;

    t.max_allowed_time_init = to_duration(r.max_time);
    if (r.repair_time > 0.0) {
        t.max_allowed_time = to_duration(r.repair_time);
    } else {
        t.max_allowed_time = t.max_allowed_time_init;
    }

    m_initial_eps = r.initial_eps;
    m_final_eps = r.final_eps;
    m_delta_eps = r.dec_eps;
}

// Test whether the search has run out of time.
bool MARAStar::timedOut(
    int elapsed_expansions,
    const clock::duration& elapsed_time) const
{
    if (!m_time_params.bounded) {
        return false;
    }

    switch (m_time_params.type) {
    case TimeParameters::EXPANSIONS:
        if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_expansions >= m_time_params.max_expansions_init;
        } else {
            return elapsed_expansions >= m_time_params.max_expansions;
        }
    case TimeParameters::TIME:
        if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_time >= m_time_params.max_allowed_time_init;
        } else {
            return elapsed_time >= m_time_params.max_allowed_time;
        }
    default:
        SMPL_ERROR_NAMED(SLOG, "Invalid timer type");
        return true;
    }

    return true;
}

// Expand states to improve the current solution until a solution within the
// current suboptimality bound is found, time runs out, or no solution exists.
int MARAStar::improvePath(
    const clock::time_point& start_time,
    SearchState* goal_state,
    int& elapsed_expansions,
    clock::duration& elapsed_time)
{
    while (!m_open.empty()) {

        auto now = clock::now();
        elapsed_time = now - start_time;
        /////Expanding the base MPs///////
        SMPL_WARN("Expanding 0");
        int result = runSubGroup(goal_state, elapsed_expansions, elapsed_time,start_time, sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B1);
        
        SMPL_WARN_STREAM("sub group "<<sbpl::motion::GroupType::BASE<<","<<sbpl::motion::BaseGroupHeuristic::B1<<" result "<<result<<" Success is "<<SUCCESS);
        if (timedOut(elapsed_expansions, elapsed_time)) {
            SMPL_DEBUG_NAMED(SLOG, "Ran out of time");
            return TIMED_OUT;
        }
        if(result == SUCCESS)
            return result;
        SMPL_WARN("Expanding 1");
        result = runSubGroup(goal_state, elapsed_expansions, elapsed_time,start_time, sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B2);
        SMPL_WARN_STREAM("sub group "<<sbpl::motion::GroupType::BASE<<","<<sbpl::motion::BaseGroupHeuristic::B2<<" result "<<result);
        if (timedOut(elapsed_expansions, elapsed_time)) {
            SMPL_DEBUG_NAMED(SLOG, "Ran out of time");
            return TIMED_OUT;
        }
        if(result == SUCCESS)
            return result;

        SMPL_WARN("Expanding 2");
        result = runSubGroup(goal_state, elapsed_expansions, elapsed_time,start_time, sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B3);
        SMPL_WARN_STREAM("sub group "<<sbpl::motion::GroupType::BASE<<","<<sbpl::motion::BaseGroupHeuristic::B3<<" result "<<result);
        if (timedOut(elapsed_expansions, elapsed_time)) {
            SMPL_DEBUG_NAMED(SLOG, "Ran out of time");
            return TIMED_OUT;
        }
        if(result == SUCCESS)
            return result;

        SMPL_WARN("Expanding 3");
        result = runSubGroup(goal_state, elapsed_expansions, elapsed_time,start_time, sbpl::motion::GroupType::BASE, sbpl::motion::BaseGroupHeuristic::B4);
        SMPL_WARN_STREAM("sub group "<<sbpl::motion::GroupType::BASE<<","<<sbpl::motion::BaseGroupHeuristic::B4<<" result "<<result);
        if (timedOut(elapsed_expansions, elapsed_time)) {
            SMPL_DEBUG_NAMED(SLOG, "Ran out of time");
            return TIMED_OUT;
        }
        if(result == SUCCESS)
            return result;
        /////Expanding the arm MPs///////
        SMPL_WARN("Expanding 4");
        result = runSubGroup(goal_state, elapsed_expansions, elapsed_time,start_time, sbpl::motion::GroupType::ARM, sbpl::motion::BaseGroupHeuristic::NONE);
        SMPL_WARN_STREAM("sub group "<<sbpl::motion::GroupType::ARM<<","<<sbpl::motion::BaseGroupHeuristic::NONE<<" result "<<result);
        if (timedOut(elapsed_expansions, elapsed_time)) {
            SMPL_DEBUG_NAMED(SLOG, "Ran out of time");
            return TIMED_OUT;
        }
        if(result == SUCCESS)
            return result; 
    }

    return EXHAUSTED_OPEN_LIST;
}

int MARAStar::runSubGroup(SearchState* goal_state, int& elapsed_expansions, 
    clock::duration& elapsed_time, const clock::time_point& start_time, sbpl::motion::GroupType group, sbpl::motion::BaseGroupHeuristic base_heur)
{
    SearchState* min_state = m_open.min();

    int index=-1;
    if(group == sbpl::motion::GroupType::ARM)
        index = sbpl::motion::GroupType::ARM;
    else
        index = base_heur;

    SMPL_WARN_STREAM("MinState id is  "<<min_state->state_id<<" with f "<<min_state->f[index]);
    

    // path to goal found
    if (min_state->f[index]!= INFINITECOST && (min_state->f[index] >= goal_state->f[index] || min_state == goal_state)) {
        SMPL_DEBUG_NAMED(SLOG, "Found path to goal");
        return SUCCESS;
    }

    SMPL_DEBUG_NAMED(SELOG, "Expand state %d", min_state->state_id);
    //SMPL_WARN_STREAM("Expanding state for "<<current_planning_group<<" with id "<<min_state->state_id<< " with f-val "<<min_state->f<<" h-val "<<min_state->h);
    if(group == sbpl::motion::GroupType::ARM)
        m_open.pop(group);
    else
        m_open.pop(base_heur);

    assert(min_state->iteration_closed[sbpl::motion::BaseGroupHeuristic::B1] != m_iteration);
    assert(min_state->g != INFINITECOST);
    min_state->iteration_closed[sbpl::motion::BaseGroupHeuristic::B1] = m_iteration;

    assert(min_state->iteration_closed[sbpl::motion::BaseGroupHeuristic::B2] != m_iteration);
    assert(min_state->g != INFINITECOST);
    min_state->iteration_closed[sbpl::motion::BaseGroupHeuristic::B2] = m_iteration;

    assert(min_state->iteration_closed[sbpl::motion::BaseGroupHeuristic::B3] != m_iteration);
    assert(min_state->g != INFINITECOST);
    min_state->iteration_closed[sbpl::motion::BaseGroupHeuristic::B3] = m_iteration;

    assert(min_state->iteration_closed[sbpl::motion::BaseGroupHeuristic::B4] != m_iteration);
    assert(min_state->g != INFINITECOST);
    min_state->iteration_closed[sbpl::motion::BaseGroupHeuristic::B4] = m_iteration;

    assert(min_state->iteration_closed[sbpl::motion::GroupType::ARM] != m_iteration);
    assert(min_state->g != INFINITECOST);
    min_state->iteration_closed[sbpl::motion::GroupType::ARM] = m_iteration;

    min_state->eg = min_state->g;

    if(group == sbpl::motion::GroupType::ARM)
        expand(min_state, group, sbpl::motion::BaseGroupHeuristic::NONE);
    else
        expand(min_state, group, base_heur);

    ++ elapsed_expansions;

    return 1; 
}

// Expand a state, updating its successors and placing them into OPEN, CLOSED,
// and INCONS list appropriately.
void MARAStar::expand(SearchState* s, sbpl::motion::GroupType group, sbpl::motion::BaseGroupHeuristic base_heur)
{
    m_succs.clear();
    m_costs.clear();

    std::vector<int> base_succs, arm_succs, base_costs, arm_costs;

    m_space->GetSuccsByGroup(s->state_id, &m_succs, &m_costs, group); 
    
    int index=-1;
    if(group == sbpl::motion::GroupType::ARM)
        index = sbpl::motion::GroupType::ARM;
    else
        index = base_heur;
 
    SMPL_DEBUG_NAMED(SELOG, "group %s has  %zu successors", group, m_succs.size());
    SMPL_WARN_STREAM("Group "<<group<<" with base heur "<<base_heur<<" and index "<<index<<" has "<<m_succs.size()<<" successors.");
    
    for (size_t sidx = 0; sidx < m_succs.size(); ++sidx) {
        int succ_state_id = m_succs[sidx];
        int cost = m_costs[sidx];

        SearchState* succ_state = getSearchState(succ_state_id);
        reinitSearchState(succ_state,group, base_heur);

        int new_cost = s->eg + cost;
        //SMPL_INFO_NAMED(SELOG, "Compare new cost %d vs old cost %d", new_cost, succ_state->g);
        if (new_cost < succ_state->g) {
            succ_state->g = new_cost;
            succ_state->bp = s;

            if (succ_state->iteration_closed[sbpl::motion::BaseGroupHeuristic::B1] != m_iteration) {
                succ_state->f[index] = computeKey(succ_state, group, base_heur);
                //SMPL_WARN_STREAM("0State "<<succ_state->state_id<< " has f-val "<<succ_state->f[index]);
                if (m_open.contains(succ_state,sbpl::motion::BaseGroupHeuristic::B1)) {
                    //SMPL_WARN_STREAM("0State "<<succ_state->state_id<< " has already been expanded in base decrease priority!");
                    m_open.decrease(succ_state,sbpl::motion::BaseGroupHeuristic::B1);
                }
                else {
                    m_open.push(succ_state,sbpl::motion::BaseGroupHeuristic::B1);
                    //SMPL_WARN_STREAM("0State "<<succ_state->state_id<< " has never been expanded, added to base open list!");
                }
            } else if (!succ_state->incons) {
                m_incons.push_back(succ_state);
            }

            if (succ_state->iteration_closed[sbpl::motion::BaseGroupHeuristic::B2] != m_iteration) {
                succ_state->f[index] = computeKey(succ_state, group, base_heur);
                //SMPL_WARN_STREAM("1State "<<succ_state->state_id<< " has f-val "<<succ_state->f[index]);
                if (m_open.contains(succ_state,sbpl::motion::BaseGroupHeuristic::B2)) {
                    //SMPL_WARN_STREAM("1State "<<succ_state->state_id<< " has already been expanded in base decrease priority!");
                    m_open.decrease(succ_state,sbpl::motion::BaseGroupHeuristic::B2);
                }
                else {
                    m_open.push(succ_state,sbpl::motion::BaseGroupHeuristic::B2);
                    //SMPL_WARN_STREAM("1State "<<succ_state->state_id<< " has never been expanded, added to base open list!");
                }
            } else if (!succ_state->incons) {
                m_incons.push_back(succ_state);
            }

            if (succ_state->iteration_closed[sbpl::motion::BaseGroupHeuristic::B3] != m_iteration) {
                succ_state->f[index] = computeKey(succ_state, group, base_heur);
                //SMPL_WARN_STREAM("2State "<<succ_state->state_id<< " has f-val "<<succ_state->f[index]);
                if (m_open.contains(succ_state,sbpl::motion::BaseGroupHeuristic::B3)) {
                    //SMPL_WARN_STREAM("2State "<<succ_state->state_id<< " has already been expanded in base decrease priority!");
                    m_open.decrease(succ_state,sbpl::motion::BaseGroupHeuristic::B3);
                }
                else {
                    m_open.push(succ_state,sbpl::motion::BaseGroupHeuristic::B3);
                    //SMPL_WARN_STREAM("2State "<<succ_state->state_id<< " has never been expanded, added to base open list!");
                }
            } else if (!succ_state->incons) {
                m_incons.push_back(succ_state);
            }

            if (succ_state->iteration_closed[sbpl::motion::BaseGroupHeuristic::B4] != m_iteration) {
                succ_state->f[index] = computeKey(succ_state, group, base_heur);
                //SMPL_WARN_STREAM("3State "<<succ_state->state_id<< " has f-val "<<succ_state->f[index]);
                if (m_open.contains(succ_state,sbpl::motion::BaseGroupHeuristic::B4)) {
                    //SMPL_WARN_STREAM("3State "<<succ_state->state_id<< " has already been expanded in base decrease priority!");
                    m_open.decrease(succ_state,sbpl::motion::BaseGroupHeuristic::B4);
                }
                else {
                    m_open.push(succ_state,sbpl::motion::BaseGroupHeuristic::B4);
                    //SMPL_WARN_STREAM("3State "<<succ_state->state_id<< " has never been expanded, added to base open list!");
                }
            } else if (!succ_state->incons) {
                m_incons.push_back(succ_state);
            }

            if (succ_state->iteration_closed[sbpl::motion::GroupType::ARM] != m_iteration) {
                succ_state->f[index] = computeKey(succ_state,group, base_heur);
                //SMPL_WARN_STREAM("4State "<<succ_state->state_id<< " has f-val "<<succ_state->f[index]);
                if (m_open.contains(succ_state,sbpl::motion::GroupType::ARM)) {
                    //SMPL_WARN_STREAM("4State "<<succ_state->state_id<< " has already been expanded in arm decrease priority!");
                    m_open.decrease(succ_state,sbpl::motion::GroupType::ARM);
                }
                else {
                    //SMPL_WARN_STREAM("4State "<<succ_state->state_id<< " has never been expanded, added to arm open list!");
                    m_open.push(succ_state,sbpl::motion::GroupType::ARM);
                }
            } else if (!succ_state->incons) {
                m_incons.push_back(succ_state);
            }

        }
    }
}

// Recompute the f-values of all states in OPEN and reorder OPEN.
void MARAStar::reorderOpen()
{
    for (auto it = m_open.begin(); it != m_open.end(); ++it) {
        std::vector<double> f(5);
        (*it)->f[0] = computeKey(*it, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B1);
        (*it)->f[1] = computeKey(*it, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B2);
        (*it)->f[2] = computeKey(*it, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B3);
        (*it)->f[3] = computeKey(*it, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B4);
        (*it)->f[4] = computeKey(*it, sbpl::motion::GroupType::ARM,sbpl::motion::BaseGroupHeuristic::NONE);
        //(*it)->f = f[std::distance(f.begin(), std::max_element(f.begin(),f.end()))];
    }
    m_open.make(sbpl::motion::BaseGroupHeuristic::B1);
    m_open.make(sbpl::motion::BaseGroupHeuristic::B2);
    m_open.make(sbpl::motion::BaseGroupHeuristic::B3);
    m_open.make(sbpl::motion::BaseGroupHeuristic::B4);
    m_open.make(sbpl::motion::GroupType::ARM);
}

int MARAStar::computeKey(SearchState* s, sbpl::motion::GroupType group, sbpl::motion::BaseGroupHeuristic base_heur) const
{   
   
   if(group == sbpl::motion::GroupType::ARM)
   {
        //SMPL_ERROR_STREAM("ARM Compute Key g is "<<s->g<<" and h is "<<s->h[group]);
        return s->g + (unsigned int)(m_curr_eps * s->h[group]);
   }
    else
    {
        //SMPL_ERROR_STREAM("AUV "<<base_heur<<" Compute Key g is "<<s->g<<" and h is "<<s->h[base_heur]);
        return s->g + (unsigned int)(m_curr_eps * s->h[base_heur]);
    }
    //return s->g + (unsigned int)(m_curr_eps * s->h[0]);//std::distance(std::begin(s->h), std::min_element(std::begin(s->h),std::end(s->h))));
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
MARAStar::SearchState* MARAStar::getSearchState(int state_id)
{
    if (m_states.size() <= state_id) {
        m_states.resize(state_id + 1, nullptr);
    }

    auto& state = m_states[state_id];
    if (state == NULL) {
        state = createState(state_id);
    }

    return state;
}

// Create a new search state for a graph state.
MARAStar::SearchState* MARAStar::createState(int state_id)
{
    assert(state_id < m_states.size());

    SearchState* ss = new SearchState;
    ss->state_id = state_id;
    ss->call_number = 0;
    return ss;
}

// Lazily (re)initialize a search state.
void MARAStar::reinitSearchState(SearchState* state)
{
    if (state->call_number != m_call_number) {
        SMPL_DEBUG_NAMED(SELOG, "Reinitialize state %d", state->state_id);
        state->g = INFINITECOST;
        state->h[0] = m_heur->GetGoalHeuristic(state->state_id);
        state->h[4] = state->h[0];
        state->h[3] = state->h[0];
        state->h[2] = state->h[0];
        state->h[1] = state->h[0];
        SMPL_WARN_STREAM("cost retruned is "<<state->h[0]);
        state->f[0] = state->f[1] = state->f[2] = state->f[3] = state->f[4] = INFINITECOST;
        state->eg = INFINITECOST;
        state->iteration_closed[0] = 0;
        state->iteration_closed[1] = 0;
        state->iteration_closed[2] = 0;
        state->iteration_closed[3] = 0;
        state->iteration_closed[4] = 0;
        state->call_number = m_call_number;
        state->bp = nullptr;
        state->incons = false;
    }
}

void MARAStar::reinitSearchState(SearchState* state, sbpl::motion::GroupType planning_group, sbpl::motion::BaseGroupHeuristic base_heur)
{
    if (state->call_number != m_call_number) {
        SMPL_WARN_NAMED(SELOG, "Reinitialize state %d", state->state_id);
        state->g = INFINITECOST;
        /*std::vector<double> f(5);
        f[0] = m_heur->GetGoalHeuristic(state->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B1);
        f[1] = m_heur->GetGoalHeuristic(state->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B2);
        f[2] = m_heur->GetGoalHeuristic(state->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B3);
        f[3] = m_heur->GetGoalHeuristic(state->state_id, sbpl::motion::GroupType::BASE,sbpl::motion::BaseGroupHeuristic::B4);
        f[4] = m_heur->GetGoalHeuristic(state->state_id, sbpl::motion::GroupType::ARM,sbpl::motion::BaseGroupHeuristic::NONE);
        int index = std::distance(f.begin(), std::max_element(f.begin(),f.end()));
        SMPL_WARN_STREAM("The group with min heuristic is "<<index<<" its value "<<f[index]);
        state->h[0] = f[index];
         state->h[4] =  state->h[3] =  state->h[2] =  state->h[1] =  state->h[0] ; */

        if(planning_group == sbpl::motion::GroupType::ARM)
        {
            state->h[planning_group] = m_heur->GetGoalHeuristic(state->state_id, planning_group, base_heur);
            SMPL_WARN_STREAM("cost retruned of group "<<planning_group<<" and heur "<<base_heur<<"  is "<<state->h[planning_group]);
        }
        else
        {
            state->h[base_heur] = m_heur->GetGoalHeuristic(state->state_id, planning_group, base_heur);
            SMPL_WARN_STREAM("cost retruned of group "<<planning_group<<" and heur "<<base_heur<<" is "<<state->h[base_heur]);
        }
       
        state->f[0] = INFINITECOST;
        state->f[0] = state->f[1] = state->f[2] = state->f[3] = state->f[4] = INFINITECOST;
        state->eg = INFINITECOST;
        state->iteration_closed[0] = 0;
        state->iteration_closed[1] = 0;
        state->iteration_closed[2] = 0;
        state->iteration_closed[3] = 0;
        state->iteration_closed[4] = 0;
        state->call_number = m_call_number;
        state->bp = nullptr;
        state->incons = false;
    }
}

// Extract the path from the start state up to a new state.
void MARAStar::extractPath(
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
