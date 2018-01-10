
#ifndef SMPL_MMARAStar_H
#define SMPL_MMARAStar_H

// standard includes
#include <assert.h>
#include <algorithm>

// system includes
#include <sbpl/heuristics/heuristic.h>
#include <sbpl/planners/planner.h>

// project includes
#include <smpl/multi_index_intrusive_heap.h>
#include <smpl/time.h>
#include <smpl/types.h>
#include <smpl/graph/manip_lattice.h>

#include <smpl/console/console.h>

namespace sbpl {

class MARAStar : public SBPLPlanner
{
public:

    // parameters for controlling how long the search runs
    struct TimeParameters
    {
        bool bounded;
        bool improve;
        enum TimingType { EXPANSIONS, TIME } type;
        int max_expansions_init;
        int max_expansions;
        clock::duration max_allowed_time_init;
        clock::duration max_allowed_time;
    };

    MARAStar(DiscreteSpaceInformation* space, Heuristic* heuristic);
    ~MARAStar();

    void allowPartialSolutions(bool enabled) {
        m_allow_partial_solutions = enabled;
    }

    bool allowPartialSolutions() const { return m_allow_partial_solutions; }

    void setAllowedRepairTime(double allowed_time_secs) {
        m_time_params.max_allowed_time = to_duration(allowed_time_secs);
    }

    double allowedRepairTime() const {
        return to_seconds(m_time_params.max_allowed_time);
    }

    void setTargetEpsilon(double target_eps) {
        m_final_eps = std::max(target_eps, 1.0);
    }

    double targetEpsilon() const { return m_final_eps; }

    void setSwitchDistance(double dist){
        m_switch_dist = dist;
    }

    double switchDistance() const{ return m_switch_dist; }

    void setDeltaEpsilon(double delta_eps) {
        assert(delta_eps > 0.0);
        m_delta_eps = delta_eps;
    }

    double deltaEpsilon() const { return m_delta_eps; }

    void setImproveSolution(bool improve) {
        m_time_params.improve = improve;
    }

    bool improveSolution() const { return m_time_params.improve; }

    void setBoundExpansions(bool bound) { m_time_params.bounded = bound; }
    bool boundExpansions() const { return m_time_params.bounded; }

    int replan(
        const TimeParameters &params,
        std::vector<int>* solution,
        int* cost);

    /// \name Required Functions from SBPLPlanner
    ///@{
    int replan(double allowed_time_secs, std::vector<int>* solution) override;
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
    int get_n_expands() const override;
    double get_initial_eps() override;
    double get_initial_eps_planning_time() override;
    double get_final_eps_planning_time() override;
    int get_n_expands_init_solution() override;
    double get_final_epsilon() override;
    void get_search_stats(std::vector<PlannerStats>* s) override;
    void set_initialsolution_eps(double eps) override;
    ///@}

private:

    struct SearchState : public multi_index_heap_element
    {
        int state_id;       // corresponding graph state
        unsigned int g;     // cost-to-come
        unsigned int h[2];     // estimated cost-to-go
        unsigned int f[2];     // (g + eps * h) at time of insertion into OPEN
        unsigned int eg;    // g-value at time of expansion
        unsigned short iteration_closed[2];
        unsigned short call_number;
        SearchState* bp; 
        bool incons;
        bool merged;
    };

    struct SearchStateCompareBase
    {
        bool operator()(const SearchState& s1, const SearchState& s2) const {
            return s1.f[0] < s2.f[0];
        }
    };

    struct SearchStateCompareArm
    {
        bool operator()(const SearchState& s1, const SearchState& s2) const {
            return s1.f[1] < s2.f[1];
        }
    };

    /*intrusive_heap<SearchState, SearchStateCompare>* m_open()
    {
        if(current_planning_group==sbpl::motion::GroupType::BASE)
        {   
           // SMPL_ERROR("returning base");
            return &m_open_base;
        }
        else
        {
            //SMPL_ERROR("returning arm");
            return &m_open_arm;
        }
    };*/

    sbpl::motion::ManipLattice* m_space;
    Heuristic* m_heur;

    TimeParameters m_time_params;

    double m_initial_eps;
    double m_final_eps;
    double m_delta_eps;
    double m_switch_dist;

    bool m_allow_partial_solutions;

    std::vector<SearchState*> m_states;

    int m_start_state_id;   // graph state id for the start state
    int m_goal_state_id;    // graph state id for the goal state

    // search state (not including the values of g, f, back pointers, and
    // closed list from m_stats)
    //multi_index_intrusive_heap<SearchState, SearchStateCompare> m_open;
    
    multi_index_intrusive_heap<SearchState, SearchStateCompareBase> m_open_base;
    multi_index_intrusive_heap<SearchState, SearchStateCompareArm> m_open_arm;

    std::vector<SearchState*> m_incons;
    double m_curr_eps;
    int m_iteration;

    std::vector<int> m_succs;
    std::vector<int> m_costs;

    int m_call_number;          // for lazy reinitialization of search states
    int m_last_start_state_id;  // for lazy reinitialization of the search tree
    int m_last_goal_state_id;   // for updating the search tree when the goal changes
    double m_last_eps;          // for updating the search tree when heuristics change

    int m_expand_count_init;
    clock::duration m_search_time_init;
    int m_expand_count;
    clock::duration m_search_time;

    double m_satisfied_eps;

    bool expansion_status[2];
    double min_heuristic_found[2];
    int current_planning_group;

    void convertTimeParamsToReplanParams(
        const TimeParameters& t,
        ReplanParams& r) const;
    void convertReplanParamsToTimeParams(
        const ReplanParams& r,
        TimeParameters& t);

    bool timedOut(
        int elapsed_expansions,
        const clock::duration& elapsed_time) const;

    int improvePath(
        const clock::time_point& start_time,
        SearchState* goal_state,
        int& elapsed_expansions,
        clock::duration& elapsed_time);

    void expand(SearchState* s, sbpl::motion::GroupType group);

    void recomputeHeuristics();
    void reorderOpen();
    int computeKey(SearchState* s, sbpl::motion::GroupType planning_group);

    SearchState* getSearchState(int state_id);
    SearchState* createState(int state_id);
    void reinitSearchState(SearchState* state);

    void extractPath(
        SearchState* to_state,
        std::vector<int>& solution,
        int& cost) const;

    void switchPlanningGroup(double current_min);

    void synchronizeGroupsOpenLists();
};

} // namespace sbpl

#endif
