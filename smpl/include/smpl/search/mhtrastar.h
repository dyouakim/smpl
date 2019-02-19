#ifndef SBPL_MHTRASTAR_H
#define SBPL_MHTRASTAR_H

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
// control of EPS

// initial suboptimality bound (cost solution <= cost(eps*cost optimal solution)
#define TRA_DEFAULT_INITIAL_EPS     10.0
// as planning time exist, AD* decreases epsilon bound
#define TRA_DECREASE_EPS            0.2
// final epsilon bound
#define TRA_FINAL_EPS               1.0

#define TRA_INCONS_LIST_ID 0
#define TRAMDP_STATEID2IND ARAMDP_STATEID2IND



struct MHTRAState : public multi_index_heap_element
{
    int state_id;       // corresponding graph state

    /// \brief TRA* relevant data
    unsigned int E; // expansion time
    unsigned int C; // creation time -- first put on open list
    std::vector<int> parent_hist; // history of parents
    int source;
    // history of g-values associated with each of the parents in parent_hist
    std::vector<unsigned int> gval_hist;

    unsigned int v;
    unsigned int g;     // cost-to-come
    unsigned int h[2];     // estimated cost-to-go
    unsigned int f[2];     // (g + eps * h) at time of insertion into OPEN
    unsigned int eg;    // g-value at time of expansion
            
    unsigned short iteration_closed[2];
    short unsigned int call_number;
    short unsigned int numofexpands;

    /// \brief best predecessor and the action from it, used only in forward searches
    MHTRAState *bestpredstate;

    /// \brief the next state if executing best action
    MHTRAState  *bestnextstate;
    unsigned int costtobestnextstate;
    bool incons;

    unsigned int firstExpansionStep;
    std::vector<bool> to_erase_parents;
    MHTRAState() {};
    ~MHTRAState() {};

    int getSize()
    {
        return (int) (
                sizeof(MHTRAState) +
                sizeof(MHTRAState*) * parent_hist.size() +    // parent_hist
                sizeof(unsigned int) * gval_hist.size() );  // gval_hist
    }
};


struct SearchStateCompareBase
    {
        bool operator()(const MHTRAState& s1, const MHTRAState& s2) const {
            return s1.f[0] < s2.f[0];
        }
    };

    struct SearchStateCompareArm
    {
        bool operator()(const MHTRAState& s1, const MHTRAState& s2) const {
            return s1.f[1] < s2.f[1];
        }
    };


class MHTRAStar : public SBPLPlanner
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


    MHTRAStar(DiscreteSpaceInformation* space, Heuristic* heuristic, bool bForwardSearch);

    ~MHTRAStar();

    void allowPartialSolutions(bool enabled) { m_allow_partial_solutions = enabled; }
    bool allowPartialSolutions() const { return m_allow_partial_solutions; }

    void setAllowedRepairTime(double allowed_time_secs)
    { m_time_params.max_allowed_time = to_duration(allowed_time_secs); }

    double allowedRepairTime() const
    { return to_seconds(m_time_params.max_allowed_time); }

    void costs_changed();
    
    int replan( const TimeParameters &params, std::vector<int>* solution, int* cost);
    /// \name Required Functions from SBPLPlanner
    ///@{
    int replan(double allowed_time_secs, std::vector<int>* solution) override;
    int replan(double allowed_time_secs, std::vector<int>* solution, int* solcost) override;
    int set_goal(int state_id) override;
    int set_start(int state_id) override;
    int set_multiple_start(std::vector<int> start_statesID) override;
    int force_planning_from_scratch() override;
    int set_search_mode(bool bSearchUntilFirstSolution) override;
    void costs_changed(const StateChangeQuery& stateChange) override;
    ///@}

    /// \name Reimplemented Functions from SBPLPlanner
    ///@{
    int replan(std::vector<int>* solution, ReplanParams params) override;
    int replan(std::vector<int>* solution, ReplanParams params, int* solcost) override;
    int force_planning_from_scratch_and_free_memory() override;
    double get_solution_eps() const { return m_satisfied_eps; }
    int get_n_expands() const  { return m_expand_count; }
    double get_initial_eps() {return m_initial_eps;}
    double get_initial_eps_planning_time() {  return to_seconds(m_search_time_init); }
    double get_final_eps_planning_time() { return to_seconds(m_search_time); }
    int get_n_expands_init_solution() {  return m_expand_count_init; }
    double get_final_epsilon() { return m_final_eps;}
    void get_search_stats(std::vector<PlannerStats>* s) override;
    void set_initialsolution_eps(double initialsolution_eps) {m_initial_eps = initialsolution_eps;}
    ///@}

  
    /// \brief direct form of informing the search about the new edge costs
    /// \param succsIDV array of successors of changed edges
    /// \note this is used when the search is run forwards
    void update_succs_of_changededges(std::vector<int>* succsIDV);


    //TODO: implement!!!!!!!!!!!
    /// \brief direct form of informing the search about the new edge costs
    /// \param predsIDV array of predecessors of changed edges
    /// \note this is used when the search is run backwards
    void update_preds_of_changededges(std::vector<int>* predsIDV);

   void heuristicChanged();

   void updateHeuristic(Heuristic* heuristic)
   {
    m_heur = heuristic;
   }

   void updateSpace(DiscreteSpaceInformation* space)
   {
        m_space = space;
   }

   void initializeStartStates();

private:


    DiscreteSpaceInformation* m_space;
    Heuristic* m_heur;
    MHTRAState* start_state;
    MHTRAState* goal_state;

    TimeParameters m_time_params;

    double m_initial_eps;
    double m_final_eps;
    double m_delta_eps;
    double m_satisfied_eps;

    double initial_eps_planning_time;
    double final_eps_planning_time;
    

    bool m_allow_partial_solutions;
    bool bforwardsearch;
    // if true, then search until first solution (see planner.h for search
    // modes)
    bool bsearchuntilfirstsolution;

    std::vector<MHTRAState*> m_states;

    int m_start_state_id;   // graph state id for the start state
    int m_goal_state_id;    // graph state id for the goal state

    std::vector<int> m_start_states_ids;

    // map from graph state id to search state id, incrementally expanded
    // as states are encountered during the search
    std::vector<int> m_graph_to_search_map;

    // search state (not including the values of g, f, back pointers, and
    // closed list from m_stats)
    //intrusive_heap<MHTRAState, SearchStateCompare> m_open;

    multi_index_intrusive_heap<MHTRAState, SearchStateCompareBase> m_open_base;
    multi_index_intrusive_heap<MHTRAState, SearchStateCompareBase> m_open_base_iso;
    multi_index_intrusive_heap<MHTRAState, SearchStateCompareArm> m_open_arm;


    std::vector<MHTRAState*> m_incons;
    double m_curr_eps;
    int m_iteration;

    std::vector<int> m_clearance_cells;
    
    int m_call_number;          // for lazy reinitialization of search states
    int m_last_start_state_id;  // for lazy reinitialization of the search tree
    int m_last_goal_state_id;   // for updating the search tree when the goal changes
    double m_last_eps;          // for updating the search tree when heuristics change

    int m_expand_count_init;
    clock::duration m_search_time_init;
    int m_expand_count;
    clock::duration m_search_time;
    int MaxMemoryCounter;
    FILE *fDeb;

    unsigned int expansion_step;

    bool bReevaluatefvals;
    bool bReinitializeSearchStateSpace;
    bool bRebuildOpenList;

    bool bNewSearchIteration;

    std::vector<MHTRAState*> seen_states;


    bool costChanged;
    int restore_step;
    int last_expanded_state_id;

    void convertTimeParamsToReplanParams(const TimeParameters& t,ReplanParams& r) const;
    void convertReplanParamsToTimeParams(const ReplanParams& r, TimeParameters& t);

    bool timedOut(int elapsed_expansions, const clock::duration& elapsed_time) const;

    int improvePath(const clock::time_point& start_time, MHTRAState* goal_state, int& elapsed_expansions, clock::duration& elapsed_time);

    void expand(MHTRAState* s, sbpl::motion::GroupType group);

    void recomputeHeuristics();
    void reorderOpen();
    int computeKey(MHTRAState* s, sbpl::motion::GroupType group) const;

    MHTRAState* getSearchState(int state_id);
    MHTRAState* createState(int state_id);
    void reinitSearchState(MHTRAState* state);

    void extractPath(MHTRAState* to_state, std::vector<int>& solution, int& cost) const;

    bool RestoreSearchTree(int restoreStep);
    bool RestorePerGroup(MHTRAState* current, int restoreStep, int group, std::vector<MHTRAState*>& current_seen, bool& addedToSeen);

    bool updateParents(MHTRAState* state, unsigned int expansionStep, int& latestParenIdx, unsigned int& latestGVal, int group);

    bool storeParent(MHTRAState* succ_state, MHTRAState* state, unsigned int gVal, unsigned int expansionStep);


    unsigned int GetStateCreationTime(int stateID)
    {
        MHTRAState* state = getSearchState(stateID);
        return state->C;
    };

    // initialization of the start state
    void InitializeSearch();

    void Recomputegval(MHTRAState* state);

    // used for backward search
    void UpdatePreds(MHTRAState* state);

    //used for forward search
    void UpdateSuccs(MHTRAState* state);
    
    void BuildNewOPENList();

    // debugging
    void PrintSearchState(MHTRAState* state, FILE* fOut);


    int getParentStateIdByExpansionStep();


};

} //sbpl
#endif
