#ifndef SBPL_ADAPTIVE_TRAStar_H
#define SBPL_ADAPTIVE_TRAStar_H

// standard includes
#include <assert.h>
#include <stdio.h>
#include <time.h>
#include <vector>
#include <algorithm> 

// system includes
#include <sbpl/heuristics/heuristic.h>
#include <sbpl/planners/planner.h>
#include <smpl/intrusive_heap.h>
#include <smpl/time.h>
#include <smpl/search/arastar.h>


// project includes
//#include <sbpl_adaptive/common.h>
//#include <sbpl_adaptive/core/search/adaptive_planner.h>

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

struct TRAState : public heap_element
{
    int state_id;       // corresponding graph state

    /// \brief TRA* relevant data
    unsigned int E; // expansion time
    unsigned int C; // creation time -- first put on open list
    std::vector<TRAState*> parent_hist; // history of parents

    // history of g-values associated with each of the parents in parent_hist
    std::vector<unsigned int> gval_hist;

    unsigned int v;
    unsigned int g;     // cost-to-come
    unsigned int h;     // estimated cost-to-go
    unsigned int f;     // (g + eps * h) at time of insertion into OPEN
    unsigned int eg;    // g-value at time of expansion
            
    short unsigned int iteration_closed;
    short unsigned int call_number;
    short unsigned int numofexpands;

    /// \brief best predecessor and the action from it, used only in forward searches
    TRAState *bestpredstate;

    /// \brief the next state if executing best action
    TRAState  *bestnextstate;
    unsigned int costtobestnextstate;
    bool incons;

    unsigned int firstExpansionStep;
    TRAState() {};
    ~TRAState() {};

    int getSize()
    {
        return (int) (
                sizeof(TRAState) +
                sizeof(TRAState*) * parent_hist.size() +    // parent_hist
                sizeof(unsigned int) * gval_hist.size() );  // gval_hist
    }
};

struct SearchStateCompare
    {
        bool operator()(const TRAState& s1, const TRAState& s2) const {
            return s1.f < s2.f;
        }
    };


class TRAStar : public SBPLPlanner
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


    TRAStar(DiscreteSpaceInformation* space, Heuristic* heuristic, bool bForwardSearch);

    ~TRAStar();

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
private:


    DiscreteSpaceInformation* m_space;
    Heuristic* m_heur;

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

    std::vector<TRAState*> m_states;

    TRAState* goal_state;
    TRAState* start_state;

    int m_start_state_id;   // graph state id for the start state
    int m_goal_state_id;    // graph state id for the goal state

    // map from graph state id to search state id, incrementally expanded
    // as states are encountered during the search
    std::vector<int> m_graph_to_search_map;

    // search state (not including the values of g, f, back pointers, and
    // closed list from m_stats)
    intrusive_heap<TRAState, SearchStateCompare> m_open;
    std::vector<TRAState*> m_incons;
    double m_curr_eps;
    int m_iteration;

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

    std::vector<TRAState*> seen_states;

    bool costChanged;
    int restore_step;

    void convertTimeParamsToReplanParams(const TimeParameters& t,ReplanParams& r) const;
    void convertReplanParamsToTimeParams(const ReplanParams& r, TimeParameters& t);

    bool timedOut(int elapsed_expansions, const clock::duration& elapsed_time) const;

    int improvePath(const clock::time_point& start_time, TRAState* goal_state, int& elapsed_expansions, clock::duration& elapsed_time);

    void expand(TRAState* s);

    void recomputeHeuristics();
    void reorderOpen();
    int computeKey(TRAState* s) const;

    TRAState* getSearchState(int state_id);
    TRAState* createState(int state_id);
    void reinitSearchState(TRAState* state);

    void extractPath(TRAState* to_state, std::vector<int>& solution, int& cost) const;

    bool RestoreSearchTree(int restoreStep);

    bool updateParents(TRAState* state, unsigned int expansionStep, TRAState* latestParent, unsigned int *latestGVal);

    bool storeParent(TRAState* succ_state, TRAState* state, unsigned int gVal, unsigned int expansionStep);


    unsigned int GetStateCreationTime(int stateID)
    {
        TRAState* state = getSearchState(stateID);
        return state->C;
    };

    // initialization of the start state
    void InitializeSearch();

    void Recomputegval(TRAState* state);

    // used for backward search
    void UpdatePreds(TRAState* state);

    //used for forward search
    void UpdateSuccs(TRAState* state);
    
    void BuildNewOPENList();

    // debugging
    void PrintSearchState(TRAState* state, FILE* fOut);

};

} //sbpl
#endif
