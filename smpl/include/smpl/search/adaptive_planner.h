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

#ifndef SMPL_ADAPTIVE_PLANNER_H
#define SMPL_ADAPTIVE_PLANNER_H

// standard includes
#include <random>

// system includes
#include <sbpl/planners/planner.h>

// project includes
#include <smpl/graph/adaptive_graph_extension.h>
#include <smpl/graph/robot_planning_space.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/search/arastar.h>

namespace sbpl {
namespace motion {

class AdaptivePlanner : public SBPLPlanner
{
public:

    struct TimeParameters
    {
        enum TimingType { EXPANSIONS, TIME };

        ARAStar::TimeParameters planning;
        ARAStar::TimeParameters tracking;
    };

    AdaptivePlanner(RobotPlanningSpace* space, RobotHeuristic* heur);

    ~AdaptivePlanner();

    void set_time_parameters(const TimeParameters& params) { m_time_params = params; }
    void set_plan_eps(double eps) { m_eps_plan = eps; }
    void set_track_eps(double eps) { m_eps_track = eps; }
    double get_plan_eps() const { return m_eps_plan; }
    double get_track_eps() const { return m_eps_track; }

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
    int set_search_mode(bool first_solution_unbounded) override;
    void costs_changed(const StateChangeQuery& state_change) override;
    ///@}

private:

    ARAStar m_planner;
    ARAStar m_tracker;

    AdaptiveGraphExtension* m_adaptive_graph;

    TimeParameters m_time_params;

    std::default_random_engine m_rng;

    int m_start_state_id;
    int m_goal_state_id;

    double m_eps_plan;
    double m_eps_track;
};

} // namespace motion
} // namespace sbpl

#endif
