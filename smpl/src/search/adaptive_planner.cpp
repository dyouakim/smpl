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

#include <smpl/search/adaptive_planner.h>

// standard includes
#include <chrono>

// project includes
#include <smpl/time.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>

namespace sbpl {
namespace motion {

AdaptivePlanner::AdaptivePlanner(
    RobotPlanningSpace* space,
    RobotHeuristic* heur)
:
    m_planner(space, heur),
    m_tracker(space, heur),
    m_adaptive_graph(nullptr),
    m_start_state_id(-1),
    m_goal_state_id(-1),
    m_eps_plan(1.0),
    m_eps_track(1.0)
{
    m_adaptive_graph = space->getExtension<AdaptiveGraphExtension>();
    if (!m_adaptive_graph) {
        SMPL_WARN("Adaptive Planner recommends Adaptive Graph Extension");
    }
}

AdaptivePlanner::~AdaptivePlanner()
{

}

int AdaptivePlanner::replan(std::vector<int>* solution, ReplanParams params)
{
    int cost;
    return replan(solution, params, &cost);
}

int AdaptivePlanner::replan(
    std::vector<int>* solution,
    ReplanParams params,
    int* cost)
{
    return replan(params.max_time, solution, cost);
}

int AdaptivePlanner::force_planning_from_scratch_and_free_memory()
{
    return 0;
}

double AdaptivePlanner::get_solution_eps() const
{
    return 0.0;
}

int AdaptivePlanner::get_n_expands() const
{
    return 0;
}

double AdaptivePlanner::get_initial_eps()
{
    return 0.0;
}

double AdaptivePlanner::get_initial_eps_planning_time()
{
    return 0.0;
}

double AdaptivePlanner::get_final_eps_planning_time()
{
    return 0.0;
}

int AdaptivePlanner::get_n_expands_init_solution()
{
    return 0;
}

double AdaptivePlanner::get_final_epsilon()
{
    return 0.0;
}

void AdaptivePlanner::get_search_stats(std::vector<PlannerStats>* s)
{

}

void AdaptivePlanner::set_initialsolution_eps(double initialsolution_eps)
{
    m_eps_plan = initialsolution_eps; //std::sqrt(initialsolution_eps);
    m_eps_track = initialsolution_eps; //std::sqrt(initialsolution_eps);
}

int AdaptivePlanner::replan(double allowed_time, std::vector<int>* solution)
{
    int cost;
    return replan(allowed_time, solution, &cost);
}

enum ReplanResultCode
{
    SUCCESS = 0,
    NO_START_OR_GOAL_SET,
    FAILED_TO_SET_START,
    FAILED_TO_SET_GOAL,
    FAILED_TO_FIND_PATH,
    TIMED_OUT,

    REPLAN_RESULT_CODE_COUNT
};

int AdaptivePlanner::replan(
    double allowed_time,
    std::vector<int>* solution,
    int* cost)
{
    if (m_start_state_id == -1 || m_goal_state_id == -1) {
        SMPL_ERROR("Start or goal state is not set");
        return !NO_START_OR_GOAL_SET;
    }

    if (!m_adaptive_graph) {
        m_planner.allowPartialSolutions(false);
        m_planner.set_start(m_start_state_id);
        m_planner.set_goal(m_goal_state_id);
        return m_planner.replan(allowed_time, solution, cost);
    }

    m_tracker.allowPartialSolutions(true);

    m_adaptive_graph->addHighDimRegion(m_start_state_id);
    m_adaptive_graph->addHighDimRegion(m_goal_state_id);

    // search G^ad for a least-cost path
    if (!m_planner.set_start(m_start_state_id) ||
        !m_tracker.set_start(m_start_state_id))
    {
        SMPL_ERROR("Failed to set start state");
        return !FAILED_TO_SET_START;
    }
    if (!m_planner.set_goal(m_goal_state_id) ||
        !m_tracker.set_goal(m_goal_state_id))
    {
        SMPL_ERROR("Failed to set goal state");
        return !FAILED_TO_SET_GOAL;
    }

    m_planner.set_initialsolution_eps(m_eps_plan);
    m_tracker.set_initialsolution_eps(m_eps_track);

    double time_remaining = allowed_time;

    std::vector<int> plan_path;
    std::vector<int> track_path;
    bool done =  false;
    int iter_count = 0;
    int res;
    while (!done) {
        ++iter_count;

        plan_path.clear();
        int plan_cost = -1;
        auto plan_start = clock::now();
        m_adaptive_graph->setPlanMode();
        m_planner.force_planning_from_scratch();
        SMPL_INFO("Time remaining: %0.3fs. Plan low-dimensional path", time_remaining);
        ARAStar::TimeParameters time_params = m_time_params.planning;
        time_params.max_allowed_time_init = to_duration(time_remaining);
        time_params.max_allowed_time = std::min(time_params.max_allowed_time, to_duration(time_remaining));
        res = m_planner.replan(time_params, &plan_path, &plan_cost);
        auto plan_finish = clock::now();

        SMPL_INFO("Planner terminated");
        SMPL_INFO_STREAM("  Path: " << plan_path);
        SMPL_INFO("  Cost: %d", plan_cost);
        SMPL_INFO("  Time: %0.3f", m_planner.get_final_eps_planning_time());
        SMPL_INFO("  Expansions: %d", m_planner.get_n_expands());
        SMPL_INFO("  Suboptimality Bound: %0.3f", m_planner.get_solution_eps());

        if (!res) {
            SMPL_WARN("Failed to find least-cost path in G^ad");
            return !FAILED_TO_FIND_PATH;
        }

        if (m_adaptive_graph->isExecutable(plan_path)) {
            SMPL_INFO("Found path during planning on iteration %d", iter_count);
            *solution = std::move(plan_path);
            *cost = plan_cost;
            return !SUCCESS;
        }

        time_remaining -= to_seconds(plan_finish - plan_start);
        time_remaining = std::max(0.0, time_remaining);

        if (time_remaining == 0.0) {
            SMPL_WARN("No time to track!");
            return !TIMED_OUT;
        }

        track_path.clear();
        int track_cost = -1;
        auto track_start = clock::now();
        m_adaptive_graph->setTrackMode(plan_path);
        m_tracker.force_planning_from_scratch();
        SMPL_INFO("Time remaining: %0.3fs. Track low-dimensional path", time_remaining);
        time_params = m_time_params.tracking;
        time_params.max_allowed_time_init = to_duration(time_remaining);
        time_params.max_allowed_time = std::min(time_params.max_allowed_time, to_duration(time_remaining));
        res = m_tracker.replan(time_params, &track_path, &track_cost);
        auto track_finish = clock::now();

        SMPL_INFO("Tracker terminated");
        SMPL_INFO_STREAM("  Path: " << track_path);
        SMPL_INFO("  Cost: %d", track_cost);
        SMPL_INFO("  Time: %0.3f", m_tracker.get_final_eps_planning_time());
        SMPL_INFO("  Expansions: %d", m_tracker.get_n_expands());
        SMPL_INFO("  Suboptimality Bound: %0.3f", m_tracker.get_solution_eps());

        time_remaining -= to_seconds(plan_finish - plan_start);
        time_remaining = std::max(0.0, time_remaining);

        auto select_random_path_state = [this](const std::vector<int>& path)
        {
            std::uniform_int_distribution<int> dist(0, path.size() - 1);
            int ridx = dist(m_rng);
            return path[ridx];
        };

        if (res) { // tracker found a (partial) solution
            bool finished =
                    !track_path.empty() && track_path.back() == m_goal_state_id;

            if (finished) {
                if (track_cost <= m_eps_track * plan_cost) {
                    SMPL_INFO("Solution accepted (%d <= %0.3f * %d)", track_cost, m_eps_track, plan_cost);
                    // solution quality is acceptable
                    done = true;
                    *solution = track_path;
                    *cost = track_cost;
                    return !SUCCESS;
                } else {
                    SMPL_INFO("Solution rejected (%d > %0.3f * %d)", track_cost, m_eps_track, plan_cost);
                    if (time_remaining == 0.0) {
                        SMPL_WARN("Ran out of time after tracking phase");
                        return !TIMED_OUT;
                    }
                    // tracker solution quality is poor: identify the largest
                    // cost discrepancy between the adaptive path and the
                    // returned path and introduce a sphere there
                    int worst_state_id = select_random_path_state(plan_path);
                    m_adaptive_graph->addHighDimRegion(worst_state_id);
                }
            } else {
                if (time_remaining == 0.0) {
                    SMPL_WARN("Ran out of time after tracking phase");
                    return !TIMED_OUT;
                }
                SMPL_INFO("Add/Grow sphere at (terminal) state %d", track_path.back());
                m_adaptive_graph->addHighDimRegion(track_path.back());
            }
        } else {
            if (time_remaining == 0.0) {
                SMPL_WARN("Ran out of time after tracking phase");
                return !TIMED_OUT;
            }
            // tracker failed to find a solution
            int best_state_id = select_random_path_state(plan_path);
            SMPL_INFO("Add/Grow sphere at (random) state %d", best_state_id);
            m_adaptive_graph->addHighDimRegion(best_state_id);
        }
    }

    SMPL_INFO("Failed to find solution");
    return !TIMED_OUT;
}

int AdaptivePlanner::set_goal(int goal_state_id)
{
    m_goal_state_id = goal_state_id;
    return 1;
}

int AdaptivePlanner::set_start(int start_state_id)
{
    m_start_state_id = start_state_id;
    return 1;
}

int AdaptivePlanner::force_planning_from_scratch()
{
    m_planner.force_planning_from_scratch();
    m_tracker.force_planning_from_scratch();
    return 0;
}

int AdaptivePlanner::set_search_mode(bool first_solution_unbounded)
{
    return 0;
}

void AdaptivePlanner::costs_changed(const StateChangeQuery& state_change)
{

}

} // namespace motion
} // namespace sbpl
