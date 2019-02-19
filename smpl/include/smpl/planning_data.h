/* Author: Dina Youakim*/


#ifndef SMPL_PLANNING_DATA_H
#define SMPL_PLANNING_DATA_H

#include <smpl/types.h>


namespace sbpl {
namespace motion {


struct ActionStateInfo 
{
    int state_id;
    int source;
    int expansion_step;
    int parent_id;
    std::vector<double> state_config;
    std::vector<double> parent_state_config;
    std::vector<double> g;
    std::vector<double> h;
    double dist_collision_time;
    double marking_cell_time;
    double dist_obstacles;
    double dist_to_goal;
};

struct SolutionStateInfo
{
    int state_id;
    int source;
    std::vector<double> state_config;
};

struct PlanningData
{
    std::vector<ActionStateInfo> actionStates_;
    double startStatesIKComputeTime_;
    double restorationTime_;
    double searchTime_;
    double pathExtractionTime_;
    double shortcutTime_;
    double interpolateTime_;
    double planningTime_;
    int numExpansions_; 
    int finalEpsilon_;
    int cost_;
    int restorationStep_;
    int pathNumStates_;
    int originalWaypoints_;
    int shortcutWaypoints_;
    int interpolatedWaypoints_;
    std::vector<SolutionStateInfo> solutionStates_;
};

} // namespace motion
} // namespace sbpl

#endif

