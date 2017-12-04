////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2009, Benjamin Cohen, Andrew Dornbush
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

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#ifndef SMPL_PLANNER_INTERFACE_H
#define SMPL_PLANNER_INTERFACE_H

// standard includes
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <numeric> 
// system includes
#include <Eigen/Dense>
#include <std_msgs/Int8.h>
#include <boost/thread.hpp>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <sbpl/headers.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <smpl/collision_checker.h>
#include <smpl/forward.h>
#include <smpl/occupancy_grid.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>
#include <smpl/ros/heuristic_allocator.h>
#include <smpl/ros/planner_allocator.h>
#include <smpl/ros/planning_space_allocator.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/types.h>

SBPL_CLASS_FORWARD(SBPLPlanner);

namespace sbpl {
namespace motion {

SBPL_CLASS_FORWARD(PlannerInterface);
class PlannerInterface
{
public:

    PlannerInterface(
        RobotModel* robot,
        CollisionChecker* checker,
        OccupancyGrid* grid);

    ~PlannerInterface();

    bool init(const PlanningParams& params);

    bool solve(
        const moveit_msgs::PlanningScene& planning_scene,
        const moveit_msgs::MotionPlanRequest& req,
        moveit_msgs::MotionPlanResponse& res);

    static
    bool SupportsGoalConstraints(
        const std::vector<moveit_msgs::Constraints>& constraints,
        std::string& why);

    bool canServiceRequest(
        const moveit_msgs::MotionPlanRequest& req,
        moveit_msgs::MotionPlanResponse& res) const;

    const RobotPlanningSpacePtr& space() const { return m_pspace; }
    const SBPLPlannerPtr& search() const { return m_planner; }

    std::pair<
        std::map<std::string, RobotHeuristicPtr>::const_iterator,
        std::map<std::string, RobotHeuristicPtr>::const_iterator>
    heuristics() const
    {
        return std::make_pair(m_heuristics.begin(), m_heuristics.end());
    }

    /// @brief Return planning statistics from the last call to solve.
    ///
    /// Possible keys to statistics include:
    ///     "initial solution planning time"
    ///     "initial epsilon"
    ///     "initial solution expansions"
    ///     "final epsilon planning time"
    ///     "final epsilon"
    ///     "solution epsilon"
    ///     "expansions"
    ///     "solution cost"
    ///
    /// @return The statistics
    std::map<std::string, double> getPlannerStats();

    /// \name Visualization
    ///@{

    visualization_msgs::MarkerArray getBfsWallsVisualization() const;
    visualization_msgs::MarkerArray getBfsValuesVisualization() const;

    visualization_msgs::MarkerArray getCollisionModelTrajectoryVisualization(
        const std::vector<RobotState>& path) const;
    ///@}

    void setStateFromRobotState( const moveit::core::RobotState& robot_state, sbpl::motion::RobotState& state) const;

protected:

    RobotModel* m_robot;
    CollisionChecker* m_checker;
    OccupancyGrid* m_grid;

    ForwardKinematicsInterface* m_fk_iface;

    PlanningParams m_params;

    // params
    bool m_initialized;

    std::map<std::string, PlanningSpaceAllocatorPtr> m_pspace_allocators;
    std::map<std::string, HeuristicAllocatorPtr> m_heuristic_allocators;
    std::map<std::string, PlannerAllocatorPtr> m_planner_allocators;

    // planner components

    RobotPlanningSpacePtr m_pspace;
    std::map<std::string, RobotHeuristicPtr> m_heuristics;
    SBPLPlannerPtr m_planner;

    int m_sol_cost;

    std::string m_planner_id;

    moveit_msgs::MotionPlanRequest m_req;
    moveit_msgs::MotionPlanResponse m_res;

    std::vector<RobotState> lastPlannedPath;

    bool checkConstructionArgs() const;

    // Initialize the SBPL planner and the smpl environment
    bool initializePlannerAndEnvironment();

    bool checkParams(const PlanningParams& params) const;

    // Set start configuration
    bool setStart(const moveit_msgs::RobotState& state);

    // Set goal(s)
    bool setGoalPosition(const moveit_msgs::Constraints& goals);

    // use this to set a 7dof goal!
    bool setGoalConfiguration(const moveit_msgs::Constraints& goal_constraints);

    // Plan a path to a cartesian goal(s)
    bool planToPose(
        const moveit_msgs::MotionPlanRequest& req,
        std::vector<RobotState>& path,std::vector<geometry_msgs::PoseStamped>& eePath,
        moveit_msgs::MotionPlanResponse& res);
    bool planToConfiguration(
        const moveit_msgs::MotionPlanRequest& req,
        std::vector<RobotState>& path,
        moveit_msgs::MotionPlanResponse& res);

    // Retrieve plan from sbpl
    bool plan(double allowed_time, std::vector<RobotState>& path, std::vector<geometry_msgs::PoseStamped>& eePath);

    bool repairPlan(double allowed_time, std::vector<RobotState>& path, std::vector<geometry_msgs::PoseStamped>& eePath, int inavlidTrajPointIdx);

    bool extractGoalPoseFromGoalConstraints(
        const moveit_msgs::Constraints& goal_constraints,
        Eigen::Affine3d& goal_pose_out,
        Eigen::Vector3d& offset) const;

    // extract tolerance as an array of 6 doubles: x, y, z, roll, pitch, yaw
    bool extractGoalToleranceFromGoalConstraints(
        const moveit_msgs::Constraints& goal_constraints,
        double* tolerance_out);

    void clearMotionPlanResponse(
        const moveit_msgs::MotionPlanRequest& req,
        moveit_msgs::MotionPlanResponse& res) const;

    bool parsePlannerID(
        const std::string& planner_id,
        std::string& space_name,
        std::string& heuristic_name,
        std::string& search_name) const;

    void clearGraphStateToPlannerStateMap();
    bool reinitPlanner(const std::string& planner_id);

    bool isPathValid(const std::vector<RobotState>& path) const;
    void postProcessPath(std::vector<RobotState>& path) const;
    void convertJointVariablePathToJointTrajectory(
        const std::vector<RobotState>& path,
        trajectory_msgs::JointTrajectory& traj) const;
    void profilePath(trajectory_msgs::JointTrajectory& traj) const;
    void visualizePath(const std::vector<RobotState>& path) const;

    bool writePath(
        const moveit_msgs::RobotState& ref,
        const moveit_msgs::RobotTrajectory& traj) const;

};

} // namespace motion
} // namespace sbpl

#endif
