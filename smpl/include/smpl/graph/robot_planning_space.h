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

#ifndef SMPL_ROBOT_PLANNING_SPACE_H
#define SMPL_ROBOT_PLANNING_SPACE_H

// standard includes
#include <vector>

// system includes
#include <Eigen/Dense>
#include <sbpl/discrete_space_information/environment.h>

// project includes
#include <smpl/collision_checker.h>
#include <smpl/extension.h>
#include <smpl/forward.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>
#include <smpl/types.h>
#include <smpl/graph/robot_planning_space_observer.h>
#include <geometry_msgs/PoseStamped.h>

#include <smpl/planning_data.h>

namespace sbpl {
namespace motion {

SBPL_CLASS_FORWARD(RobotHeuristic);
SBPL_CLASS_FORWARD(RobotPlanningSpace);

class RobotPlanningSpace :
    public DiscreteSpaceInformation,
    public virtual Extension
{
public:

    virtual ~RobotPlanningSpace();

    virtual bool init(
        RobotModel* robot,
        CollisionChecker* checker,
        const PlanningParams* params);

    virtual bool setStart(const RobotState& state);
    virtual bool setMultipleStart(const std::vector<RobotState>& states);

    virtual bool setGoal(const GoalConstraint& goal);

    virtual int getStartStateID() const = 0;
    virtual int getGoalStateID() const = 0;
    virtual std::vector<int> getStartStatesID() const
    {

    }

    virtual bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) = 0;// std::vector<geometry_msgs::PoseStamped>& eePath) = 0;

    virtual bool insertHeuristic(RobotHeuristic* h);
    virtual bool eraseHeuristic(const RobotHeuristic* h);
    virtual bool hasHeuristic(const RobotHeuristic* h);

    RobotModel* robot() { return m_robot; }
    const RobotModel* robot() const { return m_robot; }

    CollisionChecker* collisionChecker() { return m_checker; }
    const CollisionChecker* collisionChecker() const { return m_checker; }

    const PlanningParams* params() const { return m_params; }

    const RobotState& startState() const { return m_start; }
    const GoalConstraint& goal() const { return m_goal; }

    size_t numHeuristics() const;
    RobotHeuristic* heuristic(size_t i);
    const RobotHeuristic* heuristic(size_t i) const;

    void insertObserver(RobotPlanningSpaceObserver* obs);
    void eraseObserver(RobotPlanningSpaceObserver* obs);
    bool hasObserver(RobotPlanningSpaceObserver* obs) const;

    void notifyStartChanged(const RobotState& state);
    void notifyGoalChanged(const GoalConstraint& goal);

    /// \name DiscreteSpaceInformation Interface Overrides
    ///@{
    virtual int GetGoalHeuristic(int state_id) override;
    virtual int GetGoalHeuristic(int stateID, int planning_group, int base_heuristic_idx);


    virtual int GetStartHeuristic(int state_id) override;
    virtual int GetFromToHeuristic(int from_id, int to_id) override;

    virtual void GetLazySuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs,
        std::vector<bool>* true_costs) override;

    virtual int GetTrueCost(int parentID, int childID) override;
    ///@}

    /// \name Restate DiscreteSpaceInformation Interface
    ///@{
    virtual void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override = 0;

    virtual void GetSuccsByGroup(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs, std::vector<int>* clearance_cells, int group) override;

    virtual void GetSuccsByGroupAndExpansion(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs, int group, int expanion_step) override;

    virtual void GetSuccsWithExpansion(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs, int expanion_step) override;

    virtual void GetPredsByGroupAndExpansion(int TargetStateID, std::vector<int>* PredIDV, 
        std::vector<int>* CostV, std::vector<int>* clearance_cells, int group, int expanion_step, int parent_id) override;

    virtual bool updateMultipleStartStates (std::vector<int>* new_starts, std::vector<double>* new_costs, int restore_step) override;

    virtual void displaySelectedGoal (int goalStateID);

    virtual void setMotionPlanRequestType (int request_type)
    {
        
    }
    
    virtual void setSelectedStartId (int start_id)
    {

    }

    virtual void GetPreds(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs) override = 0;

    virtual void PrintState(
        int state_id,
        bool verbose,
        FILE* f = nullptr) override = 0;


    sbpl::motion::PlanningData* getPlanningData()
    {
        return planning_data;
    }

    ///@}

    //virtual void getExpandedStates(std::vector<RobotState>& states) const  = 0;

protected:
    sbpl::motion::PlanningData* planning_data;    
private:

    RobotModel* m_robot             = nullptr;
    CollisionChecker* m_checker     = nullptr;
    const PlanningParams* m_params  = nullptr;

    RobotState m_start;
    GoalConstraint m_goal;

    std::vector<RobotHeuristic*> m_heuristics;

    std::vector<RobotPlanningSpaceObserver*> m_obs;


   

    // Make all attempts to hide the set of useless functions from
    // DiscreteSpaceInformation
    virtual bool InitializeEnv(const char*) final { return false; }
    virtual bool InitializeMDPCfg(MDPConfig*) final { return false; }
    virtual int SizeofCreatedEnv() final { return 0; }
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE*) final { }
    virtual void SetAllPreds(CMDPSTATE*) final { }
    virtual void PrintEnv_Config(FILE*) final { }
};

class PointProjectionExtension : public virtual Extension
{
public:

    virtual ~PointProjectionExtension() { }

    virtual bool projectToPoint(int state_id, Eigen::Vector3d& pos) = 0;

    virtual bool projectToBasePoint(int state_id, Eigen::Vector3d& pos)
    {
        return projectToPoint(state_id, pos);
    }

};

class PoseProjectionExtension : public PointProjectionExtension
{
public:

    virtual ~PoseProjectionExtension() { }

    bool projectToPoint(int state_id, Eigen::Vector3d& pos)
    {
        Eigen::Affine3d pose;
        if (!projectToPose(state_id, pose)) {
            return false;
        }
        pos = pose.translation();
        return true;
    }

    virtual bool projectToPose(int state_id, Eigen::Affine3d& pose) = 0;
};

class ExtractRobotStateExtension : public virtual Extension
{
public:

    virtual ~ExtractRobotStateExtension() { }

    virtual const RobotState& extractState(int state_id) = 0;
};

inline
size_t RobotPlanningSpace::numHeuristics() const
{
    return m_heuristics.size();
}

inline
RobotHeuristic* RobotPlanningSpace::heuristic(size_t i)
{
    if (i >= m_heuristics.size()) {
        return nullptr;
    }
    return m_heuristics[i];
}

inline
const RobotHeuristic* RobotPlanningSpace::heuristic(size_t i) const
{
    if (i >= m_heuristics.size()) {
        return nullptr;
    }
    return m_heuristics[i];
}

} // namespace motion
} // namespace sbpl

#endif
