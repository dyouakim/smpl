////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2008, Benjamin Cohen, Andrew Dornbush
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

#ifndef SMPL_MANIP_LATTICE_H
#define SMPL_MANIP_LATTICE_H

// standard includes
#include <time.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// system includes
#include <boost/functional/hash.hpp>

// project includes
#include <smpl/angles.h>
#include <smpl/time.h>
#include <smpl/collision_checker.h>
#include <smpl/occupancy_grid.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>
#include <smpl/types.h>
#include <smpl/graph/robot_planning_space.h>
#include <smpl/graph/action_space.h>
namespace sbpl {
namespace motion {

class RobotHeuristic;

typedef std::vector<int> RobotCoord;

struct ManipLatticeState
{
    RobotCoord coord;   // discrete coordinate
    RobotState state;   // corresponding continuous coordinate
    int source;
};

inline
bool operator==(const ManipLatticeState& a, const ManipLatticeState& b)
{
    return a.coord == b.coord;
}

} // namespace motion
} // namespace sbpl

namespace std {

template <>
struct hash<sbpl::motion::ManipLatticeState>
{
    typedef sbpl::motion::ManipLatticeState argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

} // namespace std

namespace sbpl {
namespace motion {

SBPL_CLASS_FORWARD(ManipLattice);

/// \class Discrete space constructed by expliciting discretizing each joint
class ManipLattice :
    public RobotPlanningSpace,
    public PoseProjectionExtension,
    public ExtractRobotStateExtension
{
public:

    ~ManipLattice();

    bool init(
        RobotModel* robot,
        CollisionChecker* checker,
        const PlanningParams* params,
        const std::vector<double>& resolutions,
        ActionSpace* actions);

    const std::vector<double>& resolutions() const { return m_coord_deltas; }
    ActionSpace* actionSpace() { return m_actions; }
    const ActionSpace* actionSpace() const { return m_actions; }

    RobotState getStartConfiguration() const;

    //void getExpandedStates(std::vector<RobotState>& states) const override;

    void setVisualizationFrameId(const std::string& frame_id);
    const std::string& visualizationFrameId() const;

    RobotState getDiscreteCenter(const RobotState& state) const;

    /// \name Reimplemented Public Functions from RobotPlanningSpace
    ///@{
    void GetLazySuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs,
        std::vector<bool>* true_costs) override;
    int GetTrueCost(int parent_id, int child_id) override;
    ///@}

    /// \name Required Public Functions from ExtractRobotStateExtension
    ///@{
    const RobotState& extractState(int state_id) override;
    ///@}

    /// \name Required Public Functions from PoseProjectionExtension
    ///@{
    bool projectToPose(int state_id, Eigen::Affine3d& pos) override;
    ///@}

    bool projectToBasePoint(int state_id, Eigen::Vector3d& pos);
    /// \name Required Public Functions from RobotPlanningSpace
    ///@{
    bool setStart(const RobotState& state) override;
    bool setMultipleStart(const std::vector<RobotState>& states) override;
    bool setGoal(const GoalConstraint& goal) override;
    int getStartStateID() const override;
    int getGoalStateID() const override;
    std::vector<int> getStartStatesID() const override;
    bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path)//, std::vector<geometry_msgs::PoseStamped>& eePath) 
    override;
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    virtual Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Required Public Functions from DiscreteSpaceInformation
    ///@{
    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;
    void PrintState(int state_id, bool verbose, FILE* fout = nullptr) override;
    void GetPreds(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs) override;
    ///@}

    void GetSuccsByGroup(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs, 
        std::vector<int>* clearance_cells,int group) override;

    void GetSuccsWithExpansion(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs, int expanion_step) override;

    void GetSuccsByGroupAndExpansion(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs,  int group, int expanion_step) override;

    void GetPredsByGroupAndExpansion(int state_id, std::vector<int>* preds, 
    std::vector<int>* costs, std::vector<int>* clearance_cells, int group, int expansion_step, int parent_id) override;

    bool updateMultipleStartStates (std::vector<int>* new_starts, std::vector<double>* new_costs, int restore_step) override;

    sbpl::motion::GroupType switchPlanningGroup(int state_id, double switchThreshold);

    void setMotionPlanRequestType (int request_type);

    void setSelectedStartId (int start_id);

protected:

    /// \name discretization methods
    ///@{
    void coordToState(const RobotCoord& coord, RobotState& state) const;
    void stateToCoord(const RobotState& state, RobotCoord& coord) const;
    ///@}

    ManipLatticeState* getHashEntry(int state_id) const;

    int getHashEntry(const RobotCoord& coord);
    int createHashEntry(const RobotCoord& coord, const RobotState& state);
    int getOrCreateState(const RobotCoord& coord, const RobotState& state);
    int reserveHashEntry();

    bool computePlanningFrameFK(
        const RobotState& state,
        std::vector<double>& pose) const;

    bool computeBaseFrameIK(
        const std::vector<double>& pose,
        RobotState& state) const;


    int cost(
        ManipLatticeState* HashEntry1,
        ManipLatticeState* HashEntry2,
        bool bState2IsGoal) const;

    int cost(
        ManipLatticeState* HashEntry1,
        ManipLatticeState* HashEntry2,
        double actionWeight,
        bool bState2IsGoal) const;

    bool checkAction(const RobotState& state, const Action& action);


    bool checkAction(const RobotState& state, const Action& action, double& distToObst, int& distToObstCells);

    bool isGoal(const RobotState& state);

    auto getStateVisualization(const RobotState& vars, const std::string& ns)
        -> std::vector<visual::Marker>;

    auto getStateVisualizationByGroup(const RobotState& vars, const std::string& ns, int group)
        -> std::vector<visual::Marker>;

    auto makePathVisualization(const std::vector<RobotState>& path, std::vector<int> sourceGroup) 
        -> std::vector<visual::Marker>;

    void displaySelectedGoal (int goalStateID);

private:

    ForwardKinematicsInterface* m_fk_iface = nullptr;
    InverseKinematicsInterface* m_ik_iface = nullptr;
    ActionSpace* m_actions = nullptr;
    double clearance_threshold_;
    double dist_threshold_;
    // cached from robot model
    std::vector<double> m_min_limits;
    std::vector<double> m_max_limits;
    std::vector<bool> m_continuous;
    std::vector<bool> m_bounded;

    std::vector<int> m_coord_vals;
    std::vector<double> m_coord_deltas;

    int m_goal_state_id = -1;
    int m_start_state_id = -1;
    std::vector<int> m_start_states_ids;
    int m_goal_unique_id;
    
    // maps from coords to stateID
    typedef ManipLatticeState StateKey;
    typedef PointerValueHash<StateKey> StateHash;
    typedef PointerValueEqual<StateKey> StateEqual;
    hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;

    // maps from stateID to coords
    std::vector<ManipLatticeState*> m_states;

    bool m_near_goal = false;
    clock::time_point m_t_start;

    std::string m_viz_frame_id;

    bool setGoalPose(const GoalConstraint& goal);
    bool setGoalConfiguration(const GoalConstraint& goal);

    void startNewSearch();

    auto getTargetOffsetPose(const std::vector<double>& tip_pose) const
        -> std::vector<double>;

    double getDistanceToGoal(int state_id);

    /// \name planning
    ///@{
    ///@}
};

} // namespace motion
} // namespace sbpl

#endif
