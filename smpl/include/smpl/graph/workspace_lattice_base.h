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

#ifndef SMPL_WORKSPACE_LATTICE_BASE_H
#define SMPL_WORKSPACE_LATTICE_BASE_H

// project includes
#include <smpl/graph/robot_planning_space.h>

namespace sbpl {
namespace motion {

/// continuous state ( x, y, z, R, P, Y, j1, ..., jn )
typedef std::vector<double> WorkspaceState;

/// discrete coordinate ( x, y, z, R, P, Y, j1, ..., jn )
typedef std::vector<int> WorkspaceCoord;

/// 6-dof pose ( x, y, z, R, P, Y )
typedef std::vector<double> SixPose;

/// 3-dof position ( x, y, z )
typedef std::vector<double> Position;

/// Base class for graph representations that represent states via a 1:1 mapping
/// from joint space states to states in SE(3) alongside an array of redundant
/// variables used to uniquely determine the state. This class is responsible
/// for handling those transformations and discretization of the resulting
/// SE(3) + <free angle array> space.
class WorkspaceLatticeBase : public RobotPlanningSpace
{
public:

    struct Params
    {
        double res_x;
        double res_y;
        double res_z;

        int R_count;
        int P_count;
        int Y_count;

        std::vector<double> free_angle_res;
    };

    virtual bool init(
        RobotModel* robot,
        CollisionChecker* checker,
        const PlanningParams* pp,
        const Params& params);

    virtual bool initialized() const;

    const std::vector<double>& resolution() const { return m_res; }
    int dofCount() const { return m_dof_count; }

protected:

    ForwardKinematicsInterface* m_fk_iface = nullptr;
    InverseKinematicsInterface* m_ik_iface = nullptr;
    RedundantManipulatorInterface* m_rm_iface = nullptr;

    std::vector<double> m_res;
    std::vector<int> m_val_count;
    int m_dof_count = 0;
    std::vector<std::size_t> m_fangle_indices;

    size_t freeAngleCount() const { return m_fangle_indices.size(); }

    // conversions between robot states, workspace states, and workspace coords
    void stateRobotToWorkspace(const RobotState& state, WorkspaceState& ostate) const;
    void stateRobotToCoord(const RobotState& state, WorkspaceCoord& coord) const;
    bool stateWorkspaceToRobot(const WorkspaceState& state, RobotState& ostate) const;
    void stateWorkspaceToCoord(const WorkspaceState& state, WorkspaceCoord& coord) const;
    bool stateCoordToRobot(const WorkspaceCoord& coord, RobotState& state) const;
    void stateCoordToWorkspace(const WorkspaceCoord& coord, WorkspaceState& state) const;

    bool stateWorkspaceToRobot(
        const WorkspaceState& state, const RobotState& seed, RobotState& ostate) const;

    // TODO: variants of workspace -> robot that don't restrict redundant angles
    // TODO: variants of workspace -> robot that take in a full seed state

    // conversions from discrete coordinates to continuous states
    void posWorkspaceToCoord(const double* wp, int* gp) const;
    void posCoordToWorkspace(const int* gp, double* wp) const;
    void rotWorkspaceToCoord(const double* wr, int* gr) const;
    void rotCoordToWorkspace(const int* gr, double* wr) const;
    void poseWorkspaceToCoord(const double* wp, int* gp) const;
    void poseCoordToWorkspace(const int* gp, double* wp) const;
    void favWorkspaceToCoord(const double* wa, int* ga) const;
    void favCoordToWorkspace(const int* ga, double* wa) const;

    void normalizeEulerAngles(double *wr) const;
};

} // namespace motion
} // namespace sbpl

#endif
