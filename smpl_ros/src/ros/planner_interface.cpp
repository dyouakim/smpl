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

#include <smpl/ros/planner_interface.h>

// standard includes
#include <assert.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <utility>

// system includes
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/utils.h>
#include <sbpl/planners/mhaplanner.h>
#include <trajectory_msgs/JointTrajectory.h>

// project includes
#include <smpl/angles.h>
#include <smpl/post_processing.h>
#include <smpl/time.h>
#include <smpl/types.h>

#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>

#include <smpl/graph/adaptive_workspace_lattice.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/graph/workspace_lattice.h>
#include <smpl/graph/manip_lattice_egraph.h>

#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/multi_bfs_heuristic.h>
#include <smpl/heuristic/egraph_bfs_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/heuristic/multi_frame_bfs_heuristic.h>
#include <smpl/heuristic/joint_dist_heuristic.h>
#include <smpl/heuristic/generic_egraph_heuristic.h>

#include <smpl/search/adaptive_planner.h>
#include <smpl/search/arastar.h>
#include <smpl/search/trastar.h>
#include <smpl/search/marastar.h>
#include <smpl/search/mhtrastar.h>
#include <smpl/search/experience_graph_planner.h>
#include <leatherman/binvox.h>

namespace sbpl {
namespace motion {

template <class T, class... Args>
auto make_unique(Args&&... args) -> std::unique_ptr<T> {
    return std::unique_ptr<T>(new T(args...));
}

const char* PI_LOGGER = "simple";

struct ManipLatticeActionSpaceParams
{
    std::string mprim_filename;
    bool use_multiple_ik_solutions = false;
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    double xyz_snap_thresh;
    double rpy_snap_thresh;
    double xyzrpy_snap_thresh;
    double short_dist_mprims_thresh;
};

// Lookup parameters for ManipLatticeActionSpace, setting reasonable defaults
// for missing parameters. Return false if any required parameter is not found.
bool GetManipLatticeActionSpaceParams(
    ManipLatticeActionSpaceParams& params,
    const PlanningParams& pp)
{
    if (!pp.getParam("mprim_filename", params.mprim_filename)) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'mprim_filename' not found in planning params");
        return false;
    }

    pp.param("use_multiple_ik_solutions", params.use_multiple_ik_solutions, false);

    pp.param("use_xyz_snap_mprim", params.use_xyz_snap_mprim, false);
    pp.param("use_rpy_snap_mprim", params.use_rpy_snap_mprim, false);
    pp.param("use_xyzrpy_snap_mprim", params.use_xyzrpy_snap_mprim, false);
    pp.param("use_short_dist_mprims", params.use_short_dist_mprims, false);

    pp.param("xyz_snap_dist_thresh", params.xyz_snap_thresh, 0.0);
    pp.param("rpy_snap_dist_thresh", params.rpy_snap_thresh, 0.0);
    pp.param("xyzrpy_snap_dist_thresh", params.xyzrpy_snap_thresh, 0.0);
    pp.param("short_dist_mprims_thresh", params.short_dist_mprims_thresh, 0.0);
    return true;
}

auto MakeManipLattice(
    const OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ////////////////
    // Parameters //
    ////////////////

    std::vector<double> resolutions(robot->jointVariableCount());
    std::string disc_string;
    if (!params->getParam("discretization", disc_string)) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'discretization' not found in planning params");
        return nullptr;
    }

    std::unordered_map<std::string, double> disc;
    std::stringstream ss(disc_string);
    std::string joint;
    double jres;
    while (ss >> joint >> jres) {
        disc.insert(std::make_pair(joint, jres));
    }
    ROS_DEBUG_NAMED(PI_LOGGER, "Parsed discretization for %zu joints", disc.size());

    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        const std::string& vname = robot->getPlanningJoints()[vidx];
        const size_t sidx = vname.find('/');
        if (sidx != std::string::npos) {
            // adjust variable name if a variable of a multi-dof joint
            std::string mdof_vname =
                    vname.substr(0, sidx) + "_" + vname.substr(sidx + 1);
            auto dit = disc.find(mdof_vname);
            if (dit == disc.end()) {
                ROS_ERROR_NAMED(PI_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
                return nullptr;
            }
            resolutions[vidx] = dit->second;
        } else {
            auto dit = disc.find(vname);
            if (dit == disc.end()) {
                ROS_ERROR_NAMED(PI_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
                return nullptr;
            }
            resolutions[vidx] = dit->second;
        }

        ROS_DEBUG_NAMED(PI_LOGGER, "resolution(%s) = %0.3f", vname.c_str(), resolutions[vidx]);
    }

    ManipLatticeActionSpaceParams action_params;
    if (!GetManipLatticeActionSpaceParams(action_params, *params)) {
        return nullptr;
    }

    ////////////////////
    // Initialization //
    ////////////////////

    // helper struct to couple the lifetime of ManipLattice and
    // ManipLatticeActionSpace
    struct SimpleManipLattice : public ManipLattice {
        ManipLatticeActionSpace actions;
    };

    auto space = make_unique<SimpleManipLattice>();

    if (!space->init(robot, checker, params, resolutions, &space->actions)) {
        ROS_ERROR_NAMED(PI_LOGGER, "Failed to initialize Manip Lattice");
        return nullptr;
    }

    if (!space->actions.init(space.get())) {
        ROS_ERROR_NAMED(PI_LOGGER, "Failed to initialize Manip Lattice Action Space");
        return nullptr;
    }

    if (grid) {
        space->setVisualizationFrameId(grid->getReferenceFrame());
    }

    auto& actions = space->actions;
    actions.useMultipleIkSolutions(action_params.use_multiple_ik_solutions);
    actions.useAmp(MotionPrimitive::SNAP_TO_XYZ, action_params.use_xyz_snap_mprim);
    actions.useAmp(MotionPrimitive::SNAP_TO_RPY, action_params.use_rpy_snap_mprim);
    actions.useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, action_params.use_xyzrpy_snap_mprim);
    actions.useAmp(MotionPrimitive::SHORT_DISTANCE, action_params.use_short_dist_mprims);
    actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ, action_params.xyz_snap_thresh);
    actions.ampThresh(MotionPrimitive::SNAP_TO_RPY, action_params.rpy_snap_thresh);
    actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, action_params.xyzrpy_snap_thresh);
    actions.ampThresh(MotionPrimitive::SHORT_DISTANCE, action_params.short_dist_mprims_thresh);

    if (!actions.load(action_params.mprim_filename)) {
        ROS_ERROR("Failed to load actions from file '%s'", action_params.mprim_filename.c_str());
        return nullptr;
    }

    ROS_DEBUG_NAMED(PI_LOGGER, "Action Set:");
    for (auto ait = actions.begin(); ait != actions.end(); ++ait) {
        ROS_DEBUG_NAMED(PI_LOGGER, "  type: %s", to_cstring(ait->type));
        if (ait->type == MotionPrimitive::SNAP_TO_RPY) {
            ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", actions.useAmp(MotionPrimitive::SNAP_TO_RPY) ? "true" : "false");
            ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", actions.ampThresh(MotionPrimitive::SNAP_TO_RPY));
        } else if (ait->type == MotionPrimitive::SNAP_TO_XYZ) {
            ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", actions.useAmp(MotionPrimitive::SNAP_TO_XYZ) ? "true" : "false");
            ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ));
        } else if (ait->type == MotionPrimitive::SNAP_TO_XYZ_RPY) {
            ROS_DEBUG_NAMED(PI_LOGGER, "    enabled: %s", actions.useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY) ? "true" : "false");
            ROS_DEBUG_NAMED(PI_LOGGER, "    thresh: %0.3f", actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY));
        } else if (ait->type == MotionPrimitive::LONG_DISTANCE ||
            ait->type == MotionPrimitive::SHORT_DISTANCE)
        {
            ROS_DEBUG_STREAM_NAMED(PI_LOGGER, "    action: " << ait->action);
        }
    }

    return std::move(space);
}

auto MakeManipLatticeEGraph(
    const OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ////////////////
    // Parameters //
    ////////////////

    std::vector<double> resolutions(robot->jointVariableCount());

    std::string disc_string;
    if (!params->getParam("discretization", disc_string)) {
        ROS_ERROR_NAMED(PI_LOGGER, "Parameter 'discretization' not found in planning params");
        return nullptr;
    }
    std::map<std::string, double> disc;
    std::stringstream ss(disc_string);
    std::string joint;
    double jres;
    while (ss >> joint >> jres) {
        disc.insert(std::make_pair(joint, jres));
    }
    ROS_DEBUG_NAMED(PI_LOGGER, "Parsed discretization for %zu joints", disc.size());

    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        const std::string& vname = robot->getPlanningJoints()[vidx];
        auto dit = disc.find(vname);
        if (dit == disc.end()) {
            ROS_ERROR_NAMED(PI_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
            return nullptr;
        }
        resolutions[vidx] = dit->second;
    }

    ManipLatticeActionSpaceParams action_params;
    if (!GetManipLatticeActionSpaceParams(action_params, *params)) {
        return nullptr; // errors logged within
    }

    ////////////////////
    // Initialization //
    ////////////////////

    // helper struct to couple the lifetime of ManipLatticeEgraph and
    // ManipLatticeActionSpace
    struct SimpleManipLatticeEgraph : public ManipLatticeEgraph {
        ManipLatticeActionSpace actions;
    };

    auto space = make_unique<SimpleManipLatticeEgraph>();

    if (!space->init(robot, checker, params, resolutions, &space->actions)) {
        ROS_ERROR("Failed to initialize Manip Lattice Egraph");
        return nullptr;
    }

    if (!space->actions.init(space.get())) {
        ROS_ERROR("Failed to initialize Manip Lattice Action Space");
        return nullptr;
    }

    auto& actions = space->actions;
    actions.useMultipleIkSolutions(action_params.use_multiple_ik_solutions);
    actions.useAmp(MotionPrimitive::SNAP_TO_XYZ, action_params.use_xyz_snap_mprim);
    actions.useAmp(MotionPrimitive::SNAP_TO_RPY, action_params.use_rpy_snap_mprim);
    actions.useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, action_params.use_xyzrpy_snap_mprim);
    actions.useAmp(MotionPrimitive::SHORT_DISTANCE, action_params.use_short_dist_mprims);
    actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ, action_params.xyz_snap_thresh);
    actions.ampThresh(MotionPrimitive::SNAP_TO_RPY, action_params.rpy_snap_thresh);
    actions.ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, action_params.xyzrpy_snap_thresh);
    actions.ampThresh(MotionPrimitive::SHORT_DISTANCE, action_params.short_dist_mprims_thresh);
    if (!actions.load(action_params.mprim_filename)) {
        ROS_ERROR("Failed to load actions from file '%s'", action_params.mprim_filename.c_str());
        return nullptr;
    }

    std::string egraph_path;
    if (params->getParam("egraph_path", egraph_path)) {
        // warning printed within, allow to fail silently
        (void)space->loadExperienceGraph(egraph_path);
    } else {
        ROS_WARN("No experience graph file parameter");
    }

    return std::move(space);
}

auto MakeWorkspaceLattice(
    const OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ROS_INFO_NAMED(PI_LOGGER, "Initialize Workspace Lattice");

    WorkspaceLatticeBase::Params wsp;
    wsp.res_x = grid->resolution();
    wsp.res_y = grid->resolution();
    wsp.res_z = grid->resolution();
    wsp.R_count = 360;
    wsp.P_count = 180 + 1;
    wsp.Y_count = 360;

    auto* rmi = robot->getExtension<RedundantManipulatorInterface>();
    if (!rmi) {
        ROS_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return nullptr;
    }
    wsp.free_angle_res.resize(rmi->redundantVariableCount(), angles::to_radians(1.0));

    auto space = make_unique<WorkspaceLattice>();
    if (!space->init(robot, checker, params, wsp)) {
        ROS_ERROR("Failed to initialize Workspace Lattice");
        return nullptr;
    }

    space->setVisualizationFrameId(grid->getReferenceFrame());

    return std::move(space);
}

auto MakeAdaptiveWorkspaceLattice(
    const OccupancyGrid* grid,
    RobotModel* robot,
    CollisionChecker* checker,
    PlanningParams* params)
    -> std::unique_ptr<RobotPlanningSpace>
{
    ROS_INFO_NAMED(PI_LOGGER, "Initialize Workspace Lattice");
    WorkspaceLatticeBase::Params wsp;
    wsp.res_x = grid->resolution();
    wsp.res_y = grid->resolution();
    wsp.res_z = grid->resolution();
    wsp.R_count = 36; //360;
    wsp.P_count = 19; //180 + 1;
    wsp.Y_count = 36; //360;

    auto* rmi = robot->getExtension<RedundantManipulatorInterface>();
    if (!rmi) {
        ROS_WARN("Workspace Lattice requires Redundant Manipulator Interface");
        return nullptr;
    }
    wsp.free_angle_res.resize(rmi->redundantVariableCount(), angles::to_radians(1.0));

    auto space = make_unique<AdaptiveWorkspaceLattice>();
    if (!space->init(robot, checker, params, wsp, grid)) {
        ROS_ERROR("Failed to initialize Workspace Lattice");
        return nullptr;
    }

    return std::move(space);
}

auto MakeMultiFrameBFSHeuristic(
    RobotPlanningSpace* space,
    const OccupancyGrid* grid,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<MultiFrameBfsHeuristic>();
    h->setCostPerCell(params.cost_per_cell);
    h->setInflationRadius(params.planning_link_sphere_radius);
    if (!h->init(space, grid)) {
        return nullptr;
    }
    return std::move(h);
}

auto MakeBFSHeuristic(
    RobotPlanningSpace* space,
    const OccupancyGrid* grid,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<BfsHeuristic>();
    h->setCostPerCell(params.cost_per_cell);
    h->setInflationRadius(params.planning_link_sphere_radius);
    if (!h->init(space, grid)) {
        return nullptr;
    }
    return std::move(h);
};

auto MakeMultiBFSHeuristic(
    RobotPlanningSpace* space,
    const OccupancyGrid* grid,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<MultiBfsHeuristic>();
    h->setCostPerCell(params.cost_per_cell);
    h->setInflationRadius(params.planning_link_sphere_radius);
    h->setBaseInflationRadius(params.base_radius);
    if (!h->init(space, grid)) {
        return nullptr;
    }
    return std::move(h);
};

auto MakeEuclidDistHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<EuclidDistHeuristic>();

    if (!h->init(space)) {
        return nullptr;
    }

    double wx, wy, wz, wr;
    params.param("x_coeff", wx, 1.0);
    params.param("y_coeff", wy, 1.0);
    params.param("z_coeff", wz, 1.0);
    params.param("rot_coeff", wr, 1.0);

    h->setWeightX(wx);
    h->setWeightY(wx);
    h->setWeightZ(wx);
    h->setWeightRot(wx);
    return std::move(h);
};

auto MakeJointDistHeuristic(RobotPlanningSpace* space)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<JointDistHeuristic>();
    if (!h->init(space)) {
        return nullptr;
    }
    return std::move(h);
};

auto MakeDijkstraEgraphHeuristic3D(
    RobotPlanningSpace* space,
    const OccupancyGrid* grid,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    auto h = make_unique<DijkstraEgraphHeuristic3D>();

//    h->setCostPerCell(params.cost_per_cell);
    h->setInflationRadius(params.planning_link_sphere_radius);
    if (!h->init(space, grid)) {
        return nullptr;
    }

    return std::move(h);
};

auto MakeJointDistEGraphHeuristic(
    RobotPlanningSpace* space,
    const PlanningParams& params)
    -> std::unique_ptr<RobotHeuristic>
{
    struct JointDistEGraphHeuristic : public GenericEgraphHeuristic {
        JointDistHeuristic jd;
    };

    auto h = make_unique<JointDistEGraphHeuristic>();
    if (!h->init(space, &h->jd)) {
        return nullptr;
    }

    double egw;
    params.param("egraph_epsilon", egw, 1.0);
    h->setWeightEGraph(egw);
    return std::move(h);
};

auto MakeARAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    const bool forward_search = true;
    auto search = make_unique<ARAStar>(space, heuristic);

    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    bool search_mode;
    space->params()->param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);

    bool allow_partial_solutions;
    if (space->params()->getParam("allow_partial_solutions", allow_partial_solutions)) {
        search->allowPartialSolutions(allow_partial_solutions);
    }

    double target_eps;
    if (space->params()->getParam("target_epsilon", target_eps)) {
        search->setTargetEpsilon(target_eps);
    }

    double delta_eps;
    if (space->params()->getParam("delta_epsilon", delta_eps)) {
        search->setDeltaEpsilon(delta_eps);
    }

    bool improve_solution;
    if (space->params()->getParam("improve_solution", improve_solution)) {
        search->setImproveSolution(improve_solution);
    }

    bool bound_expansions;
    if (space->params()->getParam("bound_expansions", bound_expansions)) {
        search->setBoundExpansions(bound_expansions);
    }

    double repair_time;
    if (space->params()->getParam("repair_time", repair_time)) {
        search->setAllowedRepairTime(repair_time);
    }
    return std::move(search);
}


auto MakeTRAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    
    const bool forward_search = true;
    auto search = make_unique<TRAStar>(space, heuristic, forward_search);

    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    bool search_mode;
    space->params()->param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);

    bool allow_partial_solutions;
    if (space->params()->getParam("allow_partial_solutions", allow_partial_solutions)) {
        search->allowPartialSolutions(allow_partial_solutions);
    }

    /*double target_eps;
    if (space->params()->getParam("target_epsilon", target_eps)) {
        search->setTargetEpsilon(target_eps);
    }

    double delta_eps;
    if (space->params()->getParam("delta_epsilon", delta_eps)) {
        search->setDeltaEpsilon(delta_eps);
    }

    bool improve_solution;
    if (space->params()->getParam("improve_solution", improve_solution)) {
        search->setImproveSolution(improve_solution);
    }

    bool bound_expansions;
    if (space->params()->getParam("bound_expansions", bound_expansions)) {
        search->setBoundExpansions(bound_expansions);
    }*/

    double repair_time;
    if (space->params()->getParam("repair_time", repair_time)) {
        search->setAllowedRepairTime(repair_time);
    }

    ROS_ERROR_STREAM("in makeTRA* before return");
    return std::move(search);
}


auto MakeMHTRAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    ROS_ERROR_STREAM("in makeMHTRA* start");
    const bool forward_search = true;
    auto search = make_unique<MHTRAStar>(space, heuristic, forward_search);

    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    bool search_mode;
    space->params()->param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);

    bool allow_partial_solutions;
    if (space->params()->getParam("allow_partial_solutions", allow_partial_solutions)) {
        search->allowPartialSolutions(allow_partial_solutions);
    }

    double repair_time;
    if (space->params()->getParam("repair_time", repair_time)) {
        search->setAllowedRepairTime(repair_time);
    }
    return std::move(search);
}

auto MakeMARAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    const bool forward_search = true;
    auto search = make_unique<MARAStar>(space, heuristic);

    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    bool search_mode;
    space->params()->param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);

    bool allow_partial_solutions;
    if (space->params()->getParam("allow_partial_solutions", allow_partial_solutions)) {
        search->allowPartialSolutions(allow_partial_solutions);
    }

    double target_eps;
    if (space->params()->getParam("target_epsilon", target_eps)) {
        search->setTargetEpsilon(target_eps);
    }

    double delta_eps;
    if (space->params()->getParam("delta_epsilon", delta_eps)) {
        search->setDeltaEpsilon(delta_eps);
    }

    bool improve_solution;
    if (space->params()->getParam("improve_solution", improve_solution)) {
        search->setImproveSolution(improve_solution);
    }

    bool bound_expansions;
    if (space->params()->getParam("bound_expansions", bound_expansions)) {
        search->setBoundExpansions(bound_expansions);
    }

    double repair_time;
    if (space->params()->getParam("repair_time", repair_time)) {
        search->setAllowedRepairTime(repair_time);
    }

    double switch_dist;
    if (space->params()->getParam("switch_dist", switch_dist)) {
        search->setSwitchDistance(switch_dist);
    }

    return std::move(search);
}

auto MakeMHAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    struct MHAPlannerAdapter : public MHAPlanner {
        std::vector<Heuristic*> heuristics;

        MHAPlannerAdapter(
            DiscreteSpaceInformation* space,
            Heuristic* anchor,
            Heuristic** heurs,
            int hcount)
        :
            MHAPlanner(space, anchor, heurs, hcount)
        { }
    };

    std::vector<Heuristic*> heuristics;
    heuristics.push_back(heuristic);

    const bool forward_search = true;
    auto search = make_unique<MHAPlannerAdapter>(
            space, heuristics[0], &heuristics[0], heuristics.size());

    search->heuristics = std::move(heuristics);

    double mha_eps;
    space->params()->param("epsilon_mha", mha_eps, 1.0);
    search->set_initial_mha_eps(mha_eps);

    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);

    bool search_mode;
    space->params()->param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);

    return std::move(search);
}

auto MakeLARAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    const bool forward_search = true;
    auto search = make_unique<LazyARAPlanner>(space, forward_search);
    double epsilon;
    space->params()->param("epsilon", epsilon, 1.0);
    search->set_initialsolution_eps(epsilon);
    bool search_mode;
    space->params()->param("search_mode", search_mode, false);
    search->set_search_mode(search_mode);
    return std::move(search);
}

auto MakeEGWAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    return make_unique<ExperienceGraphPlanner>(space, heuristic);
}

auto MakePADAStar(RobotPlanningSpace* space, RobotHeuristic* heuristic)
    -> std::unique_ptr<SBPLPlanner>
{
    auto search = make_unique<AdaptivePlanner>(space, heuristic);

    double epsilon_plan;
    space->params()->param("epsilon_plan", epsilon_plan, 1.0);
    search->set_plan_eps(epsilon_plan);

    double epsilon_track;
    space->params()->param("epsilon_track", epsilon_track, 1.0);
    search->set_track_eps(epsilon_track);

    AdaptivePlanner::TimeParameters tparams;
    tparams.planning.bounded = true;
    tparams.planning.improve = false;
    tparams.planning.type = ARAStar::TimeParameters::TIME;
    tparams.planning.max_allowed_time_init = clock::duration::zero();
    tparams.planning.max_allowed_time = clock::duration::zero();

    tparams.tracking.bounded = true;
    tparams.tracking.improve = false;
    tparams.tracking.type = ARAStar::TimeParameters::TIME;
    tparams.tracking.max_allowed_time_init = std::chrono::seconds(5);
    tparams.tracking.max_allowed_time = clock::duration::zero();

    search->set_time_parameters(tparams);

    return std::move(search);
}

PlannerInterface::PlannerInterface(
    RobotModel* robot,
    CollisionChecker* checker,
    OccupancyGrid* grid)
:
    m_robot(robot),
    m_checker(checker),
    m_grid(grid),
    m_fk_iface(nullptr),
    m_params(),
    m_initialized(false),
    m_pspace(),
    m_heuristics(),
    m_planner(),
    m_sol_cost(INFINITECOST),
    m_planner_id(),
    m_req(),
    m_res()
{
    if (m_robot) {
        m_fk_iface = m_robot->getExtension<ForwardKinematicsInterface>();
    }

    ////////////////////////////////////
    // Setup Planning Space Factories //
    ////////////////////////////////////

    m_space_factories["manip"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        PlanningParams* p)
    {
        return MakeManipLattice(m_grid, r, c, p);
    };

    m_space_factories["manip_lattice_egraph"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        PlanningParams* p)
    {
        return MakeManipLatticeEGraph(m_grid, r, c, p);
    };

    m_space_factories["workspace"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        PlanningParams* p)
    {
        return MakeWorkspaceLattice(m_grid, r, c, p);
    };

    m_space_factories["adaptive_workspace_lattice"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        PlanningParams* p)
    {
        return MakeAdaptiveWorkspaceLattice(m_grid, r, c, p);
    };

    ///////////////////////////////
    // Setup Heuristic Factories //
    ///////////////////////////////

    m_heuristic_factories["mfbfs"] = [this](RobotPlanningSpace* space) {
        return MakeMultiFrameBFSHeuristic(space, m_grid, m_params);
    };

    m_heuristic_factories["bfs"] = [this](RobotPlanningSpace* space) {
        return MakeBFSHeuristic(space, m_grid, m_params);
    };

     m_heuristic_factories["mbfs"] = [this](RobotPlanningSpace* space) {
        return MakeMultiBFSHeuristic(space, m_grid, m_params);
    };

    m_heuristic_factories["euclid"] = [this](RobotPlanningSpace* space) {
        return MakeEuclidDistHeuristic(space, m_params);
    };

    m_heuristic_factories["joint_distance"] = [this](RobotPlanningSpace* space) {
        return MakeJointDistHeuristic(space);
    };

    m_heuristic_factories["bfs_egraph"] = [this](RobotPlanningSpace* space) {
        return MakeDijkstraEgraphHeuristic3D(space, m_grid, m_params);
    };

    m_heuristic_factories["joint_distance_egraph"] = [this](RobotPlanningSpace* space) {
        return MakeJointDistEGraphHeuristic(space, m_params);
    };

    /////////////////////////////
    // Setup Planner Factories //
    /////////////////////////////

    m_planner_factories["arastar"] = MakeARAStar;
    m_planner_factories["mhastar"] = MakeMHAStar;
    m_planner_factories["larastar"] = MakeLARAStar;
    m_planner_factories["egwastar"] = MakeEGWAStar;
    m_planner_factories["padastar"] = MakePADAStar;
    m_planner_factories["trastar"] = MakeTRAStar;
    m_planner_factories["mhtrastar"] = MakeMHTRAStar;
    m_planner_factories["marastar"] = MakeMARAStar;

    ros::NodeHandle nh;
    motionPlanResPub_ = nh.advertise<moveit_msgs::MotionPlanResponse>("/sbpl_planner/motion_plan_response",5);
    plannerDataPub_ = nh.advertise<moveit_msgs::PlanningData>("/sbpl_planner/planner_data",5);
    ROS_ERROR_STREAM("grid # occupied "<<m_grid->getOccupiedVoxelCount()<< " and frame "<<m_grid->getReferenceFrame()<<" and res is "<<m_grid->resolution());
    ROS_ERROR_STREAM("end of planner interface constructor");
    //SV_SHOW_INFO_NAMED("OCCUPIED", m_grid->getOccupiedVoxelsVisualization());
}

PlannerInterface::~PlannerInterface()
{
}

bool PlannerInterface::init(const PlanningParams& params)
{
    ROS_INFO_NAMED(PI_LOGGER, "initialize arm planner interface");

    ROS_INFO_NAMED(PI_LOGGER, "  Planning Frame: %s", params.planning_frame.c_str());

    ROS_INFO_NAMED(PI_LOGGER, "  Planning Link Sphere Radius: %0.3f", params.planning_link_sphere_radius);

    ROS_INFO_NAMED(PI_LOGGER, "  Shortcut Path: %s", params.shortcut_path ? "true" : "false");
    ROS_INFO_NAMED(PI_LOGGER, "  Shortcut Type: %s", to_string(params.shortcut_type).c_str());
    ROS_INFO_NAMED(PI_LOGGER, "  Interpolate Path: %s", params.interpolate_path ? "true" : "false");

    ROS_ERROR_STREAM("in planner interface init!");
    if (!checkConstructionArgs()) {
        return false;
    }

    if (!checkParams(params)) {
        return false;
    }

    m_params = params;

    m_grid->setReferenceFrame(m_params.planning_frame);

    lastPlanRequestId_ = -1;
    m_initialized = true;

    ROS_INFO_NAMED(PI_LOGGER, "initialized arm planner interface");
    return m_initialized;
}

bool PlannerInterface::checkConstructionArgs() const
{
    if (!m_robot) {
        ROS_ERROR("Robot Model given to Arm Planner Interface must be non-null");
        return false;
    }

    if (!m_checker) {
        ROS_ERROR("Collision Checker given to Arm Planner Interface must be non-null");
        return false;
    }

    if (!m_grid) {
        ROS_ERROR("Occupancy Grid given to Arm Planner Interface must be non-null");
        return false;
    }

    return true;
}

bool PlannerInterface::solve(
    const moveit_msgs::PlanningScene& planning_scene,
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res)
{
    ROS_ERROR_STREAM("planner interface solve call");
    clearMotionPlanResponse(req, res);

    if (!m_initialized) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    if (!canServiceRequest(req, res)) {
        return false;
    }

    m_req = req; // record the last attempted request


    if (req.goal_constraints.empty()) {
        ROS_WARN_NAMED(PI_LOGGER, "No goal constraints in request!");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return true;
    }
    
    // TODO: lazily reinitialize planner when algorithm changes
    if (!reinitPlanner(req.planner_id)) {
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

     // plan
    res.trajectory_start = planning_scene.robot_state;
    ROS_INFO_NAMED(PI_LOGGER, "Allowed Time (s): %0.3f", req.allowed_planning_time);

    auto then = clock::now();

    std::vector<RobotState> path;
    if (req.goal_constraints.front().position_constraints.size() > 0) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning to position!");
        if (!planToPose(planning_scene, req, path, res)) {
            auto now = clock::now();
            res.planning_time = to_seconds(now - then);
            return false;
        }
    } else if (req.goal_constraints.front().joint_constraints.size() > 0) {
        ROS_INFO_NAMED(PI_LOGGER, "Planning to joint configuration!");
        if (!planToConfiguration(req, path, res)) {
            auto now = clock::now();
            res.planning_time = to_seconds(now - then);
            return false;
        }
    } else {
        ROS_ERROR("Both position and joint constraints empty!");
        auto now = clock::now();
        res.planning_time = to_seconds(now - then);
        return false;
    }

    ROS_DEBUG_NAMED(PI_LOGGER, "planner path:");
    for (size_t pidx = 0; pidx < path.size(); ++pidx) {
        const auto& point = path[pidx];
        ROS_DEBUG_STREAM_NAMED(PI_LOGGER, "  " << pidx << ": " << point);
    }
    auto now = clock::now();
    ROS_INFO_STREAM("just after planning the time is "<<to_seconds(now - then));

    postProcessPath(path);
    
    now = clock::now();

    ROS_INFO_STREAM("just after post process the time is "<<to_seconds(now - then));

    SV_SHOW_DEBUG_NAMED("trajectory", makePathVisualization(path));

    ROS_DEBUG_NAMED(PI_LOGGER, "smoothed path:");
    for (size_t pidx = 0; pidx < path.size(); ++pidx) {
        const auto& point = path[pidx];
        ROS_DEBUG_STREAM_NAMED(PI_LOGGER, "  " << pidx << ": " << point);
    }

    auto& traj = res.trajectory.joint_trajectory;
    convertJointVariablePathToJointTrajectory(path, traj);
    traj.header.seq = 0;
    traj.header.stamp = ros::Time::now();

    /*if (!m_params.plan_output_dir.empty()) {
        writePath(res.trajectory_start, res.trajectory);
    }

    now = clock::now();
    
    ROS_INFO_STREAM("just after convert & write the time is "<<to_seconds(now - then));

    profilePath(traj);

    now = clock::now();
    
    ROS_INFO_STREAM("just after profiling the time is "<<to_seconds(now - then));*/
//    removeZeroDurationSegments(traj);

    now = clock::now();
    //keep statistical metrics for benchmarking (expansion & epsilon...TODO change the response message to add the cost as well)
   
    res.planning_time = to_seconds(now - then);
    res.iterations = m_planner->get_n_expands();
    res.cost = m_planner->get_final_epsilon();
    m_pspace->getPlanningData()->planningTime_ = res.planning_time;

    m_res = res; // record the last result
    ROS_ERROR_STREAM("Planning succeeded with planning time "<<res.planning_time);
    moveit_msgs::PlanningData planning_data_msg;
    convertPlanningDataToMsg(planning_data_msg);
    plannerDataPub_.publish(planning_data_msg);
    motionPlanResPub_.publish(m_res);
    return true;
}

bool PlannerInterface::checkParams(
    const PlanningParams& params) const
{
    if (params.planning_frame.empty()) {
        return false;
    }

    // TODO: check for existence of planning joints in robot model

    if (params.cost_per_cell < 0) {
        return false;
    }

    return true;
}

bool PlannerInterface::setMultipleStart(std::vector<moveit_msgs::RobotState>& states, int best_idx)
{
    ROS_INFO_NAMED(PI_LOGGER, "set multiple start configuration");

    /*std::string file_name = "../g500_csip/g500_csip_benchmarks/objects/subsea_controller/binvox_subsea";
    bool res = leatherman::createBinvoxFile("../g500_csip/g500_csip_benchmarks/objects/subsea_controller/subsea_controller.stl",file_name);
    if(res)
        ROS_WARN("created!");
    else
        ROS_WARN("failed!");*/
    ROS_ERROR("beginning to set multiple start");
    std::vector<RobotState> initial_positions(states.size());
    std::vector<std::string> missing;
    for(int i=0;i<states.size();i++)
    {
        if (!leatherman::getJointPositions(
                states[i].joint_state,
                states[i].multi_dof_joint_state,
                m_robot->getPlanningJoints(),
                initial_positions[i],
                missing))
        {
             ROS_ERROR_STREAM("start state is missing planning joints: " << missing);
            return false;
        }
    }


    //ROS_INFO_STREAM("Initial joint variables: " << initial_positions);
    /*RobotState best = initial_positions[best_idx];
    initial_positions.clear();
    initial_positions.push_back(best);*/
    if (!m_pspace->setMultipleStart(initial_positions)) {
        ROS_ERROR("Failed to set multiple start state");
        return false;
    }

    const std::vector<int> start_ids = m_pspace->getStartStatesID();
    if (start_ids.empty()) {
        ROS_ERROR("No start state has been set");
        return false;
    }

    if (m_planner->set_multiple_start(start_ids) == 0) {
        ROS_ERROR("Failed to set start state");
        return false;
    }

    ROS_ERROR("end to set multiple start");
    return true;
}

bool PlannerInterface::setStart(const moveit_msgs::RobotState& state)
{
    ROS_INFO_NAMED(PI_LOGGER, "set start configuration");

    /*std::string file_name = "../g500_csip/g500_csip_benchmarks/objects/subsea_controller/binvox_subsea";
    bool res = leatherman::createBinvoxFile("../g500_csip/g500_csip_benchmarks/objects/subsea_controller/subsea_controller.stl",file_name);
    if(res)
        ROS_WARN("created!");
    else
        ROS_WARN("failed!");*/
    RobotState initial_positions;
    std::vector<std::string> missing;
    if (!leatherman::getJointPositions(
            state.joint_state,
            state.multi_dof_joint_state,
            m_robot->getPlanningJoints(),
            initial_positions,
            missing))
    {
         ROS_ERROR_STREAM("start state is missing planning joints: " << missing);
        return false;
    }

    ROS_INFO_STREAM("Initial joint variables: " << initial_positions);

    if (!m_pspace->setStart(initial_positions)) {
        ROS_ERROR("Failed to set start state");
        return false;
    }

    const int start_id = m_pspace->getStartStateID();
    if (start_id == -1) {
        ROS_ERROR("No start state has been set");
        return false;
    }

    if (m_planner->set_start(start_id) == 0) {
        ROS_ERROR("Failed to set start state");
        return false;
    }

    return true;
}

bool PlannerInterface::setGoalConfiguration(
    const moveit_msgs::Constraints& goal_constraints)
{
    ROS_INFO_NAMED(PI_LOGGER, "Set goal configuration");

    std::vector<double> sbpl_angle_goal(m_robot->jointVariableCount(), 0);
    std::vector<double> sbpl_angle_tolerance(m_robot->jointVariableCount(), angles::to_radians(3.0));

    if (goal_constraints.joint_constraints.size() < m_robot->jointVariableCount()) {
        ROS_WARN_NAMED(PI_LOGGER, "All %zu arm joint constraints must be specified for goal!", m_robot->jointVariableCount());
        return false;
    }
    if (goal_constraints.joint_constraints.size() > m_robot->jointVariableCount()) {
        ROS_WARN_NAMED(PI_LOGGER, "%d joint constraints specified! Using the first %zu!", (int)goal_constraints.joint_constraints.size(), m_robot->jointVariableCount());
        return false;
    }

    const size_t num_angle_constraints = std::min(
            goal_constraints.joint_constraints.size(), sbpl_angle_goal.size());
    for (size_t i = 0; i < num_angle_constraints; i++) {
        const auto& joint_constraint = goal_constraints.joint_constraints[i];
        const std::string& joint_name = joint_constraint.joint_name;
        auto jit = std::find(
                m_robot->getPlanningJoints().begin(),
                m_robot->getPlanningJoints().end(),
                joint_name);
        if (jit == m_robot->getPlanningJoints().end()) {
            ROS_ERROR("Failed to find goal constraint for joint '%s'", joint_name.c_str());
            return false;
        }
        int jidx = std::distance(m_robot->getPlanningJoints().begin(), jit);
        sbpl_angle_goal[jidx] = joint_constraint.position;
        sbpl_angle_tolerance[jidx] = std::min(
                fabs(joint_constraint.tolerance_above),
                fabs(joint_constraint.tolerance_below));
        ROS_DEBUG_NAMED(PI_LOGGER, "Joint %zu [%s]: goal position: %.3f, goal tolerance: %.3f", i, joint_name.c_str(), sbpl_angle_goal[jidx], sbpl_angle_tolerance[jidx]);
    }

    GoalConstraint goal;
    goal.type = GoalType::JOINT_STATE_GOAL;
    goal.angles = sbpl_angle_goal;
    goal.angle_tolerances = sbpl_angle_tolerance;

    // TODO: really need to reevaluate the necessity of the planning link
    if (m_fk_iface) {
        m_fk_iface->computePlanningLinkFK(goal.angles, goal.pose);
        goal.tgt_off_pose = goal.pose;
    } else {
        goal.pose.resize(6, 0.0);
        goal.tgt_off_pose.resize(6, 0.0);
    }

    ROS_ERROR_STREAM("Setting the planner interface goal "<< goal.pose[0]<<","<<goal.pose[1]<<","<<goal.pose[2]);
    ROS_ERROR_STREAM("Setting the planner interface goal off pose "<<goal.tgt_off_pose[0]<<","<<goal.tgt_off_pose[1]<<","<<goal.tgt_off_pose[2]);
    // set sbpl environment goal
    if (!m_pspace->setGoal(goal)) {
        ROS_ERROR("Failed to set goal");
        return false;
    }

    // set planner goal
    const int goal_id = m_pspace->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return false;
    }

    if (m_planner->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return false;
    }

    return true;
}

bool PlannerInterface::setGoalPosition(
    const moveit_msgs::Constraints& goal_constraints)
{
    ROS_INFO_NAMED(PI_LOGGER, "Setting goal position");

    Eigen::Affine3d goal_pose;
    Eigen::Vector3d offset;
    if (!extractGoalPoseFromGoalConstraints(
            goal_constraints, goal_pose, offset))
    {
        ROS_WARN_NAMED(PI_LOGGER, "Failed to extract goal pose from goal constraints");
        return false;
    }

    GoalConstraint goal;
    goal.type = GoalType::XYZ_RPY_GOAL;
    goal.pose.resize(6);
    goal.pose[0] = goal_pose.translation()[0];
    goal.pose[1] = goal_pose.translation()[1];
    goal.pose[2] = goal_pose.translation()[2];

    angles::get_euler_zyx(
            goal_pose.rotation(), goal.pose[5], goal.pose[4], goal.pose[3]);
    goal.xyz_offset[0] = offset.x();
    goal.xyz_offset[1] = offset.y();
    goal.xyz_offset[2] = offset.z();

    std::vector<double> sbpl_tolerance(6, 0.0);
    if (!extractGoalToleranceFromGoalConstraints(goal_constraints, &sbpl_tolerance[0])) {
        ROS_WARN_NAMED(PI_LOGGER, "Failed to extract goal tolerance from goal constraints");
        return false;
    }

    goal.xyz_tolerance[0] = sbpl_tolerance[0];
    goal.xyz_tolerance[1] = sbpl_tolerance[1];
    goal.xyz_tolerance[2] = sbpl_tolerance[2];
    goal.rpy_tolerance[0] = sbpl_tolerance[3];
    goal.rpy_tolerance[1] = sbpl_tolerance[4];
    goal.rpy_tolerance[2] = sbpl_tolerance[5];

    ROS_INFO_NAMED(PI_LOGGER, "New Goal");
    ROS_INFO_NAMED(PI_LOGGER, "    frame: %s", m_params.planning_frame.c_str());
    ROS_INFO_NAMED(PI_LOGGER, "    pose: (x: %0.3f, y: %0.3f, z: %0.3f, R: %0.3f, P: %0.3f, Y: %0.3f)", goal.pose[0], goal.pose[1], goal.pose[2], goal.pose[3], goal.pose[4], goal.pose[5]);
    ROS_INFO_NAMED(PI_LOGGER, "    offset: (%0.3f, %0.3f, %0.3f)", goal.xyz_offset[0], goal.xyz_offset[1], goal.xyz_offset[2]);
    ROS_INFO_NAMED(PI_LOGGER, "    tolerance: (dx: %0.3f, dy: %0.3f, dz: %0.3f, dR: %0.3f, dP: %0.3f, dY: %0.3f)", sbpl_tolerance[0], sbpl_tolerance[1], sbpl_tolerance[2], sbpl_tolerance[3], sbpl_tolerance[4], sbpl_tolerance[5]);

    // ...a lot more relies on this than I had hoped
    Eigen::Affine3d target_pose = goal_pose * Eigen::Translation3d(offset);
    goal.tgt_off_pose =
    {
        target_pose.translation()[0],
        target_pose.translation()[1],
        target_pose.translation()[2],
        goal.pose[3],
        goal.pose[4],
        goal.pose[5]
    };


    if (!m_pspace->setGoal(goal)) {
        ROS_ERROR("Failed to set goal");
        return false;
    }
    
    // set sbpl planner goal
    const int goal_id = m_pspace->getGoalStateID();
    if (goal_id == -1) {
        ROS_ERROR("No goal state has been set");
        return false;
    }

    if (m_planner->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return false;
    }

    return true;
}

bool PlannerInterface::replan(double allowed_time, std::vector<RobotState>& path, int restore_step)
{
    
    ROS_INFO_STREAM("RE-Planning!!!!! by restoring tree at step "<<restore_step);
    
    //SV_SHOW_INFO_NAMED("bfs_walls_replan", getBfsWallsVisualization());
    //SV_SHOW_INFO_NAMED("bfs_values_replan", getBfsValuesVisualization());


    bool b_ret = false;
    std::vector<int> solution_state_ids;

    // reinitialize the search space
    //m_planner->force_planning_from_scratch();

    // plan
    ReplanParams params(allowed_time);
    params.restore_step = restore_step;
    b_ret = m_planner->replan(&solution_state_ids, params, &m_sol_cost);
    ROS_WARN_STREAM("Planner replan done with value "<<b_ret);
    //replan(allowed_time, &solution_state_ids, &m_sol_cost);

    // check if an empty plan was received.
    if (b_ret && solution_state_ids.size() <= 0) {
        ROS_WARN_NAMED(PI_LOGGER, "Path returned by the planner is empty?");
        b_ret = false;
    }

    // if a path is returned, then pack it into msg form
    if (b_ret && (solution_state_ids.size() > 0)) {
        m_pspace->getPlanningData()->numExpansions_ = m_planner->get_n_expands();
        m_pspace->getPlanningData()->cost_ = m_sol_cost;
        m_pspace->getPlanningData()->searchTime_ = m_planner->get_final_eps_planning_time();
        m_pspace->getPlanningData()->finalEpsilon_ = m_planner->get_solution_eps();
        m_pspace->getPlanningData()->pathNumStates_ = solution_state_ids.size();

        ROS_INFO_NAMED(PI_LOGGER, "Planning succeeded");
        ROS_INFO_NAMED(PI_LOGGER, "  Num Expansions (Initial): %d", m_planner->get_n_expands_init_solution());
        ROS_INFO_NAMED(PI_LOGGER, "  Num Expansions (Final): %d", m_planner->get_n_expands());
        ROS_INFO_NAMED(PI_LOGGER, "  Epsilon (Initial): %0.3f", m_planner->get_initial_eps());
        ROS_INFO_NAMED(PI_LOGGER, "  Epsilon (Final): %0.3f", m_planner->get_solution_eps());
        ROS_INFO_NAMED(PI_LOGGER, "  Time (Initial): %0.3f", m_planner->get_initial_eps_planning_time());
        ROS_INFO_NAMED(PI_LOGGER, "  Time (Final): %0.3f", m_planner->get_final_eps_planning_time());
        ROS_INFO_NAMED(PI_LOGGER, "  Path Length (states): %zu", solution_state_ids.size());
        ROS_INFO_NAMED(PI_LOGGER, "  Solution Cost: %d", m_sol_cost);

        path.clear();
        auto startTime = ros::Time::now().toSec();
        if (!m_pspace->extractPath(solution_state_ids, path)) {
            ROS_ERROR("Failed to convert state id path to joint variable path");
            return false;
        }
        auto endTime = ros::Time::now().toSec();
        m_pspace->getPlanningData()->pathExtractionTime_ = endTime - startTime;
    }
    return b_ret;

    
}

bool PlannerInterface::plan(double allowed_time, std::vector<RobotState>& path)
{
    // NOTE: this should be done after setting the start/goal in the environment
    // to allow the heuristic to tailor the visualization to the current
    // scenario
    SV_SHOW_INFO_NAMED("bfs_walls", getBfsWallsVisualization());
    SV_SHOW_INFO_NAMED("bfs_values", getBfsValuesVisualization());

    ROS_WARN_NAMED(PI_LOGGER, "Planning!!!!!");
    bool b_ret = false;
    std::vector<int> solution_state_ids;

    // reinitialize the search space
    m_planner->force_planning_from_scratch();

    // plan
    b_ret = m_planner->replan(allowed_time, &solution_state_ids, &m_sol_cost);

    // check if an empty plan was received.
    if (b_ret && solution_state_ids.size() <= 0) {
        ROS_WARN_NAMED(PI_LOGGER, "Path returned by the planner is empty?");
        b_ret = false;
    }

    // if a path is returned, then pack it into msg form
    if (b_ret && (solution_state_ids.size() > 0)) {
        m_pspace->getPlanningData()->numExpansions_ = m_planner->get_n_expands();
        m_pspace->getPlanningData()->cost_ = m_sol_cost;
        m_pspace->getPlanningData()->searchTime_ = m_planner->get_final_eps_planning_time();
        m_pspace->getPlanningData()->finalEpsilon_ = m_planner->get_solution_eps();
        m_pspace->getPlanningData()->pathNumStates_ = solution_state_ids.size();

        ROS_INFO_NAMED(PI_LOGGER, "Planning succeeded");
        ROS_INFO_NAMED(PI_LOGGER, "  Num Expansions (Initial): %d", m_planner->get_n_expands_init_solution());
        ROS_INFO_NAMED(PI_LOGGER, "  Num Expansions (Final): %d", m_planner->get_n_expands());
        ROS_INFO_NAMED(PI_LOGGER, "  Epsilon (Initial): %0.3f", m_planner->get_initial_eps());
        ROS_INFO_NAMED(PI_LOGGER, "  Epsilon (Final): %0.3f", m_planner->get_solution_eps());
        ROS_INFO_NAMED(PI_LOGGER, "  Time (Initial): %0.3f", m_planner->get_initial_eps_planning_time());
        ROS_INFO_NAMED(PI_LOGGER, "  Time (Final): %0.3f", m_planner->get_final_eps_planning_time());
        ROS_INFO_NAMED(PI_LOGGER, "  Path Length (states): %zu", solution_state_ids.size());
        ROS_INFO_NAMED(PI_LOGGER, "  Solution Cost: %d", m_sol_cost);

        path.clear();

        auto startTime = ros::Time::now().toSec();
        if (!m_pspace->extractPath(solution_state_ids, path)) {
            ROS_ERROR("Failed to convert state id path to joint variable path");
            return false;
        }

        auto endTime = ros::Time::now().toSec();
        m_pspace->getPlanningData()->pathExtractionTime_ = endTime - startTime;
        for(int i=0;i<solution_state_ids.size();i++)
        {
            SolutionStateInfo state;
            state.state_id = solution_state_ids[i];
            state.state_config = path[i];
            m_pspace->getPlanningData()->solutionStates_.push_back(state);
        }
    }
    return b_ret;
}

bool PlannerInterface::planToPose(const moveit_msgs::PlanningScene& scene_msg,
    const moveit_msgs::MotionPlanRequest& req,
    std::vector<RobotState>& path,
    moveit_msgs::MotionPlanResponse& res)
{
    std::string search_name;
    std::string heuristic_name;
    std::string space_name;

    m_pspace->setMotionPlanRequestType(req.request_type);
    
    
    parsePlannerID(m_planner_id, space_name, heuristic_name, search_name);
    auto then = clock::now();
    bool result;
    if (search_name == "trastar" || search_name == "mhtrastar")
    {
        result =  planToPoseWithMultipleIK(scene_msg, req,path,res);
        auto now = clock::now();
        ROS_INFO_STREAM("Around plan to pose with multiple IK "<<to_seconds(now-then));
        return result;
    }

    const auto& goal_constraints_v = req.goal_constraints;
    assert(!goal_constraints_v.empty());

    // transform goal pose into reference_frame

    // only acknowledge the first constraint
    const auto& goal_constraints = goal_constraints_v.front();
    then = clock::now();
    // lastPlanRequestId_!=req.request_id &&
    if (!setGoalPosition(goal_constraints)) {
        ROS_ERROR("Failed to set goal position");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
        return false;
    }

    if (!setStart(req.start_state)) {
        ROS_ERROR("Failed to set initial configuration of robot");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
        return false;
    }
    if (!plan(req.allowed_planning_time, path)) {
            ROS_ERROR("Failed to re-plan within alotted time frame (%0.2f seconds, %d expansions)", req.allowed_planning_time, m_planner->get_n_expands());
            res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return false;
        }
    auto now = clock::now();
    ROS_INFO_STREAM("Around simple plan "<<to_seconds(now-then));
    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    lastPlanRequestId_ = req.request_id;
    return true;
}

bool isIKSolutionValid(const planning_scene::PlanningScene *planning_scene,
                       const kinematic_constraints::KinematicConstraintSet *constraint_set,
                       robot_state::RobotState *state, const robot_model::JointModelGroup *jmg,
                       const double *ik_solution)
{
    state->setJointGroupPositions(jmg, ik_solution);
    state->update();
    if(!state->satisfiesBounds(jmg))
    {
        ROS_ERROR_STREAM("out of bounds state ");
        return false;
    }
    if(!planning_scene)
    {
        ROS_ERROR_STREAM("No planning scene");
        return false;
    }
    if(planning_scene->isStateColliding(*state, jmg->getName(), true))
    {
        ROS_ERROR_STREAM("Colliding");
        return false;
    }
    return true;
}

bool PlannerInterface::planToConfigurationWithMultipleIK(const moveit_msgs::MotionPlanRequest& req,
        std::vector<RobotState>& path,
        moveit_msgs::MotionPlanResponse& res)
{

    const auto& goal_constraints_v = req.goal_constraints;
        assert(!goal_constraints_v.empty());

    // only acknowledge the first constraint
    const auto& goal_constraints = goal_constraints_v.front();
    
    moveit_msgs::Constraints startAsGoal;
    sensor_msgs::JointState start = req.start_state.joint_state;
    
    std::vector<double> sbpl_tolerance(6, 0.0);
    if (!extractGoalToleranceFromGoalConstraints(goal_constraints, &sbpl_tolerance[0])) {
        ROS_WARN_NAMED(PI_LOGGER, "Failed to extract goal tolerance from goal constraints");
        return false;
    }


    for (std::size_t i = 0; i < start.name.size(); ++i)
    {
        moveit_msgs::JointConstraint jnt;
        startAsGoal.joint_constraints.push_back(jnt);
        startAsGoal.joint_constraints[i].joint_name = start.name[i];
        startAsGoal.joint_constraints[i].position = start.position[i];
        startAsGoal.joint_constraints[i].tolerance_above = sbpl_tolerance[0];
        startAsGoal.joint_constraints[i].tolerance_below = -sbpl_tolerance[0];
        startAsGoal.joint_constraints[i].weight = 1.0;
    }

        
        if (!setGoalConfiguration(startAsGoal)) {
            ROS_ERROR("Failed to set goal position");
            res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
            return false;
        }

    if(lastPlanRequestId_!=req.request_id)
    {
        ROS_INFO_STREAM("New request received with id "<<req.request_id);
        
        std::vector<moveit_msgs::RobotState> start_states;
        moveit_msgs::RobotState temp_state = req.start_state;
        for(int i=0; i<goal_constraints.joint_constraints.size();i++)
        {
            temp_state.joint_state.position[i] = goal_constraints.joint_constraints[i].position;
        }
        start_states.push_back(temp_state);
        auto startTime = ros::Time::now();
        if (!setMultipleStart(start_states, 0)) {
            ROS_ERROR("Failed to set initial configuration of robot");
            res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
            return false;
        }

        ROS_INFO_STREAM("Setting multiple Start "<<(ros::Time::now()-startTime).toSec());
    }
    if(lastPlanRequestId_==req.request_id)
    {   
        std::map<std::vector<int>,int> added,removed;
        added = m_grid->getAddedCells();
        removed = m_grid->getRemovedCells(); 
        int x = added.size();
        int y = removed.size();
        ROS_WARN_STREAM("Replannign with added vs. removed "<<x<<":"<<y);
        int min_added_expansion = INT_MAX;
        int min_removed_expansion = INT_MAX;
        ros::Time startTime = ros::Time::now();
        for(std::map<std::vector<int>,int>::iterator it=added.begin();it!=added.end();++it)
        {
            ROS_INFO_STREAM(" added with step "<<it->second);
            if(it->second<min_added_expansion)
            {
                ROS_INFO_STREAM("found smaller added with old step "<<min_added_expansion<<" and new "<<it->second);
                min_added_expansion = it->second;
            }
        }
        for(std::map<std::vector<int>,int>::iterator it=removed.begin();it!=removed.end();++it)
        {
            ROS_INFO_STREAM("removed with step "<<it->second);
            if(it->second<min_removed_expansion)
            {
                ROS_INFO_STREAM("found smaller removed with old step "<<min_removed_expansion<<" and new "<<it->second);
               
                min_removed_expansion = it->second;
            }
        }

        int restore_step = std::min(min_added_expansion,min_removed_expansion)-1;

        resetCellsMarking(restore_step);
        
        ROS_INFO_STREAM("Replan overhead, marking reset "<<(ros::Time::now()-startTime).toSec());
        startTime = ros::Time::now();
        if (!replan(req.allowed_planning_time, path, restore_step)) {
            ROS_ERROR("Failed to re-plan within alotted time frame (%0.2f seconds, %d expansions)", req.allowed_planning_time, m_planner->get_n_expands());
            res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return false;
        }

        ROS_INFO_STREAM("Replan time "<<(ros::Time::now()-startTime).toSec());
    }
    else
    {
        ros::Time startTime = ros::Time::now();
        if (!plan(req.allowed_planning_time, path)) {
            ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds, %d expansions)", req.allowed_planning_time, m_planner->get_n_expands());
            res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return false;
        }
        ROS_INFO_STREAM("initial plan  time "<<(ros::Time::now()-startTime).toSec());
    }

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    lastPlanRequestId_ = req.request_id;
    return true; 
}

bool PlannerInterface::planToPoseWithMultipleIK(const moveit_msgs::PlanningScene& scene_msg,
        const moveit_msgs::MotionPlanRequest& req,
        std::vector<RobotState>& path,
        moveit_msgs::MotionPlanResponse& res)
{
   const auto& goal_constraints_v = req.goal_constraints;
    assert(!goal_constraints_v.empty());

    // only acknowledge the first constraint
    const auto& goal_constraints = goal_constraints_v.front();

    std::vector<double> sbpl_tolerance(6, 0.0);
    if (!extractGoalToleranceFromGoalConstraints(goal_constraints, &sbpl_tolerance[0])) {
        ROS_WARN_NAMED(PI_LOGGER, "Failed to extract goal tolerance from goal constraints");
        return false;
    }

    moveit_msgs::Constraints startAsGoal;
    sensor_msgs::JointState start = req.start_state.joint_state;
    
    for (std::size_t i = 0; i < start.name.size(); ++i)
    {
        moveit_msgs::JointConstraint jnt;
        startAsGoal.joint_constraints.push_back(jnt);
        startAsGoal.joint_constraints[i].joint_name = start.name[i];
        startAsGoal.joint_constraints[i].position = start.position[i];
        startAsGoal.joint_constraints[i].tolerance_above = sbpl_tolerance[0];
        startAsGoal.joint_constraints[i].tolerance_below = -sbpl_tolerance[0];
        startAsGoal.joint_constraints[i].weight = 1.0;
    }
    
    if (!setGoalConfiguration(startAsGoal)) {
        ROS_ERROR("Failed to set goal position");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
        return false;
    }
    
    if(lastPlanRequestId_!=req.request_id)
    {
        ROS_INFO_STREAM("New request received with id "<<req.request_id);
        const moveit_msgs::PositionConstraint& position_constraint = goal_constraints.position_constraints.front();
        const moveit_msgs::OrientationConstraint& orientation_constraint = goal_constraints.orientation_constraints.front();

        const geometry_msgs::Pose& primitive_pose = position_constraint.constraint_region.primitive_poses.front();
        moveit_msgs::RobotState goalAsStart;
        
        ros::Time startTime = ros::Time::now();

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id="world";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = primitive_pose.position.x;
        pose.pose.position.y = primitive_pose.position.y;
        pose.pose.position.z = primitive_pose.position.z;
        pose.pose.orientation = orientation_constraint.orientation;
        std::vector<moveit_msgs::RobotState> start_states;
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_state::RobotState state(robot_model_loader.getModel());
        
        kinematic_constraints::KinematicConstraintSet kset(robot_model_loader.getModel());
        const planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model_loader.getModel()));
        planning_scene->setPlanningSceneMsg(scene_msg);
        const moveit::core::JointModelGroup* joint_group = state.getJointModelGroup(req.group_name);
        bool result = true;
        std::vector<double> position(8,0);
       /* if(req.request_id==0)
        {
            //position = {2.51559, 0.170909, 4.40869, -1.29945, 0.909369, 0.014292, 0.0427095, 0.389251};
            //position = {2.05823,-0.586781,4.39388,-0.427331,0.909369, -0.05,0.10701,-0.482871};
            //position = {2.28255,-1.33881,4.49743,0.350936,0.909373,0.405431,-0.34843,-1.26114};
            //position = {2.22561,-0.122468,4.39388,-0.902797,0.909369,-0.05,0.107008,-0.00740517};
            //position = {2.28,-1.32858, 4.51035,0.34442,0.909365,0.465657,-0.408656,-1.25462};
            //position={2.11281,-0.37205,4.48744,-0.634947,0.909369, 0.359815, -0.302813, -0.275255};
            position={14,0,5,0,0,0,0,0};
            state.setVariablePositions (position);
            moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
            start_states.push_back(goalAsStart);
        }
        else
        {
            state.setFromIK(joint_group,pose.pose,1,0.05,boost::bind(&isIKSolutionValid,planning_scene.get(), kset.empty() ? NULL : &kset, _1, _2, _3));
            
           
            moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
            
            if(result)
            {
                start_states.push_back(goalAsStart);
                ROS_ERROR_STREAM("found!");
                ROS_INFO_STREAM(state);
            }
                
            else
                ROS_ERROR_STREAM("invalid solution!");

            /* Start sampling the yaw and generate 12 seeds covering different yaws */
          /*  std::vector<double> yawDecomposition = {0,0.52,1.0, 1.57, 2.0, 2.6, 3.14, 3.66, 4.1, 4.7, 5.2, 5.8};
            for(int i=0; i<yawDecomposition.size(); i++)
            {
                state.setToRandomPositions(joint_group);
                state.setVariablePosition(state.getVariableNames()[3], yawDecomposition[i]);
                state.setVariablePosition(state.getVariableNames()[4], scene_msg.robot_state.joint_state.position[4]);
                state.setVariablePosition(state.getVariableNames()[5], scene_msg.robot_state.joint_state.position[5]);
                state.setVariablePosition(state.getVariableNames()[6], scene_msg.robot_state.joint_state.position[6]);
                state.setVariablePosition(state.getVariableNames()[7], scene_msg.robot_state.joint_state.position[7]);
                
                result = state.setFromIK(joint_group,pose.pose,1,0.05,boost::bind(&isIKSolutionValid,planning_scene.get(), kset.empty() ? NULL : &kset, _1, _2, _3));
                moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
                if(result)
                {
                    start_states.push_back(goalAsStart);
                    ROS_DEBUG_STREAM("found!");
                    ROS_INFO_STREAM(goalAsStart.joint_state);
                }
                else
                    ROS_ERROR_STREAM("invalid solution!");
            }
        
        }*/
        position = {22.2282, 1.34413, 2.94735, -2.72511, -0.42, 1.03368, 0.55, -0.0561427}; 
        //{4.07492, 0.378673, 4.74129, 3.14159, 0.909369, -0.0499901, 0.106992, 2.23138 };
        state.setVariablePositions (position);
        moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
        start_states.push_back(goalAsStart);

        position = {22.2282, 1.05203,2.94525, -3.13294, -0.012599, 1.05449, 0.55, -0.0462054};
        //{2.18927, -0.583771, 4.86344, 0.234922, 0.909365, 0.492631, -0.43563, -1.14512};
        state.setVariablePositions (position);
        moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
        start_states.push_back(goalAsStart);


        position = { 22.2007,1.04727, 2.97618, -3.14159, -0.00394715, 1.44, 0.164893, -0.0459056};
        //{3.99882, -0.541233, 4.76421, 2.25298, 0.909369, 0.0494151, 0.00758647, 3.12};
        state.setVariablePositions (position);
        moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
        start_states.push_back(goalAsStart);


        position = { 21.9207, 0.472544, 2.97324, 2.24697, 0.891375, 1.44, 0.186985, -0.00182074}; 
        //{2.10853, -0.427041, 4.87433, 0.0702956, 0.909369, 0.545313, -0.488312, -0.980498};
        state.setVariablePositions (position);
        moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
        start_states.push_back(goalAsStart);


        position = {21.741, 0.340013, 2.94351, 1.85009, 1.28883, 1.07293,  0.55, 0.0203087};
        //{3.93958, -0.542388, 4.92045, 2.25297, 0.909369, 0.789559, -0.732558, 3.12};
        state.setVariablePositions (position);
        moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
        start_states.push_back(goalAsStart);


        position = {21.5497,0.312556, 2.95421, 1.60911, 1.53, 1.15657, 0.459793, 0.0323417}; 
        //{3.80233, -0.764837, 4.85745, 1.98273, 0.909369, 0.464231, -0.40723, -2.89293 };
        state.setVariablePositions (position);
        moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
        start_states.push_back(goalAsStart);

        //CHOSEN GOAL
        position = {21.4878, 0.312044, 2.97466,1.60911, 1.53, 1.44, 0.176358,0.0323418}; 
        //{3.821, -0.707116, 4.91116, 2.05116, 0.909367, 0.736732, -0.67973, -2.96136};
        state.setVariablePositions (position);
        moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
        start_states.push_back(goalAsStart);

        position = {22.2007, 1.04728, 2.97618, 3.14159, -0.00396183, 1.44, 0.164893, -0.0459061}; 
        //{2.34059, -0.782995, 4.81714, 0.462571, 0.909369, 0.280429, -0.223428, -1.37277};
        state.setVariablePositions (position);
        moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
        start_states.push_back(goalAsStart);

        position = {22.1996, 1.01364, 2.97591, 3.09602, 0.0416018, 1.44, 0.166902, -0.0442719}; 
        //{3.83826, -0.577701, 4.97669, 2.20981, 0.909369, 1.20579, -1.14879, -3.12};
        state.setVariablePositions (position);
        moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
        start_states.push_back(goalAsStart);

        position = {22.2007, 1.04728, 2.97618, 3.14159, -0.00396077, 1.44,0.164894, -0.045906}; 
        //{4.07376, 0.377261, 4.80396, 3.14159, 0.909369, 0.22226, -0.165259, 2.23138 };
        state.setVariablePositions (position);
        moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
        start_states.push_back(goalAsStart);

        position = {22.2835, 1.04566, 2.94522, 3.14159, -0.00396072, 1.05489, 0.55, -0.045906};
        //{4.07554, 0.379516, 4.77725, 3.14159, 0.909369, 0.105901, -0.0488999, 2.23138};
        state.setVariablePositions (position);
        moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
        start_states.push_back(goalAsStart);

        position = {22.2835, 1.04566,2.94522, 3.14159, -0.00396152, 1.05489, 0.55, -0.0459061};
        //{4.07131, 0.374137, 4.82285, 3.14159, 0.909369, 0.305836, -0.248834, 2.23138 };
        state.setVariablePositions (position);
        moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
        start_states.push_back(goalAsStart);

        /*position = {3.3291, 1.04329, 4.7699, -2.17522, 0.909369, 0.0740504, -0.0170489, 1.26502};
        //{2.33859, -0.678686, 4.9503, 0.434925, 0.909369, 0.981016, -0.924015, -1.34513 };
        state.setVariablePositions (position);
        moveit::core::robotStateToRobotStateMsg(state,goalAsStart);
        start_states.push_back(goalAsStart);*/

        /*const planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model_loader.getModel()));

        kinematic_constraints::KinematicConstraintSet kset(robot_model_loader.getModel());
        planning_scene->setPlanningSceneMsg(scene_msg);
        std::vector<std::vector<double>> self_motions;
        
        int counter = 0;
        while(counter<=5)
        {
            self_motions.resize(1,std::vector<double>(state.getJointModelGroup(req.group_name)->getVariableCount(),0));
            if(state.computeSelfMotions(state.getJointModelGroup(req.group_name), self_motions, 
                boost::bind(&isIKSolutionValid,planning_scene.get(), kset.empty() ? NULL : &kset, _1, _2, _3)))
            {
                state.setJointGroupPositions(state.getJointModelGroup(req.group_name), self_motions[0]);
                ROS_ERROR_STREAM(state);
                counter ++;
                start_states.push_back(self_motions[0]);
            }
            //self_motions.clear();
            std::getchar();  
        }*/
        
        if(start_states.size()==0)
        {
             ROS_ERROR("Failed to set initial configuration of robot");
            res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
            return false;
        }
       
        m_pspace->getPlanningData()->startStatesIKComputeTime_ = (ros::Time::now()-startTime).toSec();
        ROS_INFO_STREAM("Computing IK solutions "<<(ros::Time::now()-startTime).toSec());
        
        startTime = ros::Time::now();
        if (!setMultipleStart(start_states, 0)) {
            ROS_ERROR("Failed to set initial configuration of robot");
            res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
            return false;
        }

        ROS_INFO_STREAM("Setting multiple Start "<<(ros::Time::now()-startTime).toSec());
    }
    if(lastPlanRequestId_==req.request_id)
    {   
        std::map<std::vector<int>,int> added,removed;
        added = m_grid->getAddedCells();
        removed = m_grid->getRemovedCells(); 
        int x = added.size();
        int y = removed.size();
        ROS_WARN_STREAM("Replannign with added vs. removed "<<x<<":"<<y);
        int min_added_expansion = INT_MAX;
        int min_removed_expansion = INT_MAX;
        ros::Time startTime = ros::Time::now();
        for(std::map<std::vector<int>,int>::iterator it=added.begin();it!=added.end();++it)
        {
            ROS_INFO_STREAM(" added with step "<<it->second);
            if(it->second<min_added_expansion)
            {
                ROS_INFO_STREAM("found smaller added with old step "<<min_added_expansion<<" and new "<<it->second);
                min_added_expansion = it->second;
            }
        }
        for(std::map<std::vector<int>,int>::iterator it=removed.begin();it!=removed.end();++it)
        {
            ROS_INFO_STREAM("removed with step "<<it->second);
            if(it->second<min_removed_expansion)
            {
                ROS_INFO_STREAM("found smaller removed with old step "<<min_removed_expansion<<" and new "<<it->second);
               
                min_removed_expansion = it->second;
            }
        }
        int restore_step = std::min(min_added_expansion,min_removed_expansion);

        resetCellsMarking(restore_step);
        
        ROS_INFO_STREAM("Replan overhead, marking reset "<<(ros::Time::now()-startTime).toSec());
        startTime = ros::Time::now();
        if (!replan(req.allowed_planning_time, path, restore_step)) {
            ROS_ERROR("Failed to re-plan within alotted time frame (%0.2f seconds, %d expansions)", req.allowed_planning_time, m_planner->get_n_expands());
            res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return false;
        }

        ROS_INFO_STREAM("Replan time "<<(ros::Time::now()-startTime).toSec());
    }
    else
    {
        ros::Time startTime = ros::Time::now();
        if (!plan(req.allowed_planning_time, path)) {
            ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds, %d expansions)", req.allowed_planning_time, m_planner->get_n_expands());
            res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
            return false;
        }
        ROS_INFO_STREAM("initial plan  time "<<(ros::Time::now()-startTime).toSec());
    }

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    lastPlanRequestId_ = req.request_id;
    return true; 
}

void PlannerInterface::resetCellsMarking(int restore_step)
{
    m_grid->resetCellsMarking(restore_step);
}

bool PlannerInterface::planToConfiguration(
    const moveit_msgs::MotionPlanRequest& req,
    std::vector<RobotState>& path,
    moveit_msgs::MotionPlanResponse& res)
{
    
    std::string search_name;
    std::string heuristic_name;
    std::string space_name;

    m_pspace->setMotionPlanRequestType(req.request_type);
    
    
    parsePlannerID(m_planner_id, space_name, heuristic_name, search_name);
    auto then = clock::now();
    bool result;
    //if (search_name == "trastar" || search_name == "mhtrastar")
    {
        result =  planToConfigurationWithMultipleIK(req,path,res);
        auto now = clock::now();
        ROS_INFO_STREAM("Around plan to configuration with multiple IK "<<to_seconds(now-then));
        return result;
    }


    const auto& goal_constraints_v = req.goal_constraints;
    assert(!goal_constraints_v.empty());

    // only acknowledge the first constraint
    const auto& goal_constraints = goal_constraints_v.front();

    
    if (!setGoalConfiguration(goal_constraints)) {
        ROS_ERROR("Failed to set goal position");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
        return false;
    }

    if (!setStart(req.start_state)) {
        ROS_ERROR("Failed to set initial configuration of robot");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
        return false;
    }

    if (!plan(req.allowed_planning_time, path)) {
        ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds, %d expansions)", req.allowed_planning_time, m_planner->get_n_expands());
        res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    return true;
}

/// Test if a particular set of goal constraints it supported.
///
/// This tests whether, in general, any planning algorithm supported by this
/// interface can support a particular set of constraints. Certain planning
/// algorithms may not be able to handle a given set of constraints. This
/// method also cannot check for validity of constraints against a particular
/// robot model at this step. In particular, it cannot assert that the a set
/// of joint constraints lists a constraint for each joint, which is currently
/// required.
bool PlannerInterface::SupportsGoalConstraints(
    const std::vector<moveit_msgs::Constraints>& constraints,
    std::string& why)
{
    if (constraints.empty()) {
        return true;
    }

    if (constraints.size() > 1) {
        why = "no planner currently supports more than one goal constraint";
        return false;
    }

    const moveit_msgs::Constraints& constraint = constraints.front();

    if (!constraint.visibility_constraints.empty()) {
        why = "no planner currently supports goal visibility constraints";
        return false;
    }

    // technically multiple goal position/orientation constraints can be
    // solved for if there is one position/orientation volume that entirely
    // bounds all other position/orientation volumes...ignoring for now

    if (constraint.position_constraints.size() > 1) {
        why = "no planner currently supports more than one position constraint";
        return false;
    }

    if (constraint.orientation_constraints.size() > 1) {
        why = "no planner currently supports more than one orientation constraint";
        return false;
    }

    const bool no_pose_constraint =
            constraint.position_constraints.empty() &&
            constraint.orientation_constraints.empty();
    const bool has_pose_constraint =
            constraint.position_constraints.size() == 1 &&
            constraint.orientation_constraints.size() == 1;
    const bool has_joint_constraints = !constraint.joint_constraints.empty();

    if (has_joint_constraints) {
        if (has_pose_constraint) {
            why = "no planner currently supports both pose and joint constraints";
            return false;
        }
    } else {
        if (no_pose_constraint) {
            // no constraints -> ok!
            return true;
        } else if (has_pose_constraint) {
            if (constraint.position_constraints.front().link_name !=
                constraint.orientation_constraints.front().link_name)
            {
                why = "pose constraint must be for a single link";
                return false;
            }
            return true;
        } else {
            // pose constraint is invalid
            why = "no planner supports only one position constraint or one orientation constraint";
            return false;
        }
    }

    // made it through the gauntlet
    return true;
}

bool PlannerInterface::canServiceRequest(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res) const
{
    // check for an empty start state
    // TODO: generalize this to "missing necessary state information"
    if (req.start_state.joint_state.position.empty()) {
        ROS_ERROR("No start state given. Unable to plan.");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
        return false;
    }

    // check if position & orientation constraints is empty
    const moveit_msgs::Constraints& goal_constraints =
            req.goal_constraints.front();

    if ((
            goal_constraints.position_constraints.empty() ||
            goal_constraints.orientation_constraints.empty()
        ) &&
        goal_constraints.joint_constraints.empty())
    {
        ROS_ERROR("Position or orientation constraint is empty");
        ROS_ERROR("Joint constraint is empty");
        ROS_ERROR("PlannerInterface expects a 6D pose constraint or set of joint constraints");
        return false;
    }

    // check if there is more than one goal constraint
    if (goal_constraints.position_constraints.size() > 1 ||
        goal_constraints.orientation_constraints.size() > 1)
    {
        ROS_WARN_NAMED(PI_LOGGER, "The planning request message contains %zd position and %zd orientation constraints. Currently the planner only supports one position & orientation constraint pair at a time. Planning to the first goal may not satisfy move_arm.", goal_constraints.position_constraints.size(), goal_constraints.orientation_constraints.size());
    }

    return true;
}

std::map<std::string, double> PlannerInterface::getPlannerStats()
{
    std::map<std::string, double> stats;
    stats["initial solution planning time"] = m_planner->get_initial_eps_planning_time();
    stats["initial epsilon"] = m_planner->get_initial_eps();
    stats["initial solution expansions"] = m_planner->get_n_expands_init_solution();
    stats["final epsilon planning time"] = m_planner->get_final_eps_planning_time();
    stats["final epsilon"] = m_planner->get_final_epsilon();
    stats["solution epsilon"] = m_planner->get_solution_eps();
    stats["expansions"] = m_planner->get_n_expands();
    stats["solution cost"] = m_sol_cost;
    return stats;
}

auto PlannerInterface::makePathVisualization(
    const std::vector<RobotState>& path) const
    -> std::vector<visual::Marker>
{
    std::vector<visual::Marker> ma;

    if (path.empty()) {
        return ma;
    }

    double cinc = 1.0 / double(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        auto markers = m_checker->getCollisionModelVisualization(path[i]);

        for (auto& marker : markers) {
            const float r = 0.1f;
            const float g = cinc * (float)(path.size() - (i + 1));
            const float b = cinc * (float)i;
            marker.color = visual::Color{ r, g, b, 1.0f };
        }

        for (auto& m : markers) {
            ma.push_back(std::move(m));
        }
    }

    for (size_t i = 0; i < ma.size(); ++i) {
        auto& marker = ma[i];
        marker.ns = "trajectory";
        marker.id = i;
    }

    return ma;
}

auto PlannerInterface::getBfsValuesVisualization() const -> visual::Marker
{
    if (m_heuristics.empty()) {
        return visual::Marker{ };
    }

    auto first = m_heuristics.begin();

    if (auto* hbfs = dynamic_cast<BfsHeuristic*>(first->second.get())) {
        return hbfs->getValuesVisualization();
    } else if (auto* hmfbfs = dynamic_cast<MultiFrameBfsHeuristic*>(first->second.get())) {
        return hmfbfs->getValuesVisualization();
    } else if (auto* debfs = dynamic_cast<DijkstraEgraphHeuristic3D*>(first->second.get())) {
        return debfs->getValuesVisualization();
    }  else if(auto* mhbfs = dynamic_cast<MultiBfsHeuristic*>(first->second.get())){
        return mhbfs->getValuesVisualization();
    } else {
        return visual::Marker{ };
    }
}

auto PlannerInterface::getBfsWallsVisualization() const -> visual::Marker
{
    if (m_heuristics.empty()) {
        return visual::Marker{ };
    }

    auto first = m_heuristics.begin();

    if (auto* hbfs = dynamic_cast<BfsHeuristic*>(first->second.get())) {
        return hbfs->getWallsVisualization();
    } else if (auto* hmfbfs = dynamic_cast<MultiFrameBfsHeuristic*>(first->second.get())) {
        return hmfbfs->getWallsVisualization();
    } else if(auto* mhbfs = dynamic_cast<MultiBfsHeuristic*>(first->second.get())){
        return mhbfs->getWallsVisualization();
    } else if (auto* debfs = dynamic_cast<DijkstraEgraphHeuristic3D*>(first->second.get())) {
        return debfs->getWallsVisualization();
    } else {
        return visual::Marker{ };
    }
}

bool PlannerInterface::extractGoalPoseFromGoalConstraints(
    const moveit_msgs::Constraints& constraints,
    Eigen::Affine3d& goal_pose,
    Eigen::Vector3d& offset) const
{
    if (constraints.position_constraints.empty() ||
        constraints.orientation_constraints.empty())
    {
        ROS_WARN_NAMED(PI_LOGGER, "Conversion from goal constraints to goal pose requires at least one position and one orientation constraint");
        return false;
    }

    // TODO: where is it enforced that the goal position/orientation constraints
    // should be for the planning link?
    const moveit_msgs::PositionConstraint& position_constraint = constraints.position_constraints.front();
    const moveit_msgs::OrientationConstraint& orientation_constraint = constraints.orientation_constraints.front();

    if (position_constraint.constraint_region.primitive_poses.empty()) {
        ROS_WARN_NAMED(PI_LOGGER, "Conversion from goal constraints to goal pose requires at least one primitive shape pose associated with the position constraint region");
        return false;
    }

    const shape_msgs::SolidPrimitive& bounding_primitive = position_constraint.constraint_region.primitives.front();
    const geometry_msgs::Pose& primitive_pose = position_constraint.constraint_region.primitive_poses.front();

    // undo the translation
    Eigen::Affine3d T_planning_eef = // T_planning_off * T_off_eef;
            Eigen::Translation3d(
                    primitive_pose.position.x,
                    primitive_pose.position.y,
                    primitive_pose.position.z) *
            Eigen::Quaterniond(
                    primitive_pose.orientation.w,
                    primitive_pose.orientation.x,
                    primitive_pose.orientation.y,
                    primitive_pose.orientation.z);
    Eigen::Vector3d eef_pos(T_planning_eef.translation());
    
    Eigen::Quaterniond eef_orientation;
    tf::quaternionMsgToEigen(orientation_constraint.orientation, eef_orientation);

    goal_pose = Eigen::Translation3d(eef_pos) * eef_orientation;

    tf::vectorMsgToEigen(position_constraint.target_point_offset, offset);
    return true;
}

bool PlannerInterface::extractGoalToleranceFromGoalConstraints(
    const moveit_msgs::Constraints& goal_constraints,
    double* tol)
{
    if (!goal_constraints.position_constraints.empty() &&
        !goal_constraints.position_constraints.front()
                .constraint_region.primitives.empty())
    {
        const moveit_msgs::PositionConstraint& position_constraint =
                goal_constraints.position_constraints.front();
        const shape_msgs::SolidPrimitive& constraint_primitive =
                position_constraint.constraint_region.primitives.front();
        const std::vector<double>& dims = constraint_primitive.dimensions;
        switch (constraint_primitive.type) {
        case shape_msgs::SolidPrimitive::BOX:
            tol[0] = dims[shape_msgs::SolidPrimitive::BOX_X];
            tol[1] = dims[shape_msgs::SolidPrimitive::BOX_Y];
            tol[2] = dims[shape_msgs::SolidPrimitive::BOX_Z];
            break;
        case shape_msgs::SolidPrimitive::SPHERE:
            tol[0] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            break;
        case shape_msgs::SolidPrimitive::CYLINDER:
            tol[0] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            break;
        case shape_msgs::SolidPrimitive::CONE:
            tol[0] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            break;
        }
    }
    else {
        tol[0] = tol[1] = tol[2] = 0.0;
    }

    if (!goal_constraints.orientation_constraints.empty()) {
        const std::vector<moveit_msgs::OrientationConstraint>& orientation_constraints = goal_constraints.orientation_constraints;
        const moveit_msgs::OrientationConstraint& orientation_constraint = orientation_constraints.front();
        tol[3] = orientation_constraint.absolute_x_axis_tolerance;
        tol[4] = orientation_constraint.absolute_y_axis_tolerance;
        tol[5] = orientation_constraint.absolute_z_axis_tolerance;
    }
    else {
        tol[3] = tol[4] = tol[5] = 0.0;
    }
    return true;
}

void PlannerInterface::clearMotionPlanResponse(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res) const
{
    res.trajectory_start.joint_state;
    res.trajectory_start.multi_dof_joint_state;
    res.trajectory_start.attached_collision_objects;
    res.trajectory_start.is_diff = false;
    res.group_name = req.group_name;
    res.trajectory.joint_trajectory.header.seq = 0;
    res.trajectory.joint_trajectory.header.stamp = ros::Time(0);
    res.trajectory.joint_trajectory.header.frame_id = "";
    res.trajectory.joint_trajectory.joint_names.clear();
    res.trajectory.joint_trajectory.points.clear();
    res.trajectory.multi_dof_joint_trajectory.header.seq = 0;
    res.trajectory.multi_dof_joint_trajectory.header.stamp = ros::Time(0);
    res.trajectory.multi_dof_joint_trajectory.header.frame_id = "";
    res.trajectory.multi_dof_joint_trajectory.joint_names.clear();
    res.trajectory.multi_dof_joint_trajectory.points.clear();
    res.planning_time = 0.0;
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
}

bool PlannerInterface::parsePlannerID(
    const std::string& planner_id,
    std::string& space_name,
    std::string& heuristic_name,
    std::string& search_name) const
{
    boost::regex alg_regex("(\\w+)(?:\\.(\\w+))?(?:\\.(\\w+))?");

    boost::smatch sm;

    ROS_INFO("Match planner id '%s' against regex '%s'", planner_id.c_str(), alg_regex.str().c_str());
    if (!boost::regex_match(planner_id, sm, alg_regex)) {
        return false;
    }

    const std::string default_search_name = "arastar";
    const std::string default_heuristic_name = "bfs";
    const std::string default_space_name = "manip";

    if (sm.size() < 2 || sm[1].str().empty()) {
        search_name = default_search_name;
    } else {
        search_name = sm[1];
    }

    if (sm.size() < 3 || sm[2].str().empty()) {
        heuristic_name = default_heuristic_name;
    } else {
        heuristic_name = sm[2];
    }

    if (sm.size() < 4 || sm[3].str().empty()) {
        space_name = default_space_name;
    } else {
        space_name = sm[3];
    }

    return true;
}

void PlannerInterface::clearGraphStateToPlannerStateMap()
{
    if (!m_pspace) {
        return;
    }

    std::vector<int*>& state_id_to_index = m_pspace->StateID2IndexMapping;
    for (int* mapping : state_id_to_index) {
        for (int i = 0; i < NUMOFINDICES_STATEID2IND; ++i) {
            mapping[i] = -1;
        }
    }
}

bool PlannerInterface::reinitPlanner(const std::string& planner_id)
{
    

    std::string search_name;
    std::string heuristic_name;
    std::string space_name;

    if (planner_id == m_planner_id) {
        // TODO: check for specification of default planning components when
        // they may not have been previously specified
        // initialize heuristics
        parsePlannerID(planner_id, space_name, heuristic_name, search_name);
        /*auto psait = m_space_factories.find(space_name);
        if (psait == m_space_factories.end()) {
            ROS_ERROR("Unrecognized planning space name '%s'", space_name.c_str());
            return false;
        }

        m_pspace = psait->second(m_robot, m_checker, &m_params);
        if (!m_pspace) {
            ROS_ERROR("Failed to build planning space '%s'", space_name.c_str());
            return false;
        }
*/
        m_pspace->eraseHeuristic(begin(m_heuristics)->second.get());
        auto hait = m_heuristic_factories.find(heuristic_name);
        if (hait == m_heuristic_factories.end()) {
            ROS_ERROR("Unrecognized heuristic name '%s'", heuristic_name.c_str());
            return false;
        }

        auto heuristic = hait->second(m_pspace.get());
        if (!heuristic) {
            ROS_ERROR("Failed to build heuristic '%s'", heuristic_name.c_str());
            return false;
        }

        // initialize heuristics
        m_heuristics.clear();
        m_heuristics.insert(std::make_pair(heuristic_name, std::move(heuristic)));

        for (const auto& entry : m_heuristics) {
            m_pspace->insertHeuristic(entry.second.get());
        }

        if(search_name=="trastar") {
            (dynamic_cast<TRAStar*> (m_planner.get()))->updateHeuristic(begin(m_heuristics)->second.get());
        }
        else if(search_name=="mhtrastar")
        {
            (dynamic_cast<MHTRAStar*> (m_planner.get()))->updateHeuristic(begin(m_heuristics)->second.get());

        }
        return true;
    }

    ROS_INFO_NAMED(PI_LOGGER, "Initialize planner");

    
    if (!parsePlannerID(planner_id, space_name, heuristic_name, search_name)) {
        ROS_ERROR("Failed to parse planner setup");
        return false;
    }
    
    ROS_INFO_NAMED(PI_LOGGER, " -> Planning Space: %s", space_name.c_str());
    ROS_INFO_NAMED(PI_LOGGER, " -> Heuristic: %s", heuristic_name.c_str());
    ROS_INFO_NAMED(PI_LOGGER, " -> Search: %s", search_name.c_str());

    auto psait = m_space_factories.find(space_name);
    if (psait == m_space_factories.end()) {
        ROS_ERROR("Unrecognized planning space name '%s'", space_name.c_str());
        return false;
    }

    m_pspace = psait->second(m_robot, m_checker, &m_params);
    if (!m_pspace) {
        ROS_ERROR("Failed to build planning space '%s'", space_name.c_str());
        return false;
    }

    auto hait = m_heuristic_factories.find(heuristic_name);
    if (hait == m_heuristic_factories.end()) {
        ROS_ERROR("Unrecognized heuristic name '%s'", heuristic_name.c_str());
        return false;
    }

    auto heuristic = hait->second(m_pspace.get());
    if (!heuristic) {
        ROS_ERROR("Failed to build heuristic '%s'", heuristic_name.c_str());
        return false;
    }

    // initialize heuristics
    m_heuristics.clear();
    m_heuristics.insert(std::make_pair(heuristic_name, std::move(heuristic)));

    for (const auto& entry : m_heuristics) {
        m_pspace->insertHeuristic(entry.second.get());
    }

    auto pait = m_planner_factories.find(search_name);
    if (pait == m_planner_factories.end()) {
        ROS_ERROR("Unrecognized search name '%s'", search_name.c_str());
        return false;
    }

    auto first_heuristic = begin(m_heuristics);
    m_planner = pait->second(m_pspace.get(), first_heuristic->second.get());
    if (!m_planner) {
        ROS_ERROR("Failed to build planner '%s'", search_name.c_str());
        return false;
    }
    m_planner_id = planner_id;
    return true;
}

void PlannerInterface::convertPlanningDataToMsg(moveit_msgs::PlanningData& planning_data_msg)
{
    planning_data_msg.start_IK_compute_time = m_pspace->getPlanningData()->startStatesIKComputeTime_;
    planning_data_msg.restoration_time = m_pspace->getPlanningData()->restorationTime_;
    planning_data_msg.search_time = m_pspace->getPlanningData()->searchTime_;
    planning_data_msg.path_extraction_time = m_pspace->getPlanningData()->pathExtractionTime_;
    planning_data_msg.shortcut_time = m_pspace->getPlanningData()->shortcutTime_;
    planning_data_msg.interpolation_time = m_pspace->getPlanningData()->interpolateTime_;
    planning_data_msg.planning_time = m_pspace->getPlanningData()->planningTime_;
    planning_data_msg.num_expansions = m_pspace->getPlanningData()->numExpansions_;
    planning_data_msg.epsilon = m_pspace->getPlanningData()->finalEpsilon_;
    planning_data_msg.cost = m_pspace->getPlanningData()->cost_;
    planning_data_msg.restoration_step =  m_pspace->getPlanningData()->restorationStep_;
    planning_data_msg.path_num_states = m_pspace->getPlanningData()->pathNumStates_;
    planning_data_msg.original_waypoints = m_pspace->getPlanningData()->originalWaypoints_;
    planning_data_msg.interpolation_waypoints = m_pspace->getPlanningData()->interpolatedWaypoints_;
    planning_data_msg.shortcut_waypoints = m_pspace->getPlanningData()->shortcutWaypoints_;
    //planning_data_msg.path_state_ids.resize(m_pspace->getPlanningData()->path_state_ids.size());
    //std::copy(m_pspace->getPlanningData()->path_state_ids.begin(), m_pspace->getPlanningData()->path_state_ids.end(),planning_data_msg.path_state_ids.begin());
    for(int i=0;i<m_pspace->getPlanningData()->actionStates_.size();i++)
    {
        moveit_msgs::ActionState state;
        state.state_id = m_pspace->getPlanningData()->actionStates_[i].state_id;
        state.expansion_step = i+1;//m_pspace->getPlanningData()->actionStates_[i].expansion_step;
        state.parent_state_id = m_pspace->getPlanningData()->actionStates_[i].parent_id;
        state.source_group = m_pspace->getPlanningData()->actionStates_[i].source;
        state.marking_cell_time = m_pspace->getPlanningData()->actionStates_[i].marking_cell_time;
        state.dist_obst_time = m_pspace->getPlanningData()->actionStates_[i].dist_collision_time;
        state.dist_obstacles = m_pspace->getPlanningData()->actionStates_[i].dist_obstacles;
        state.dist_to_goal = m_pspace->getPlanningData()->actionStates_[i].dist_to_goal;
        state.state_config = m_pspace->getPlanningData()->actionStates_[i].state_config;
        state.parent_state_config = m_pspace->getPlanningData()->actionStates_[i].parent_state_config;
        state.g = m_pspace->getPlanningData()->actionStates_[i].g;
        state.h = m_pspace->getPlanningData()->actionStates_[i].h;

        planning_data_msg.action_states.push_back(state);
    }
    for(int i=0;i<m_pspace->getPlanningData()->solutionStates_.size();i++)
    {
        moveit_msgs::SolutionState state;
        state.state_id = m_pspace->getPlanningData()->solutionStates_[i].state_id;
        state.state_config = m_pspace->getPlanningData()->solutionStates_[i].state_config;
        planning_data_msg.solution_states.push_back(state);
    }

}

void PlannerInterface::profilePath(trajectory_msgs::JointTrajectory& traj) const
{
    if (traj.points.empty()) {
        return;
    }

    auto& joint_names = traj.joint_names;

    for (size_t i = 1; i < traj.points.size(); ++i) {
        auto& prev_point = traj.points[i - 1];
        auto& curr_point = traj.points[i];

        // find the maximum time required for any joint to reach the next
        // waypoint
        double max_time = 0.0;
        for (size_t jidx = 0; jidx < joint_names.size(); ++jidx) {
            const double from_pos = prev_point.positions[jidx];
            const double to_pos = curr_point.positions[jidx];
            const double vel = m_robot->velLimit(jidx);
            if (vel <= 0.0) {
                continue;
            }
            double t = 0.0;
            if (m_robot->isContinuous(jidx)) {
                t = angles::shortest_angle_dist(from_pos, to_pos) / vel;
            } else {
                t = fabs(to_pos - from_pos) / vel;
            }

            max_time = std::max(max_time, t);
        }

        curr_point.time_from_start = prev_point.time_from_start + ros::Duration(max_time);
    }
}

void PlannerInterface::removeZeroDurationSegments(
    trajectory_msgs::JointTrajectory& traj) const
{
    if (traj.points.empty()) {
        return;
    }

    // filter out any duplicate points
    // TODO: find out where these are happening
    size_t end_idx = 1; // current end of the non-filtered range
    for (size_t i = 1; i < traj.points.size(); ++i) {
        auto& prev = traj.points[end_idx - 1];
        auto& curr = traj.points[i];
        if (curr.time_from_start != prev.time_from_start) {
            ROS_INFO("Move index %zu into %zu", i, end_idx);
            if (end_idx != i) {
                traj.points[end_idx] = std::move(curr);
            }
            end_idx++;
        }
    }
    traj.points.resize(end_idx);
}

bool PlannerInterface::isPathValid(const std::vector<RobotState>& path) const
{
    for (size_t i = 1; i < path.size(); ++i) {
        if (!m_checker->isStateToStateValid(path[i - 1], path[i])) {
            ROS_ERROR_STREAM("path between " << path[i - 1] << " and " << path[i] << " is invalid (" << i - 1 << " -> " << i << ")");
            return false;
        }
    }
    return true;
}

void PlannerInterface::postProcessPath(std::vector<RobotState>& path) const
{
    const bool check_planned_path = true;
    if (check_planned_path && !isPathValid(path)) {
        ROS_ERROR("Planned path is invalid");
    }
    m_pspace->getPlanningData()->originalWaypoints_ = path.size();
    auto then = clock::now();
    // shortcut path
    if (m_params.shortcut_path) {
        if (!InterpolatePath(*m_checker, path)) {
            ROS_WARN_NAMED(PI_LOGGER, "Failed to interpolate planned path with %zu waypoints before shortcutting.", path.size());
            std::vector<RobotState> ipath = path;
            path.clear();
            ShortcutPath(m_robot, m_checker, ipath, path, m_params.shortcut_type);

   
        } else {
            std::vector<RobotState> ipath = path;
            path.clear();
            ShortcutPath(m_robot, m_checker, ipath, path, m_params.shortcut_type);
        }
    }

    auto now = clock::now();
    ROS_INFO_STREAM("just after shortcut the time is "<<to_seconds(now - then));
    m_pspace->getPlanningData()->shortcutTime_ = to_seconds(now - then);
    m_pspace->getPlanningData()->shortcutWaypoints_ = path.size();
    
    then = clock::now();
    
    // interpolate path
    if (m_params.interpolate_path) {
        if (!InterpolatePath(*m_checker, path)) {
            ROS_WARN_NAMED(PI_LOGGER, "Failed to interpolate trajectory");
        }
    }

    now = clock::now();
    m_pspace->getPlanningData()->interpolateTime_ = to_seconds(now - then);
    m_pspace->getPlanningData()->interpolatedWaypoints_ = path.size();
    ROS_INFO_STREAM("just after interpolation the time is "<<to_seconds(now - then));
}

void PlannerInterface::convertJointVariablePathToJointTrajectory(
    const std::vector<RobotState>& path,
    trajectory_msgs::JointTrajectory& traj) const
{
    traj.header.frame_id = m_params.planning_frame;
    traj.joint_names = m_robot->getPlanningJoints();
    traj.points.clear();
    traj.points.reserve(path.size());
    for (const auto& point : path) {
        trajectory_msgs::JointTrajectoryPoint traj_pt;
        traj_pt.positions = point;
        traj.points.push_back(std::move(traj_pt));
    }
}

bool PlannerInterface::writePath(
    const moveit_msgs::RobotState& ref,
    const moveit_msgs::RobotTrajectory& traj) const
{
    boost::filesystem::path p(m_params.plan_output_dir);

    try {
        if (!boost::filesystem::exists(p)) {
            ROS_INFO("Create plan output directory %s", p.native().c_str());
            boost::filesystem::create_directory(p);
        }

        if (!boost::filesystem::is_directory(p)) {
            ROS_ERROR("Failed to log path. %s is not a directory", m_params.plan_output_dir.c_str());
            return false;
        }
    } catch (const boost::filesystem::filesystem_error& ex) {
        ROS_ERROR("Failed to create plan output directory %s", p.native().c_str());
        return false;
    }

    std::stringstream ss_filename;
    auto now = clock::now();
    ss_filename << "path_" << now.time_since_epoch().count();
    p /= ss_filename.str();

    std::ofstream ofs(p.native());
    if (!ofs.is_open()) {
        return false;
    }

    ROS_INFO("Log path to %s", p.native().c_str());

    // write header
    for (size_t vidx = 0; vidx < m_robot->jointVariableCount(); ++vidx) {
        const std::string& var_name = m_robot->getPlanningJoints()[vidx];
        ofs << var_name; // TODO: sanitize variable name for csv?
        if (vidx != m_robot->jointVariableCount() - 1) {
            ofs << ',';
        }
    }
    ofs << '\n';

    const size_t wp_count = std::max(
            traj.joint_trajectory.points.size(),
            traj.multi_dof_joint_trajectory.points.size());
    for (size_t widx = 0; widx < wp_count; ++widx) {
        // fill the complete robot state
        moveit_msgs::RobotState state = ref;

        if (widx < traj.joint_trajectory.points.size()) {
            const trajectory_msgs::JointTrajectoryPoint& wp =
                    traj.joint_trajectory.points[widx];
            const size_t joint_count = traj.joint_trajectory.joint_names.size();
            for (size_t jidx = 0; jidx < joint_count; ++jidx) {
                const std::string& joint_name =
                        traj.joint_trajectory.joint_names[jidx];
                double vp = wp.positions[jidx];
                auto it = std::find(
                        state.joint_state.name.begin(),
                        state.joint_state.name.end(),
                        joint_name);
                if (it != state.joint_state.name.end()) {
                    size_t tvidx = std::distance(state.joint_state.name.begin(), it);
                    state.joint_state.position[tvidx] = vp;
                }
            }
        }
        if (widx < traj.multi_dof_joint_trajectory.points.size()) {
            const trajectory_msgs::MultiDOFJointTrajectoryPoint& wp =
                    traj.multi_dof_joint_trajectory.points[widx];
            const size_t joint_count = traj.multi_dof_joint_trajectory.joint_names.size();
            for (size_t jidx = 0; jidx < joint_count; ++jidx) {
                const std::string& joint_name =
                        traj.multi_dof_joint_trajectory.joint_names[jidx];
                const geometry_msgs::Transform& t = wp.transforms[jidx];
                auto it = std::find(
                        state.multi_dof_joint_state.joint_names.begin(),
                        state.multi_dof_joint_state.joint_names.end(),
                        joint_name);
                if (it != state.multi_dof_joint_state.joint_names.end()) {
                    size_t tvidx = std::distance(state.multi_dof_joint_state.joint_names.begin(), it);
                    state.multi_dof_joint_state.transforms[tvidx] = t;
                }
            }
        }

        // write the planning variables out to file
        for (size_t vidx = 0; vidx < m_robot->jointVariableCount(); ++vidx) {
            const std::string& var_name = m_robot->getPlanningJoints()[vidx];
            const bool var_is_mdof = false; // TODO: multi-dof joints in robot model
            if (var_is_mdof) {

            } else {
                auto it = std::find(
                        state.joint_state.name.begin(),
                        state.joint_state.name.end(),
                        var_name);
                if (it != state.joint_state.name.end()) {
                    size_t tvidx = std::distance(state.joint_state.name.begin(), it);
                    double vp = state.joint_state.position[tvidx];
                    ofs << vp;
                    if (vidx != m_robot->jointVariableCount() - 1) {
                        ofs << ',';
                    }
                }
            }
        }
        ofs << '\n';
    }

    return true;
}

} // namespace motion
} // namespace sbpl
