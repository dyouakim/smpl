////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Benjamin Cohen, Andrew Dornbush
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

#include <smpl/post_processing.h>

// standard includes
#include <chrono>
#include <numeric>

// system includes
#include <Eigen/Dense>

// project includes
#include <smpl/angles.h>
#include <smpl/time.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/geometry/shortcut.h>

namespace sbpl {
namespace motion {

double distance(
    const RobotModel& robot,
    const RobotState& from,
    const RobotState& to)
{
    double dist = 0.0;
    for (size_t vidx = 0; vidx < robot.getPlanningJoints().size(); ++vidx) {
        if (!robot.hasPosLimit(vidx)) {
            dist += angles::shortest_angle_dist(to[vidx], from[vidx]);
        }
        else {
            dist += fabs(to[vidx] - from[vidx]);
        }
    }
    return dist;
}

double pv_distance(
    const RobotModel& robot,
    const RobotState& from,
    const RobotState& to)
{
    auto sqrd = [](double d) { return d * d; };
    const double pweight = 1.0;
    const double vweight = 1.0;

    const size_t var_count = robot.getPlanningJoints().size();

    double dist = 0.0;
    for (size_t vidx = 0; vidx < var_count; ++vidx) {
        if (!robot.hasPosLimit(vidx)) {
            dist += angles::shortest_angle_dist(to[vidx], from[vidx]);
        }
        else {
            dist += fabs(to[vidx] - from[vidx]);
        }
    }

    double vdist = 0.0;
    for (size_t vidx = 0; vidx < var_count; ++vidx) {
        vdist += fabs(to[var_count + vidx] - from[var_count + vidx]);
    }

    return pweight * dist + vweight * vdist;
}

class JointPositionShortcutPathGenerator
{
public:

    JointPositionShortcutPathGenerator(RobotModel* rm, CollisionChecker* cc) :
        m_robot(rm),
        m_cc(cc)
    { }

    template <typename OutputIt>
    bool operator()(
        const RobotState& start, const RobotState& finish,
        OutputIt ofirst, double& cost) const
    {
        const size_t var_count = m_robot->getPlanningJoints().size();
        if (m_cc->isStateToStateValid(start, finish)) {
            *ofirst++ = start;
            *ofirst++ = finish;
            cost = distance(*m_robot, start, finish);
            return true;
        }
        else {
            return false;
        }
    }

private:

    RobotModel* m_robot;
    CollisionChecker* m_cc;
};

class JointPositionVelocityShortcutPathGenerator
{
public:

    JointPositionVelocityShortcutPathGenerator(
        RobotModel* rm,
        CollisionChecker* cc)
    :
        m_robot(rm),
        m_cc(cc)
    { }

    template <typename OutputIt>
    bool operator()(
        const RobotState& start, const RobotState& finish,
        OutputIt ofirst, double& cost) const
    {
        const size_t var_count = m_robot->getPlanningJoints().size();
        const RobotState pstart(start.begin(), start.begin() + var_count);
        const RobotState pend(finish.begin(), finish.begin() + var_count);
        if (m_cc->isStateToStateValid(pstart, pend)) {
            *ofirst++ = start;
            *ofirst++ = finish;
            cost = pv_distance(*m_robot, start, finish);
            return true;
        }
        else {
            return false;
        }
    }

private:

    RobotModel* m_robot;
    CollisionChecker* m_cc;
};

class EuclidShortcutPathGenerator
{
public:

    EuclidShortcutPathGenerator(RobotModel* rm, CollisionChecker* cc) :
        m_rm(rm),
        m_cc(cc),
        m_fk_iface(nullptr),
        m_ik_iface(nullptr)
    {
        m_fk_iface = rm->getExtension<ForwardKinematicsInterface>();
        m_ik_iface = rm->getExtension<InverseKinematicsInterface>();
    }

    template <typename OutputIt>
    bool operator()(
        const RobotState& start, const RobotState& end,
        OutputIt ofirst, double& cost) const
    {
        if (!m_fk_iface || !m_ik_iface) {
            return false;
        }

        // compute forward kinematics for the start an end configurations
        std::vector<double> from_pose, to_pose;
        if (!m_fk_iface->computePlanningLinkFK(start, from_pose) ||
            !m_fk_iface->computePlanningLinkFK(end, to_pose))
        {
            return false;
        }

        Eigen::Vector3d pstart(from_pose[0], from_pose[1], from_pose[2]);
        Eigen::Vector3d pend(to_pose[0], to_pose[1], to_pose[2]);

        Eigen::Quaterniond qstart(
                Eigen::AngleAxisd(from_pose[5], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(from_pose[4], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(from_pose[3], Eigen::Vector3d::UnitX()));
        Eigen::Quaterniond qend(
                Eigen::AngleAxisd(to_pose[5], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(to_pose[4], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(to_pose[3], Eigen::Vector3d::UnitX()));

        if (qstart.dot(qend) < 0.0) {
            // negate one end of the quaternion path to ensure interpolation
            // takes the short arc
            qend = Eigen::Quaterniond(-qend.w(), -qend.x(), -qend.y(), -qend.z());
        }

        // compute the number of path waypoints
        double posdiff = (pend - pstart).norm();
        double rotdiff = angles::normalize_angle(2.0 * acos(qstart.dot(qend)));

        const double interp_pres = 0.01;
        const double interp_rres = angles::to_radians(5.0);

        int num_points = 2;
        num_points = std::max(num_points, (int)ceil(posdiff / interp_pres));
        num_points = std::max(num_points, (int)ceil(rotdiff / interp_rres));

        std::vector<RobotState> cpath;

        cpath.push_back(start);
        double dist = 0.0;
        for (int i = 1; i < num_points; ++i) {
            // compute the intermediate pose
            double alpha = (double)i / (double)(num_points - 1);
            Eigen::Vector3d ppos = (1.0 - alpha) * pstart + alpha * pend;
            Eigen::Quaterniond prot = qstart.slerp(alpha, qend);

            const Eigen::Affine3d ptrans = prot * Eigen::Translation3d(ppos);
            Eigen::Vector3d rpy = ptrans.rotation().eulerAngles(2, 1, 0);

            std::vector<double> ipose(6, 0.0);
            ipose[0] = ppos.x();
            ipose[1] = ppos.y();
            ipose[2] = ppos.z();
            ipose[3] = rpy[2];
            ipose[4] = rpy[1];
            ipose[5] = rpy[0];

            // run inverse kinematics with the previous pose as the seed state
            const RobotState& prev_wp = cpath.back();
            RobotState wp(m_rm->getPlanningJoints().size(), 0.0);
            if (!m_ik_iface->computeIK(ipose, prev_wp, wp)) {
                return false;
            }

            // check the path segment for collisions
            if (!m_cc->isStateToStateValid(prev_wp, wp)) {
                return false;
            }

            dist += distance(*m_rm, prev_wp, wp);

            cpath.push_back(wp);
        }

        for (auto& point : cpath) {
            *ofirst++ = std::move(point);
        }
        cost = dist;
        return true;
    }

private:

    RobotModel* m_rm;
    CollisionChecker* m_cc;

    ForwardKinematicsInterface* m_fk_iface;
    InverseKinematicsInterface* m_ik_iface;
};

void ShortcutPath(
    RobotModel* rm,
    CollisionChecker* cc,
    const std::vector<RobotState>& pin,
    std::vector<RobotState>& pout,
    ShortcutType type)
{
    if (pin.size() < 2) {
        pout = pin;
        return;
    }

    auto then = clock::now();
    double prev_cost = 0.0, next_cost = 0.0;
    switch (type) {
    case ShortcutType::JOINT_SPACE:
    {
        std::vector<double> costs;
        ComputePositionPathCosts(rm, pin, costs);
        JointPositionShortcutPathGenerator generators[] =
        {
            JointPositionShortcutPathGenerator(rm, cc)
        };
        shortcut::ShortcutPath(
                pin.begin(), pin.end(),
                costs.begin(), costs.end(),
                generators, generators + 1,
                std::back_inserter(pout));
    }   break;
    case ShortcutType::EUCLID_SPACE:
    {
        std::vector<double> costs;
        ComputePositionPathCosts(rm, pin, costs);

        EuclidShortcutPathGenerator generators[] =
        {
            EuclidShortcutPathGenerator(rm, cc)
        };

        shortcut::ShortcutPath(
                pin.begin(), pin.end(),
                costs.begin(), costs.end(),
                generators, generators + 1,
                std::back_inserter(pout));
    }   break;
    case ShortcutType::JOINT_POSITION_VELOCITY_SPACE:
    {
        std::vector<RobotState> pv_path;
        CreatePositionVelocityPath(rm, pin, pv_path);

        std::vector<double> costs;
        ComputePositionVelocityPathCosts(rm, pv_path, costs);
        prev_cost = std::accumulate(costs.begin(), costs.end(), 0.0);

        JointPositionVelocityShortcutPathGenerator generators[] =
        {
            JointPositionVelocityShortcutPathGenerator(rm, cc)
        };

        std::vector<RobotState> opvpath;
        shortcut::ShortcutPath(
                pv_path.begin(), pv_path.end(),
                costs.begin(), costs.end(),
                generators, generators + 1,
                std::back_inserter(opvpath));

        std::vector<RobotState> opvpath_dnc;
        shortcut::DivideAndConquerShortcutPath(
                pv_path.begin(), pv_path.end(),
                costs.begin(), costs.end(),
                generators, generators + 1,
                std::back_inserter(opvpath_dnc));

        std::vector<double> new_costs;
        ComputePositionVelocityPathCosts(rm, opvpath, new_costs);
        next_cost = std::accumulate(new_costs.begin(), new_costs.end(), 0.0);

        std::vector<double> new_costs_dnc;
        ComputePositionVelocityPathCosts(rm, opvpath_dnc, new_costs_dnc);
        double new_cost_dnc = std::accumulate(new_costs_dnc.begin(), new_costs_dnc.end(), 0.0);
        if (new_cost_dnc < next_cost) {
            SMPL_INFO("Divide and Conquer Wins! (%0.3f < %0.3f)", new_cost_dnc, next_cost);
            next_cost = new_cost_dnc;
            opvpath = std::move(opvpath_dnc);
        }

        ExtractPositionPath(rm, opvpath, pout);
    }   break;
    default:
        break;
    }

    auto now = clock::now();
    SMPL_INFO("Path shortcutting took %0.3f seconds", std::chrono::duration<double>(now - then).count());

    SMPL_INFO("Original path: waypoint count: %zu, cost: %0.3f", pin.size(), prev_cost);
    SMPL_INFO("Shortcutted path: waypount_count: %zu, cost: %0.3f", pout.size(), next_cost);
}

bool CreatePositionVelocityPath(
    RobotModel* rm,
    const std::vector<RobotState>& path,
    std::vector<RobotState>& opath)
{
    const size_t var_count = rm->getPlanningJoints().size();

    // position + zero velocity for the first point
    std::vector<RobotState> pv_path(path.size());
    if (!pv_path.empty()) {
        pv_path[0].resize(2 * var_count);
        // p_0 = p_in_0, v_0 = { 0, ..., 0 }
        std::copy(path[0].begin(), path[0].begin() + var_count, pv_path[0].begin());
        std::generate(pv_path[0].begin() + var_count, pv_path[0].end(), []() { return 0.0; });
    }

    // position + velocities for the remaining points
    for (size_t i = 1; i < path.size(); ++i) {
        const RobotState& from = path[i - 1];
        const RobotState& to = path[i];
        pv_path[i].resize(2 * var_count);
        std::copy(to.begin(), to.begin() + var_count, pv_path[i].begin());
        for (size_t vidx = 0; vidx < var_count; ++vidx) {
            if (!rm->hasPosLimit(vidx)) {
                pv_path[i][var_count + vidx] = std::copysign(1.0, angles::shortest_angle_diff(to[vidx], from[vidx]));
            }
            else {
                pv_path[i][var_count + vidx] = std::copysign(1.0, to[vidx] - from[vidx]);
            }
        }
    }

    opath = std::move(pv_path);
    return true;
}

bool ExtractPositionPath(
    RobotModel* rm,
    const std::vector<RobotState>& pv_path,
    std::vector<RobotState>& path)
{
    path.resize(pv_path.size());
    const size_t var_count = rm->getPlanningJoints().size();
    for (size_t pidx = 0; pidx < pv_path.size(); ++pidx) {
        path[pidx].resize(var_count);
        std::copy(&pv_path[pidx][0], &pv_path[pidx][0] + var_count, &path[pidx][0]);
    }
    return true;
}

bool ComputePositionPathCosts(
    RobotModel* rm,
    const std::vector<RobotState>& path,
    std::vector<double>& costs)
{
    if (path.empty()) {
        return false;
    }

    costs.clear();
    costs.resize(path.size() - 1);
    for (size_t i = 1; i < path.size(); ++i) {
        costs[i - 1] = distance(*rm, path[i - 1], path[i]);
    }
    return true;
}

bool ComputePositionVelocityPathCosts(
    RobotModel* rm,
    const std::vector<RobotState>& pv_path,
    std::vector<double>& costs)
{
    if (pv_path.empty()) {
        return false;
    }

    costs.clear();
    costs.resize(pv_path.size() - 1);
    for (size_t i = 1; i < pv_path.size(); ++i) {
        costs[i - 1] = pv_distance(*rm, pv_path[i - 1], pv_path[i]);
    }
    return true;
}

bool InterpolatePath(CollisionChecker& cc, std::vector<RobotState>& path)
{
    if (path.empty()) {
        return true;
    }

    const size_t num_joints = path.front().size();
    for (const auto& pt : path) {
        if (pt.size() != num_joints) {
            SMPL_ERROR("Failed to interpolate trajectory. Input trajectory is malformed");
            return false;
        }
    }

    std::vector<RobotState> opath;

    // tack on the first point of the trajectory
    opath.push_back(path.front());

    // iterate over path segments
    for (size_t i = 0; i < path.size() - 1; ++i) {
        const RobotState& start = path[i];
        const RobotState& end = path[i + 1];

        SMPL_DEBUG_STREAM("Interpolating between " << start << " and " << end);

        std::vector<RobotState> ipath;
        if (!cc.interpolatePath(start, end, ipath)) {
            SMPL_ERROR("Failed to interpolate between waypoint %zu and %zu because it's infeasible given the limits.", i, i + 1);
            return false;
        }

        // check the interpolated path for collisions, as the interpolator may
        // take a slightly different
        bool collision = false;
        for (const auto& point : ipath) {
            if (!cc.isStateValid(point, false)) {
                collision = true;
                break;
            }
        }

        if (collision) {
            SMPL_ERROR("Interpolated path collides. Resorting to original waypoints");
            opath.push_back(end);
            continue;
        }

        if (!ipath.empty()) {
            // concatenate current path and the intermediate path (we already
            // have the first waypoint in the path from last iteration)
            opath.insert(opath.end(), std::next(ipath.begin()), ipath.end());
        }

        SMPL_DEBUG("[%zu] path length: %zu", i, opath.size());
    }

    SMPL_INFO("Original path length: %zu   Interpolated path length: %zu", path.size(), opath.size());
    path = std::move(opath);
    return true;
}

} // namespace motion
} // namespace sbpl
