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

#include <smpl/heuristic/bfs_heuristic.h>

// project includes
#include <smpl/bfs3d/bfs3d.h>
#include <smpl/console/console.h>
#include <smpl/intrusive_heap.h>
#include <smpl/grid.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>

namespace sbpl {
namespace motion {

static const char* LOG = "heuristic.bfs";

BfsHeuristic::~BfsHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool BfsHeuristic::init(RobotPlanningSpace* space, const OccupancyGrid* grid)
{
    if (!grid) {
        return false;
    }

    if (!RobotHeuristic::init(space)) {
        return false;
    }

    m_grid = grid;

    m_pp = space->getExtension<PointProjectionExtension>();
    if (m_pp) {
        SMPL_INFO_NAMED(LOG, "Got Point Projection Extension!");
    }
    syncGridAndBfs();

    return true;
}

void BfsHeuristic::setInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

void BfsHeuristic::setCostPerCell(int cost_per_cell)
{
    m_cost_per_cell = cost_per_cell;
}

void BfsHeuristic::updateGoal(const GoalConstraint& goal)
{
    int gx, gy, gz;
    grid()->worldToGrid(
            goal.tgt_off_pose[0], goal.tgt_off_pose[1], goal.tgt_off_pose[2],
            gx, gy, gz);

    SMPL_DEBUG_NAMED(LOG, "Setting the BFS heuristic goal (%d, %d, %d)", gx, gy, gz);

    if (!m_bfs->inBounds(gx, gy, gz)) {
        SMPL_ERROR_NAMED(LOG, "Heuristic goal is out of BFS bounds");
    }

    m_goal_x = gx;
    m_goal_y = gy;
    m_goal_z = gz;

    m_bfs->run(gx, gy, gz);
}

double BfsHeuristic::getMetricStartDistance(double x, double y, double z)
{
    int start_id = planningSpace()->getStartStateID();

    if (!m_pp) {
        return 0.0;
    }

    Eigen::Vector3d p;
    if (!m_pp->projectToPoint(planningSpace()->getStartStateID(), p)) {
        return 0.0;
    }

    int sx, sy, sz;
    grid()->worldToGrid(p.x(), p.y(), p.z(), sx, sy, sz);

    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);

    // compute the manhattan distance to the start cell
    const int dx = sx - gx;
    const int dy = sy - gy;
    const int dz = sz - gz;
    return grid()->resolution() * (abs(dx) + abs(dy) + abs(dz));
}

double BfsHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);
    if (!m_bfs->inBounds(gx, gy, gz)) {
        return (double)BFS_3D::WALL * grid()->resolution();
    } else {
        return (double)m_bfs->getDistance(gx, gy, gz) * grid()->resolution();
    }
}

Extension* BfsHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

int BfsHeuristic::GetGoalHeuristic(int state_id)
{
    if (!m_pp) {
        return 0;
    }

    Eigen::Vector3d p;
    if (!m_pp->projectToPoint(state_id, p)) {
        return 0;
    }

    Eigen::Vector3i dp;
    grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());

    return getBfsCostToGoal(*m_bfs, dp.x(), dp.y(), dp.z());
}

int BfsHeuristic::GetStartHeuristic(int state_id)
{
    SMPL_WARN_ONCE("BfsHeuristic::GetStartHeuristic unimplemented");
    return 0;
}

int BfsHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    if (to_id == planningSpace()->getGoalStateID()) {
        return GetGoalHeuristic(from_id);
    }
    else {
        SMPL_WARN_ONCE("BfsHeuristic::GetFromToHeuristic unimplemented for arbitrary state pair");
        return 0;
    }
}

auto BfsHeuristic::getWallsVisualization() const -> visual::Marker
{
    std::vector<Eigen::Vector3d> centers;
    int dimX = grid()->numCellsX();
    int dimY = grid()->numCellsY();
    int dimZ = grid()->numCellsZ();
    for (int x = 0; x < dimX; x++) {
    for (int y = 0; y < dimY; y++) {
    for (int z = 0; z < dimZ; z++) {
        if (m_bfs->isWall(x, y, z)) {
            Eigen::Vector3d p;
            grid()->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            centers.push_back(p);
        }
    }
    }
    }

    SMPL_DEBUG_NAMED(LOG, "BFS Visualization contains %zu points", centers.size());

    visual::Color color;
    color.r = 100.0f / 255.0f;
    color.g = 149.0f / 255.0f;
    color.b = 238.0f / 255.0f;
    color.a = 1.0f;

    return visual::MakeCubesMarker(
            centers,
            grid()->resolution(),
            color,
            grid()->getReferenceFrame(),
            "bfs_walls");
}

auto BfsHeuristic::getValuesVisualization() -> visual::Marker
{
    if (m_goal_x < 0 || m_goal_y < 0 || m_goal_z < 0) {
        return visual::MakeEmptyMarker();
    }

    if (m_bfs->isWall(m_goal_x, m_goal_y, m_goal_z)) {
        return visual::MakeEmptyMarker();
    }

    // hopefully this doesn't screw anything up too badly...this will flush the
    // bfs to a little past the start, but this would be done by the search
    // hereafter anyway
    int start_heur = GetGoalHeuristic(planningSpace()->getStartStateID());
    if (start_heur == Infinity) {
        return visual::MakeEmptyMarker();
    }

    SMPL_INFO("Start cell heuristic: %d", start_heur);

    const int max_cost = (int)(1.1 * start_heur);

    SMPL_INFO("Get visualization of cells up to cost %d", max_cost);

    // ...and this will also flush the bfs...

    const size_t max_points = 4 * 4096;

    std::vector<Eigen::Vector3d> points;
    std::vector<visual::Color> colors;

    struct CostCell
    {
        int x, y, z, g;
    };
    std::queue<CostCell> cells;
    Grid3<bool> visited(grid()->numCellsX(), grid()->numCellsY(), grid()->numCellsZ(), false);
    visited(m_goal_x, m_goal_y, m_goal_z) = true;
    cells.push({m_goal_x, m_goal_y, m_goal_z, 0});
    while (!cells.empty()) {
        CostCell c = cells.front();
        cells.pop();

        if (c.g > max_cost || points.size() >= max_points) {
            break;
        }

        {
            double cost_pct = (double)c.g / (double)max_cost;

            visual::Color color = visual::MakeColorHSV(300.0 - 300.0 * cost_pct);

            auto clamp = [](double d, double lo, double hi) {
                if (d < lo) {
                    return lo;
                } else if (d > hi) {
                    return hi;
                } else {
                    return d;
                }
            };

            color.r = clamp(color.r, 0.0f, 1.0f);
            color.g = clamp(color.g, 0.0f, 1.0f);
            color.b = clamp(color.b, 0.0f, 1.0f);

            Eigen::Vector3d p;
            grid()->gridToWorld(c.x, c.y, c.z, p.x(), p.y(), p.z());
            points.push_back(p);

            colors.push_back(color);
        }

//        visited(c.x, c.y, c.z) = true;

        const int d = m_cost_per_cell * m_bfs->getDistance(c.x, c.y, c.z);

        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
            if (!(dx | dy | dz)) {
                continue;
            }

            int sx = c.x + dx;
            int sy = c.y + dy;
            int sz = c.z + dz;

            // check if neighbor is valid
            if (!m_bfs->inBounds(sx, sy, sz) || m_bfs->isWall(sx, sy, sz)) {
                continue;
            }

            // check if cost can be improved
            if (visited(sx, sy, sz)) {
                continue;
            }

            visited(sx, sy, sz) = true;

            int dd = m_cost_per_cell * m_bfs->getDistance(sx, sy, sz);
            cells.push({sx, sy, sz, dd});
        }
        }
        }
    }

    return visual::MakeCubesMarker(
            std::move(points),
            0.5 * grid()->resolution(),
            std::move(colors),
            grid()->getReferenceFrame(),
            "bfs_values");
}

void BfsHeuristic::syncGridAndBfs()
{
    const int xc = grid()->numCellsX();
    const int yc = grid()->numCellsY();
    const int zc = grid()->numCellsZ();
//    SMPL_DEBUG_NAMED(LOG, "Initializing BFS of size %d x %d x %d = %d", xc, yc, zc, xc * yc * zc);
    m_bfs.reset(new BFS_3D(xc, yc, zc));
    const int cell_count = xc * yc * zc;
    int wall_count = 0;
    for (int x = 0; x < xc; ++x) {
    for (int y = 0; y < yc; ++y) {
    for (int z = 0; z < zc; ++z) {
        const double radius = m_inflation_radius;
        if (grid()->getDistance(x, y, z) <= radius) {
            m_bfs->setWall(x, y, z);
            ++wall_count;
        }
    }
    }
    }

    SMPL_DEBUG_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

int BfsHeuristic::getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const
{
    if (!bfs.inBounds(x, y, z)) {
        return Infinity;
    }
    else if (bfs.getDistance(x, y, z) == BFS_3D::WALL) {
        return Infinity;
    }
    else {
        return m_cost_per_cell * bfs.getDistance(x, y, z);
    }
}

} // namespace motion
} // namespace sbpl
