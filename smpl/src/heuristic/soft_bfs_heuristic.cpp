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

#include <smpl/heuristic/soft_bfs_heuristic.h>

#include <smpl/console/console.h>

namespace sbpl {
namespace motion {

SoftBfsHeuristic::SoftBfsHeuristic(
    EnvironmentROBARM3D* env,
    const OccupancyGridConstPtr& grid,
    const PlanningParams* params)
:
    RobotHeuristic(env, grid, params)
{
    syncGridAndBfs();
}

bool SoftBfsHeuristic::setGoal(int x, int y, int z)
{
    int gx, gy, gz;
    m_grid->worldToGrid(x, y, z, gx, gy, gz);

    if (!m_bfs->escapeCell(x, y, z)) {
        SMPL_ERROR("BFS goal is out of bounds or couldn't be freed");
        return false;
    }

    m_bfs->run_components(x, y, z);
    SMPL_INFO(" -> Running");
    while (m_bfs->isRunning());
    SMPL_INFO(" -> Finished: %d walls, %d undiscovered, %d discovered", m_bfs->countWalls(), m_bfs->countUndiscovered(), m_bfs->countDiscovered());

    return true;
}

bool SoftBfsHeuristic::setGoal(double x, double y, double z)
{
    int gx, gy, gz;
    m_grid->worldToGrid(x, y, z, gx, gy, gz);
    return setGoal(gx, gy, gz);
}

int SoftBfsHeuristic::GetGoalHeuristic(int state_id)
{
    const ManipLatticeState* state = m_robarm_env->getHashEntry(state_id);
    if (state) {
        const int x = state->xyz[0];
        const int y = state->xyz[1];
        const int z = state->xyz[2];
        if (m_bfs_>isWall(x, y, z)) {
            const int h = 100 * m_bfs->getNearestFreeNodeDist(x, y, z);
            return INT_MAX;
        }
        else {
            int cell_dist = m_bfs->getDistance(x, y, z);
            if (cell_dist < 0) {
                SMPL_WARN_THROTTLE(1, "queried distance for isolated cell");
                m_bfs->setWall(x, y, z);
                while ((cell_dist = m_bfs->getNearestFreeNodeDist(x, y, z)) < 0);
                return 100 * cell_dist;
            }
            else {
                return 100 * m_bfs->getDistance(x, y, z);
            }
        }
    }
    else {
        return 0;
    }
}

int SoftBfsHeuristic::GetStartHeuristic(int state_id)
{
    return 0;
}

int SoftBfsHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    return 0;
}

visualization_msgs::MarkerArray SoftBfsHeuristic::getWallsVisualization() const
{

}

visualization_msgs::MarkerArray SoftBfsHeuristic::getValuesVisualization() const
{

}

void SoftBfsHeuristic::syncGridAndBfs()
{
    const int xc = m_grid->numCellsX();
    const int yc = m_grid->numCellsY();
    const int zc = m_grid->numCellsZ();
    m_bfs.reset(new BFS_3D(xc, yc, zc));
    const int cell_count = xc * yc * zc;
    int wall_count = 0;
    for (int z = 0; z < zc; ++z) {
        for (int y = 0; y < yc; ++y) {
            for (int x = 0; x < xc; ++x) {
                if (m_grid->getDistance(x, y, z) <=
                        m_params->planning_link_sphere_radius_)
                {
                    m_bfs->setWall(x, y, z);
                    ++wall_count;
                }
            }
        }
    }

    SMPL_INFO("%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
}

} // namespace motion
} // namespace sbpl
