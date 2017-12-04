////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush
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

#ifndef SMPL_DISTANCE_MAP_H
#define SMPL_DISTANCE_MAP_H

// standard includes
#include <array>
#include <utility>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <Eigen/StdVector>

// project includes
#include <smpl/grid.h>
#include <smpl/forward.h>
#include <smpl/distance_map/distance_map_interface.h>

#include "detail/distance_map_common.h"

namespace sbpl {

template <typename Derived>
class DistanceMap : public DistanceMapInterface
{
public:

    DistanceMap(
        double origin_x, double origin_y, double origin_z,
        double size_x, double size_y, double size_z,
        double resolution,
        double max_dist);

    DistanceMap(const DistanceMap& o);
    DistanceMap(DistanceMap&& o);

    auto operator=(const DistanceMap& rhs) -> DistanceMap&;
    auto operator=(DistanceMap&& rhs) -> DistanceMap&;

    double maxDistance() const;

    double getDistance(double x, double y, double z) const;
    double getDistance(int x, int y, int z) const;

    /// \name Required Functions from DistanceMapInterface
    ///@{
    void addPointsToMap(const std::vector<Eigen::Vector3d>& points) override;
    void removePointsFromMap(const std::vector<Eigen::Vector3d>& points) override;
    void updatePointsInMap(
        const std::vector<Eigen::Vector3d>& old_points,
        const std::vector<Eigen::Vector3d>& new_points) override;

    void reset() override;

    int numCellsX() const override;
    int numCellsY() const override;
    int numCellsZ() const override;

    double getUninitializedDistance() const override;

    double getMetricDistance(double x, double y, double z) const override;
    double getCellDistance(int x, int y, int z) const override;

    void gridToWorld(
        int x, int y, int z,
        double& world_x, double& world_y, double& world_z) const override;

    void worldToGrid(
        double world_x, double world_y, double world_z,
        int& x, int& y, int& z) const override;

    bool isCellValid(int x, int y, int z) const override;
    ///@}

    friend Derived;

private:

    struct Cell
    {
        int x;
        int y;
        int z;

        int dist;
        int dist_new;
#if SMPL_DMAP_RETURN_CHANGED_CELLS
        int dist_old;
#endif
        Cell* obs;
        int bucket;
        int dir;

        int pos;
    };

    static constexpr int NO_UPDATE_DIR = dirnum(0, 0, 0);

    Grid3<Cell> m_cells;

    double m_max_dist;
    double m_inv_res;

    int m_dmax_int;
    int m_dmax_sqrd_int;

    int m_bucket;

    // Direction offsets to each of the 27 neighbors, including (0, 0, 0).
    // Indexed by a call to dirnum(x, y, z, 0);
    std::array<Eigen::Vector3i, 27> m_neighbors;

    // Storage for the indices of neighbor offsets that must have distance
    // information propagated to them, given the source's update direction. The
    // indices are arranged so that target neighbor indices for a given source
    // update direction are contiguous

    // [ s_1_t_1, ..., s_1_t_n, s_2_t_1, ..., s_2_t_n, ..., s_n_t_1, ..., s_n_t_n ]
    // where n = 2 * 27
    std::array<int, NEIGHBOR_LIST_SIZE> m_indices;

    // Map from a source update direction (obtained from dirnum(x, y, z, e)) to
    // a range of neighbor offsets (indices into m_neighbors) denoting neighbors
    // to which distance values must be propagated upon insertion
    std::array<std::pair<int, int>, NUM_DIRECTIONS> m_neighbor_ranges;

    // Map from a (source, target) update direction pair (obtained from
    // dirnum(x, y, z, e)) to a precomputed offsets into the grid for its target
    // neighbor offsets
    std::array<int, NEIGHBOR_LIST_SIZE> m_neighbor_offsets;

    // Map from a (source, target) update direction pair to the update direction
    // index
    std::array<int, NEIGHBOR_LIST_SIZE> m_neighbor_dirs;

    std::vector<double> m_sqrt_table;

    typedef std::vector<Cell*> bucket_type;
    typedef std::vector<bucket_type> bucket_list;
    bucket_list m_open;

    std::vector<Cell*> m_rem_stack;

    void rewire(const DistanceMap& o);

    void initBorderCells();

    void updateVertex(Cell* c);

    int distance(const Cell& n, const Cell& s);

    void lower(Cell* s);
    void raise(Cell* s);
    void waveout(Cell* n);
    void propagate();

    void lowerBounded(Cell* s);
    void propagateRemovals();
    void propagateBorder();

    void resetCell(Cell& c) const;
};

} // namespace sbpl

#include "detail/distance_map.hpp"

#endif
