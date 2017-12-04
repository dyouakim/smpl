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

#ifndef SMPL_DISTANCE_MAP_HPP
#define SMPL_DISTANCE_MAP_HPP

#include "../distance_map.h"

// standard includes
#include <cmath>
#include <algorithm>
#include <set>

namespace sbpl {

#define VECTOR_BUCKET_LIST_INSERT(o, key) \
{\
    o->pos = m_open[key].size();\
    m_open[key].push_back(o);\
    o->bucket = key;\
}

#define VECTOR_BUCKET_LIST_UPDATE(o, key) \
{\
    m_open[o->bucket][o->pos] = m_open[o->bucket].back();\
\
    m_open[o->bucket][o->pos]->pos = o->pos;\
    m_open[o->bucket].pop_back();\
\
    o->pos = m_open[key].size();\
    m_open[key].push_back(o);\
    o->bucket = key;\
}

#define VECTOR_BUCKET_LIST_POP(s, b) \
{\
    s = m_open[b].back();\
    m_open[b].pop_back();\
    s->bucket = -1;\
}

#define BUCKET_INSERT(o, key) do { VECTOR_BUCKET_LIST_INSERT(o, key) } while (0)
#define BUCKET_UPDATE(o, key) do { VECTOR_BUCKET_LIST_UPDATE(o, key) } while (0)
#define BUCKET_POP(s, bucket) do { VECTOR_BUCKET_LIST_POP(s, bucket) } while (0)

/// \class DistanceMap
///
/// An unsigned distance transform implementation that computes distance values
/// incrementally up to a maximum threshold. The distance of each cell is
/// determined as the closer of the distance from its nearest obstacle cell and
/// the distance to the nearest border cell. Distances for cells outside the
/// specified bounding volume will be reported as 0. Border cells are defined
/// as imaginary cells that are adjacent to valid in-bounds cells but are not
/// themselves within the bounding volume.
///
/// The basis for this class can be found in 'Sebastian Scherer, David Ferguson,
/// and Sanjiv Singh, "Efficient C-Space and Cost Function Updates in 3D for
/// Unmanned Aerial Vehicles," Proceedings International Conference on Robotics
/// and Automation, May, 2009.', albeit with some implementation differences.
/// Notably, it is preferred to compute the distance updates for simultaneous
/// obstacle insertion and removal in a two-phase fashion rather than
/// interleaving the updates for the insertion and removal of obstacles. While
/// the latter should complete in fewer propagation iterations, it is not
/// usually worth the additional overhead to maintain the priority queue
/// correctly in this domain.
///
/// The base class DistanceMapBase completes implements functionality
/// independent of the distance function that is used to update cells. It is
/// not intended to be used directly. DistanceMap implements functionality that
/// is dependent on the distance function used.
///
/// The class passed through as the template parameter specifies the distance
/// function used to compute the distance updates from neighboring cells by
/// implementing a member function with the following signature:
///
///     int distance(const Cell& s, const Cell& n);
///
/// that returns the new distance for cell s, being updated from adjacent cell
/// n. If the distance function is made private (and it likely should be, since
/// the Cell struct is private to DistanceMapBase), the derived class must
/// declare DistanceMap as a friend.

template <typename Derived>
DistanceMap<Derived>::DistanceMap(
    double origin_x, double origin_y, double origin_z,
    double size_x, double size_y, double size_z,
    double resolution,
    double max_dist)
:
    DistanceMapInterface(
        origin_x, origin_y, origin_z,
        size_x, size_y, size_z,
        resolution),
    m_cells(),
    m_max_dist(max_dist),
    m_inv_res(1.0 / resolution),
    m_dmax_int((int)std::ceil(m_max_dist * m_inv_res)),
    m_dmax_sqrd_int(m_dmax_int * m_dmax_int),
    m_bucket(m_dmax_sqrd_int + 1),
    m_neighbors(),
    m_indices(),
    m_neighbor_ranges(),
    m_neighbor_offsets(),
    m_neighbor_dirs(),
    m_open(),
    m_rem_stack()
{
    int cell_count_x = (int)(size_x * m_inv_res + 0.5) + 2;
    int cell_count_y = (int)(size_y * m_inv_res + 0.5) + 2;
    int cell_count_z = (int)(size_z * m_inv_res + 0.5) + 2;

    m_open.resize(m_dmax_sqrd_int + 1);

    // precompute table of sqrts for relevant distance values
    m_sqrt_table.resize(m_dmax_sqrd_int + 1, 0.0);
    for (int i = 0; i < m_dmax_sqrd_int + 1; ++i) {
        m_sqrt_table[i] = m_res * std::sqrt((double)i);
    }

    // init neighbors for forward propagation
    CreateNeighborUpdateList(m_neighbors, m_indices, m_neighbor_ranges);

    for (size_t i = 0; i < m_indices.size(); ++i) {
        const Eigen::Vector3i& neighbor = m_neighbors[m_indices[i]];
        m_neighbor_offsets[i] = 0;
        m_neighbor_offsets[i] += neighbor.x() * cell_count_z * cell_count_y;
        m_neighbor_offsets[i] += neighbor.y() * cell_count_z;
        m_neighbor_offsets[i] += neighbor.z() * 1;

        if (i < NON_BORDER_NEIGHBOR_LIST_SIZE) {
            m_neighbor_dirs[i] = dirnum(neighbor.x(), neighbor.y(), neighbor.z());
        } else {
            m_neighbor_dirs[i] = dirnum(neighbor.x(), neighbor.y(), neighbor.z(), 1);
        }
    }

    // initialize non-border free cells
    m_cells.resize(cell_count_x, cell_count_y, cell_count_z);
    for (int x = 1; x < m_cells.xsize() - 1; ++x) {
    for (int y = 1; y < m_cells.ysize() - 1; ++y) {
    for (int z = 1; z < m_cells.zsize() - 1; ++z) {
        Cell& c = m_cells(x, y, z);
        resetCell(c);
        c.x = x;
        c.y = y;
        c.z = z;
    }
    }
    }

    initBorderCells();
    propagateBorder();
}

template <class Derived>
DistanceMap<Derived>::DistanceMap(const DistanceMap& o) :
    DistanceMapInterface(o),
    m_cells(o.m_cells),
    m_max_dist(o.m_max_dist),
    m_inv_res(o.m_inv_res),
    m_dmax_int(o.m_dmax_int),
    m_dmax_sqrd_int(o.m_dmax_sqrd_int),
    m_bucket(o.m_bucket),
    m_neighbors(o.m_neighbors),
    m_indices(o.m_indices),
    m_neighbor_ranges(o.m_neighbor_ranges),
    m_neighbor_offsets(o.m_neighbor_offsets),
    m_neighbor_dirs(o.m_neighbor_dirs),
    m_sqrt_table(o.m_sqrt_table),
    m_open(o.m_open),
    m_rem_stack(o.m_rem_stack)
{
    rewire(o);
}

template <class Derived>
DistanceMap<Derived>::DistanceMap(DistanceMap&& o) :
    DistanceMapInterface(std::move(o)),
    m_cells(std::move(o.m_cells)),
    m_max_dist(std::move(o.m_max_dist)),
    m_inv_res(std::move(o.m_inv_res)),
    m_dmax_int(std::move(o.m_dmax_int)),
    m_dmax_sqrd_int(std::move(o.m_dmax_sqrd_int)),
    m_bucket(std::move(o.m_bucket)),
    m_neighbors(std::move(o.m_neighbors)),
    m_indices(std::move(o.m_indices)),
    m_neighbor_ranges(std::move(o.m_neighbor_ranges)),
    m_neighbor_offsets(std::move(o.m_neighbor_offsets)),
    m_neighbor_dirs(std::move(o.m_neighbor_dirs)),
    m_sqrt_table(std::move(o.m_sqrt_table)),
    m_open(std::move(o.m_open)),
    m_rem_stack(std::move(o.m_rem_stack))
{
}

template <class Derived>
auto DistanceMap<Derived>::operator=(const DistanceMap& rhs) -> DistanceMap&
{
    static_cast<DistanceMapInterface&>(*this) = rhs;
    if (this != &rhs) {
        m_cells = rhs.m_cells;
        m_max_dist = rhs.m_max_dist;
        m_inv_res = rhs.m_inv_res;
        m_dmax_int = rhs.m_dmax_int;
        m_dmax_sqrd_int = rhs.m_dmax_sqrd_int;
        m_bucket = rhs.m_bucket;
        m_neighbors = rhs.m_neighbors;
        m_indices = rhs.m_indices;
        m_neighbor_ranges = rhs.m_neighbor_ranges;
        m_neighbor_offsets = rhs.m_neighbor_offsets;
        m_neighbor_dirs = rhs.m_neighbor_dirs;
        m_sqrt_table = rhs.m_sqrt_table;
        m_open = rhs.m_open;
        m_rem_stack = rhs.m_rem_stack;
        rewire(rhs);
    }
    return *this;
}

template <class Derived>
auto DistanceMap<Derived>::operator=(DistanceMap&& rhs) -> DistanceMap&
{
    static_cast<DistanceMapInterface&>(*this) = std::move(rhs);
    if (this != &rhs) {
        m_cells = std::move(rhs.m_cells);
        m_max_dist = std::move(rhs.m_max_dist);
        m_inv_res = std::move(rhs.m_inv_res);
        m_dmax_int = std::move(rhs.m_dmax_int);
        m_dmax_sqrd_int = std::move(rhs.m_dmax_sqrd_int);
        m_bucket = std::move(rhs.m_bucket);
        m_neighbors = std::move(rhs.m_neighbors);
        m_indices = std::move(rhs.m_indices);
        m_neighbor_ranges = std::move(rhs.m_neighbor_ranges);
        m_neighbor_offsets = std::move(rhs.m_neighbor_offsets);
        m_neighbor_dirs = std::move(rhs.m_neighbor_dirs);
        m_sqrt_table = std::move(rhs.m_sqrt_table);
        m_open = std::move(rhs.m_open);
        m_rem_stack = std::move(rhs.m_rem_stack);
    }
    return *this;
}

/// Return the distance value for an invalid cell.
template <typename Derived>
double DistanceMap<Derived>::maxDistance() const
{
    return m_max_dist;
}

/// Return the distance of a cell from its nearest obstacle. This function will
/// also consider the distance to the nearest border cell. A value of 0.0 is
/// returned for obstacle cells and cells outside of the bounding volume.
template <typename Derived>
double DistanceMap<Derived>::getDistance(double x, double y, double z) const
{
    int gx, gy, gz;
    worldToGrid(x, y, z, gx, gy, gz);
    return getDistance(gx, gy, gz);
}

/// Return the distance of a cell from its nearest obstacle cell. This function
/// will also consider the distance to the nearest border cell. A value of 0.0
/// is returned for obstacle cells and cells outside of the bounding volume.
template <typename Derived>
double DistanceMap<Derived>::getDistance(int x, int y, int z) const
{
    if (!isCellValid(x, y, z)) {
        return 0.0;
    }

    int d2 = m_cells(x + 1, y + 1, z + 1).dist;
    return m_sqrt_table[d2];
}

/// Add a set of obstacle points to the distance map and update the distance
/// values of affected cells. Points outside the map and cells that are already
/// marked as obstacles will be ignored.
template <typename Derived>
void DistanceMap<Derived>::addPointsToMap(
    const std::vector<Eigen::Vector3d>& points)
{
    for (const Eigen::Vector3d& p : points) {
        int gx, gy, gz;
        worldToGrid(p.x(), p.y(), p.z(), gx, gy, gz);
        if (!isCellValid(gx, gy, gz)) {
            continue;
        }

        ++gx; ++gy; ++gz;

        Cell& c = m_cells(gx, gy, gz);
        if (c.dist_new > 0) {
            c.dir = NO_UPDATE_DIR;
            c.dist_new = 0;
            c.obs = &c;
            updateVertex(&c);
        }
    }

    propagate();
}

/// Remove a set of obstacle points from the distance map and update the
/// distance values of affected cells. Points outside the map and cells that
/// are already marked as obstacles will be ignored.
template <typename Derived>
void DistanceMap<Derived>::removePointsFromMap(
    const std::vector<Eigen::Vector3d>& points)
{
    for (const Eigen::Vector3d& p : points) {
        int gx, gy, gz;
        worldToGrid(p.x(), p.y(), p.z(), gx, gy, gz);
        if (!isCellValid(gx, gy, gz)) {
            continue;
        }

        ++gx; ++gy; ++gz;

        Cell& c = m_cells(gx, gy, gz);

        if (c.obs != &c) {
            continue;
        }

        c.dist_new = m_dmax_sqrd_int;
        c.obs = nullptr;

        c.dist = m_dmax_sqrd_int;
        c.dir = NO_UPDATE_DIR;
        m_rem_stack.push_back(&c);
    }

    propagateRemovals();
}

/// Add the set (new_points - old_points) of obstacle cells and remove the set
/// (old_points - new_points) of obstacle cells and update the distance values
/// of affected cells. Points outside the map will be ignored.
template <typename Derived>
void DistanceMap<Derived>::updatePointsInMap(
    const std::vector<Eigen::Vector3d>& old_points,
    const std::vector<Eigen::Vector3d>& new_points)
{
    std::set<Eigen::Vector3i, Eigen_Vector3i_compare> old_point_set;
    for (const Eigen::Vector3d& wp : old_points) {
        Eigen::Vector3i gp;
        worldToGrid(wp.x(), wp.y(), wp.z(), gp.x(), gp.y(), gp.z());
        if (isCellValid(gp.x(), gp.y(), gp.z())) {
            ++gp.x(); ++gp.y(); ++gp.z();
            old_point_set.insert(gp);
        }
    }

    std::set<Eigen::Vector3i, Eigen_Vector3i_compare> new_point_set;
    for (const Eigen::Vector3d& wp : new_points) {
        Eigen::Vector3i gp;
        worldToGrid(wp.x(), wp.y(), wp.z(), gp.x(), gp.y(), gp.z());
        if (isCellValid(gp.x(), gp.y(), gp.z())) {
            ++gp.x(); ++gp.y(); ++gp.z();
            new_point_set.insert(gp);
        }
    }

    Eigen_Vector3i_compare comp;

    std::vector<Eigen::Vector3i> old_not_new;
    std::set_difference(
            old_point_set.begin(), old_point_set.end(),
            new_point_set.begin(), new_point_set.end(),
            std::inserter(old_not_new, old_not_new.end()),
            comp);

    std::vector<Eigen::Vector3i> new_not_old;
    std::set_difference(
            new_point_set.begin(), new_point_set.end(),
            old_point_set.begin(), old_point_set.end(),
            std::inserter(new_not_old, new_not_old.end()),
            comp);

    // remove obstacle cells that were in the old cloud but not the new cloud
    for (const Eigen::Vector3i& p : old_not_new) {
        Cell& c = m_cells(p.x(), p.y(), p.z());
        if (c.obs != &c) {
            continue; // skip already-free cells
        }
        c.dir = NO_UPDATE_DIR;
        c.dist_new = m_dmax_sqrd_int;
        c.dist = m_dmax_sqrd_int;
        c.obs = nullptr;
        m_rem_stack.push_back(&c);
    }

    propagateRemovals();

    // add obstacle cells that are in the new cloud but not the old cloud
    for (const Eigen::Vector3i& p : new_not_old) {
        Cell& c = m_cells(p.x(), p.y(), p.z());
        if (c.dist_new == 0) {
            continue; // skip already-obstacle cells
        }
        c.dir = NO_UPDATE_DIR;
        c.dist_new = 0;
        c.obs = &c;
        updateVertex(&c);
    }

    propagate();
}

/// Reset all points in the distance map to their uninitialized (free) values.
template <typename Derived>
void DistanceMap<Derived>::reset()
{
    for (size_t x = 1; x < m_cells.xsize() - 1; ++x) {
    for (size_t y = 1; y < m_cells.ysize() - 1; ++y) {
    for (size_t z = 1; z < m_cells.zsize() - 1; ++z) {
        Cell& c = m_cells(x, y, z);
        resetCell(c);
    }
    }
    }

    initBorderCells();

    propagateBorder();
}

/// Return the number of cells along the x axis.
template <typename Derived>
int DistanceMap<Derived>::numCellsX() const
{
    return m_cells.xsize() - 2;
}

/// Return the number of cells along the y axis.
template <typename Derived>
int DistanceMap<Derived>::numCellsY() const
{
    return m_cells.ysize() - 2;
}

/// Return the number of cells along the z axis.
template <typename Derived>
int DistanceMap<Derived>::numCellsZ() const
{
    return m_cells.zsize() - 2;
}

template <typename Derived>
double DistanceMap<Derived>::getUninitializedDistance() const
{
    return m_max_dist;
}

template <typename Derived>
double DistanceMap<Derived>::getMetricDistance(double x, double y, double z) const
{
    return getDistance(x, y, z);
}

template <typename Derived>
double DistanceMap<Derived>::getCellDistance(int x, int y, int z) const
{
    return getDistance(x, y, z);
}

/// Return the point in world coordinates marking the center of the cell at the
/// given effective grid coordinates.
template <typename Derived>
void DistanceMap<Derived>::gridToWorld(
    int x, int y, int z,
    double& world_x, double& world_y, double& world_z) const
{
    world_x = (m_origin_x - m_res) + (x + 1) * m_res;
    world_y = (m_origin_y - m_res) + (y + 1) * m_res;
    world_z = (m_origin_z - m_res) + (z + 1) * m_res;
}

/// Return the effective grid coordinates of the cell containing the given point
/// specified in world coordinates.
template <typename Derived>
void DistanceMap<Derived>::worldToGrid(
    double world_x, double world_y, double world_z,
    int& x, int& y, int& z) const
{
    x = (int)(m_inv_res * (world_x - (m_origin_x - m_res)) + 0.5) - 1;
    y = (int)(m_inv_res * (world_y - (m_origin_y - m_res)) + 0.5) - 1;
    z = (int)(m_inv_res * (world_z - (m_origin_z - m_res)) + 0.5) - 1;
}

/// Test if a cell is outside the bounding volume.
template <typename Derived>
bool DistanceMap<Derived>::isCellValid(int x, int y, int z) const
{
    return x >= 0 && x < m_cells.xsize() - 2 &&
        y >= 0 && y < m_cells.ysize() - 2 &&
        z >= 0 && z < m_cells.zsize() - 2;
}

template <typename Derived>
void DistanceMap<Derived>::rewire(const DistanceMap& o)
{
    for (Cell& cell : m_cells) {
        if (cell.obs) {
            auto offset = cell.obs - o.m_cells.data();
            cell.obs = m_cells.data() + offset;
        }
    }

    for (auto& bucket : m_open) {
        for (auto& cell : bucket) {
            auto offset = cell - o.m_cells.data();
            cell = m_cells.data() + offset;
        }
    }
    for (auto& cell : m_rem_stack) {
        auto offset = cell - o.m_cells.data();
        cell = m_cells.data() + offset;
    }
}

template <typename Derived>
void DistanceMap<Derived>::initBorderCells()
{
    auto init_obs_cell = [&](int x, int y, int z) {
        Cell& c = m_cells(x, y, z);
        c.x = x;
        c.y = y;
        c.z = z;
        c.dist = m_dmax_sqrd_int;
        c.dist_new = 0;
#if SMPL_DMAP_RETURN_CHANGED_CELLS
        c.dist_old = m_dmax_sqrd_int;
#endif
        c.obs = &c;
        c.bucket = -1;

        int src_dir_x = (x == 0) ? 1 : ((x == m_cells.xsize() - 1) ? -1 : 0);
        int src_dir_y = (y == 0) ? 1 : ((y == m_cells.ysize() - 1) ? -1 : 0);
        int src_dir_z = (z == 0) ? 1 : ((z == m_cells.zsize() - 1) ? -1 : 0);
        c.dir = dirnum(src_dir_x, src_dir_y, src_dir_z, 1);
        updateVertex(&c);
    };

    // initialize border cells
    for (int y = 0; y < m_cells.ysize(); ++y) {
    for (int z = 0; z < m_cells.zsize(); ++z) {
        init_obs_cell(0, y, z);
        init_obs_cell(m_cells.xsize() - 1, y, z);
    }
    }
    for (int x = 1; x < m_cells.xsize() - 1; ++x) {
    for (int z = 0; z < m_cells.zsize(); ++z) {
        init_obs_cell(x, 0, z);
        init_obs_cell(x, m_cells.ysize() - 1, z);
    }
    }
    for (int x = 1; x < m_cells.xsize() - 1; ++x) {
    for (int y = 1; y < m_cells.ysize() - 1; ++y) {
        init_obs_cell(x, y, 0);
        init_obs_cell(x, y, m_cells.zsize() - 1);
    }
    }
}

template <typename Derived>
void DistanceMap<Derived>::updateVertex(Cell* o)
{
    const int key = std::min(o->dist, o->dist_new);
    assert(key < m_open.size());
    if (o->bucket >= 0) { // in heap
        assert(o->bucket < m_open.size());
        BUCKET_UPDATE(o, key);
    } else { // not in the heap yet
        BUCKET_INSERT(o, key);
    }
    if (key < m_bucket) {
        m_bucket = key;
    }
}

template <typename Derived>
int DistanceMap<Derived>::distance(const Cell& n, const Cell& s)
{
    return static_cast<Derived*>(this)->distance(n, s);
}

template <typename Derived>
void DistanceMap<Derived>::lower(Cell* s)
{
    int nfirst, nlast;
    std::tie(nfirst, nlast) = m_neighbor_ranges[s->dir];
    for (int i = nfirst; i != nlast; ++i) {
        Cell* n = s + m_neighbor_offsets[i];
//        if (n->dist_new > s->dist_new)
        {
            int dp = distance(*n, *s);
            if (dp < n->dist_new) {
                n->dist_new = dp;
                n->obs = s->obs;
                n->dir = m_neighbor_dirs[i];
                updateVertex(n);
            }
        }
    }
}

template <typename Derived>
void DistanceMap<Derived>::raise(Cell* s)
{
    int nfirst, nlast;
    std::tie(nfirst, nlast) = m_neighbor_ranges[NO_UPDATE_DIR];
    for (int i = nfirst; i != nlast; ++i) {
        Cell* n = s + m_neighbor_offsets[i];
        waveout(n);
    }
    waveout(s);
}

template <typename Derived>
void DistanceMap<Derived>::waveout(Cell* n)
{
    if (n == n->obs) {
        return;
    }

    n->dist_new = m_dmax_sqrd_int;
    Cell* obs_old = n->obs;
    n->obs = nullptr;

    int nfirst, nlast;
    std::tie(nfirst, nlast) = m_neighbor_ranges[NO_UPDATE_DIR];
    for (int i = nfirst; i != nlast; ++i) {
        Cell* a = n + m_neighbor_offsets[i];
        auto valid = [](Cell* c) { return c && c->obs == c; };
        if (valid(a->obs)) {
            int dp = distance(*n, *a);
            if (dp < n->dist_new) {
                n->dist_new = dp;
                n->obs = a->obs;
                n->dir = NO_UPDATE_DIR;
            }
        }
    }

    if (n->obs != obs_old) {
//        n->dir = NO_UPDATE_DIR;
        updateVertex(n);
    }
}

template <typename Derived>
void DistanceMap<Derived>::propagate()
{
    int iter_count = 0;
    while (m_bucket < (int)m_open.size()) {
        while (!m_open[m_bucket].empty()) {
            ++iter_count;
            assert(m_bucket >= 0 && m_bucket < m_open.size());
            Cell* s;
            BUCKET_POP(s, m_bucket);

            if (s->dist_new < s->dist) {
                s->dist = s->dist_new;

                // foreach n in adj(min)
                lower(s);

#if SMPL_DMAP_RETURN_CHANGED_CELLS
                if (s->dist != s->dist_old) {
                    // insert(C, s)
                    s->dist_old = s->dist;
                }
#endif
            } else {
                s->dist = m_dmax_sqrd_int;
                s->dir = NO_UPDATE_DIR;
                raise(s);
                if (s->dist != s->dist_new) {
                    updateVertex(s);
                }
            }
        }
        ++m_bucket;
    }
}

template <typename Derived>
void DistanceMap<Derived>::lowerBounded(Cell* s)
{
    int nfirst, nlast;
    std::tie(nfirst, nlast) = m_neighbor_ranges[s->dir];
    for (int i = nfirst; i != nlast; ++i) {
        Cell* n = s + m_neighbor_offsets[i];
        if (n->dist_new > s->dist_new) {
            int dp = distance(*n, *s);
            if (dp < n->dist_new) {
                n->dist_new = dp;
                n->obs = s->obs;
                n->dir = m_neighbor_dirs[i];
                updateVertex(n);
            }
        }
    }
}

template <typename Derived>
void DistanceMap<Derived>::propagateRemovals()
{
    while (!m_rem_stack.empty()) {
        Cell* s = m_rem_stack.back();
        m_rem_stack.pop_back();

        int nfirst, nlast;
        std::tie(nfirst, nlast) = m_neighbor_ranges[NO_UPDATE_DIR];
        for (int i = nfirst; i != nlast; ++i) {
            Cell* n = s + m_neighbor_offsets[i];
            auto valid = [](Cell* c) { return c && c->obs == c; };
            if (!valid(n->obs)) {
                if (n->dist_new != m_dmax_sqrd_int) {
                    n->dist_new = m_dmax_sqrd_int;
                    n->dist = m_dmax_sqrd_int;
                    n->obs = nullptr;
                    n->dir = NO_UPDATE_DIR;
                    m_rem_stack.push_back(n);
                }
            } else {
                updateVertex(n);
            }
        }
    }

    propagateBorder();
}

template <typename Derived>
void DistanceMap<Derived>::propagateBorder()
{
    while (m_bucket < (int)m_open.size()) {
        while (!m_open[m_bucket].empty()) {
            assert(m_bucket >= 0 && m_bucket < m_open.size());
            Cell* s;
            BUCKET_POP(s, m_bucket);

//            if (s->dist_new < s->dist)
            {
                assert(s->dist_new <= s->dist);
                s->dist = s->dist_new;

                // foreach n in adj(min)
                lowerBounded(s);

#if SMPL_DMAP_RETURN_CHANGED_CELLS
                if (s->dist != s->dist_old) {
                    // insert(C, s)
                    s->dist_old = s->dist;
                }
#endif
            }
        }
        ++m_bucket;
    }
}

template <typename Derived>
void DistanceMap<Derived>::resetCell(Cell& c) const
{
    c.dist = m_dmax_sqrd_int;
    c.dist_new = m_dmax_sqrd_int;
#if SMPL_DMAP_RETURN_CHANGED_CELLS
    c.dist_old = m_dmax_sqrd_int;
#endif
    c.obs = nullptr;
    c.bucket = -1;
    c.dir = NO_UPDATE_DIR;
}

} // namespace sbpl

#endif
