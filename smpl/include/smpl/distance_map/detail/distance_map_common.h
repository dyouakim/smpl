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

#ifndef SMPL_DISTANCE_MAP_COMMON_H
#define SMPL_DISTANCE_MAP_COMMON_H

// standard includes
#include <array>
#include <utility>

// system includes
#include <Eigen/Dense>

namespace sbpl {

#define SMPL_DMAP_RETURN_CHANGED_CELLS 0

static const int NUM_DIRECTIONS = 2 * 27;
static const int NON_BORDER_NEIGHBOR_LIST_SIZE = 460;
static const int BORDER_NEIGHBOR_LIST_SIZE = 316;
static const int NEIGHBOR_LIST_SIZE =
        NON_BORDER_NEIGHBOR_LIST_SIZE + BORDER_NEIGHBOR_LIST_SIZE;

/// Return the index into a 27-connected neighbors array for a given direction.
/// The index is offset by 27 if edge is 1
inline
constexpr int dirnum(int dx, int dy, int dz, int edge = 0)
{
    return 27 * edge + 9 * (dx + 1) + 3 * (dy + 1) + (dz + 1);
}

void CreateNeighborUpdateList(
    std::array<Eigen::Vector3i, 27>& neighbors,
    std::array<int, NEIGHBOR_LIST_SIZE>& indices,
    std::array<std::pair<int, int>, NUM_DIRECTIONS>& ranges);

struct Eigen_Vector3i_compare
{
    bool operator()(const Eigen::Vector3i& u, const Eigen::Vector3i& v)
    {
        return std::tie(u.x(), u.y(), u.z()) < std::tie(v.x(), v.y(), v.z());
    }
};

} // namespace sbpl

#endif
