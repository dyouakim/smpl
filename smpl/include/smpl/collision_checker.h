////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Benjamin Cohen, Andrew Dornbush
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

#ifndef SMPL_COLLISION_CHECKER_H
#define SMPL_COLLISION_CHECKER_H

// standard includes
#include <string>
#include <vector>

// project includes
#include <smpl/extension.h>
#include <smpl/types.h>
#include <smpl/debug/marker.h>

namespace sbpl {
namespace motion {

class CollisionChecker : public virtual Extension
{
public:

    CollisionChecker();
    virtual ~CollisionChecker();

    /// \brief Return whether a state is valid.
    /// \param[in] angles The joint angles of the joint group
    /// \param[in] verbose Whether to produce verbose output
    /// \param[in] visualize Whether to store collision details for the next
    ///     call to getVisualization
    /// \param[out] dist The distance to the nearest obstacle
    /// \return Whether the state is valid
    virtual bool isStateValid(const RobotState& state, bool verbose = false) = 0;

    /// \brief Return whether the interpolated path between two points is valid.
    ///
    /// Need not include the endpoints.
    ///
    /// \param[in] first The starting configuration of the joint group
    /// \param[in] last The ending configuration of the joint group
    /// \param[out] path_length The number of waypoints in the path between
    ///     angles0 and angles1
    /// \param[out] num_checks The number of collision checks to perform
    /// \param[out] dist The distance to the nearest obstacle
    /// \return true if the interpolated path is valid; false otherwise
    virtual bool isStateToStateValid(
        const RobotState& start,
        const RobotState& finish,
        bool verbose = false) = 0;

    /// \brief Return a linearly interpolated path between two joint states.
    ///
    /// This intended use is for this member function should return the path
    /// interpolated at the resolution used internally by isStateToStateValid.
    ///
    /// \param[in] start The start configuration of the joint group
    /// \param[in] end The end configuration of the joint group
    /// \param[out] path The output path
    /// \return Whether a valid linearly interpolated path could be constructed
    virtual bool interpolatePath(
        const RobotState& start,
        const RobotState& finish,
        std::vector<RobotState>& path) = 0;

    /// \name Visualization
    ///@{
    virtual auto getCollisionModelVisualization(const RobotState& state)
        -> std::vector<visual::Marker>;
    ///@}
};

class CollisionDistanceExtension : public virtual Extension
{
public:

    /// Return the distance to collision with the nearest obstacle.
    virtual double distanceToCollision(const RobotState& state) = 0;

    /// Return the distance to collision with the nearest obstacle along a
    /// motion.
    virtual double distanceToCollision(
        const RobotState& start,
        const RobotState& finish) = 0;
};

} // namespace motion
} // namespace sbpl

#endif

