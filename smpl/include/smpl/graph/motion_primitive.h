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

#ifndef SMPL_MOTION_PRIMITIVE_H
#define SMPL_MOTION_PRIMITIVE_H

// standard includes
#include <iomanip>
#include <sstream>
#include <vector>

// project includes
#include <smpl/types.h>
#include <smpl/console/console.h>

namespace sbpl {
namespace motion {

struct MotionPrimitive
{
    enum Type
    {
        LONG_DISTANCE = 0,
        SHORT_DISTANCE,
        SNAP_TO_RPY,
        SNAP_TO_XYZ,
        SNAP_TO_XYZ_RPY,
        NUMBER_OF_MPRIM_TYPES
    };

    Type type;
    Action action;

    void print() const;
};

inline auto to_cstring(MotionPrimitive::Type type) -> const char* {
    switch (type) {
    case MotionPrimitive::LONG_DISTANCE:
        return "LONG_DISTANCE";
    case MotionPrimitive::SNAP_TO_RPY:
        return "SNAP_TO_RPY";
    case MotionPrimitive::SNAP_TO_XYZ:
        return "SNAP_TO_XYZ";
    case MotionPrimitive::SNAP_TO_XYZ_RPY:
        return "SNAP_TO_XYZ_RPY";
    case MotionPrimitive::SHORT_DISTANCE:
        return "SHORT_DISTANCE";
    default:
        assert(0);
        return "";
    }
}

inline auto operator<<(std::ostream& o, MotionPrimitive::Type type)
    -> std::ostream&
{
    o << to_cstring(type);
    return o;
}

inline
void MotionPrimitive::print() const
{
    SMPL_INFO("type: %d  nsteps: %d ", type, int(action.size()));
    std::stringstream os;
    for (std::size_t j = 0; j < action.size(); ++j) {
        os.str("");
        os << "[step: " << int(j+1) << "/" << action.size() << "] ";
        for (std::size_t k = 0; k < action[j].size(); ++k) {
            os << std::setw(4) << std::setprecision(3) << std::fixed << action[j][k] << " ";
        }
        SMPL_INFO_STREAM(os.str());
    }
}

} // namespace motion
} // namespace sbpl

#endif
