////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
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

#include <smpl/robot_model.h>

#include <assert.h>
#include <sstream>

namespace sbpl {
namespace motion {

namespace ik_option {

std::ostream& operator<<(std::ostream& o, IkOption option)
{
    o << to_cstring(option);
    return o;
}

auto to_cstring(IkOption option) -> const char*
{
    switch (option) {
    case UNRESTRICTED:
        return "UNRESTRICTED";
    case RESTRICT_XYZ:
        return "RESTRICT_XYZ";
    case RESTRICT_RPY:
        return "RESTRICT_RPY";
    default:
        assert(0);
        return "";
    }
}

} // namespace ik_option

RobotModel::~RobotModel()
{
}

void RobotModel::setPlanningJoints(const std::vector<std::string>& joints)
{
    planning_joints_ = joints;
}

const std::vector<std::string>& RobotModel::getPlanningJoints() const
{
    return planning_joints_;
}

ForwardKinematicsInterface::~ForwardKinematicsInterface()
{
}

InverseKinematicsInterface::~InverseKinematicsInterface()
{
}

} // namespace motion
} // namespace sbpl
