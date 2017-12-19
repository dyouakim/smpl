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

#ifndef SMPL_TYPES_H
#define SMPL_TYPES_H

// standard includes
#include <functional>
#include <unordered_map>
#include <vector>

namespace sbpl {
namespace motion {

template <
    class Key,
    class T,
    class Hash = std::hash<Key>,
    class KeyEqual = std::equal_to<Key>,
    class Allocator = std::allocator<std::pair<const Key, T>>>
using hash_map = std::unordered_map<Key, T, Hash, KeyEqual, Allocator>;

template <typename T>
struct PointerValueHash
{
    typedef T* argument_type;
    typedef std::size_t result_type;
    result_type operator()(argument_type s) const { return std::hash<T>()(*s); }
};

template <typename T>
struct PointerValueEqual
{
    typedef T* argument_type;
    bool operator()(argument_type a, argument_type b) const { return *a == *b; }
};

#if 1
typedef std::vector<double> RobotState;
#else
// This class is eventually meant to replace the above typedef to add type
// safety to RobotState usages; currently, it is used as a compile-time
// mechanism to ensure for the time being that the RobotState identifier is used
// in all appropriate contexts.
class RobotState : public std::vector<double>
{
public:

    typedef std::vector<double> Base;
    typedef Base::value_type value_type;
    typedef Base::allocator_type allocator_type;
    typedef Base::size_type size_type;
    typedef Base::difference_type difference_type;
    typedef Base::reference reference;
    typedef Base::const_reference const_reference;
    typedef Base::pointer pointer;
    typedef Base::const_pointer const_pointer;
    typedef Base::iterator iterator;
    typedef Base::const_iterator const_iterator;
    typedef Base::reverse_iterator reverse_iterator;
    typedef Base::const_reverse_iterator const_reverse_iterator;

    explicit RobotState(const allocator_type& alloc = allocator_type()) :
        Base(alloc) { }

    RobotState(
        size_type count,
        const double& value = double(),
        const allocator_type& alloc = allocator_type())
    :
        Base(count, value, alloc)
    { }

    explicit RobotState(size_type count) : Base(count) { }

    template <class InputIt>
    RobotState(
        InputIt first,
        InputIt last,
        const allocator_type& alloc = allocator_type())
    :
        Base(first, last, alloc)
    { }

    RobotState(const RobotState& other) : Base(other) { }
    RobotState(const Base& other) : Base(other) { }

    RobotState(const RobotState& other, const allocator_type& alloc) :
        Base(other, alloc)
    { }
    RobotState(const Base& other, const allocator_type& alloc) :
        Base(other, alloc)
    { }

    RobotState(RobotState&& other) : Base(other) { }
    RobotState(Base&& other) : Base(other) { }

    RobotState(RobotState&& other, const allocator_type& alloc) :
        Base(other, alloc)
    { }
    RobotState(Base&& other, const allocator_type& alloc) :
        Base(other, alloc)
    { }

    RobotState(
            std::initializer_list<double> init,
            const allocator_type& alloc = allocator_type())
    :
        Base(init, alloc)
    { }

    RobotState& operator=(const RobotState& other) { Base::operator=(other); return *this; }
    RobotState& operator=(const Base& other) { Base::operator=(other); return *this; }
    RobotState& operator=(RobotState&& other) { Base::operator=(other); return *this; }
    RobotState& operator=(Base&& other) { Base::operator=(other); return *this; }
    RobotState& operator=(std::initializer_list<double> ilist) { Base::operator=(ilist); return *this; }
};
#endif

typedef std::vector<RobotState> Action;
typedef std::vector<double> ActionsWeight;

enum GoalType
{
    INVALID_GOAL_TYPE = -1,
    XYZ_GOAL,
    XYZ_RPY_GOAL,
    JOINT_STATE_GOAL,
    NUMBER_OF_GOAL_TYPES
};

enum GroupType
{
    FAILURE = -2,
    ANY, 
    BASE,
    ARM
} ;


struct GoalConstraint
{
    // Relevant for joint state goals
    RobotState angles;
    std::vector<double> angle_tolerances;

    // Relevant for workspace goals
    std::vector<double> pose;           // goal pose of the planning link as (x, y, z, R, P, Y)
    double xyz_offset[3];               // offset from the planning link
    double xyz_tolerance[3];            // (x, y, z) tolerance
    double rpy_tolerance[3];            // (R, P, Y) tolerance

    std::vector<double> tgt_off_pose;   // goal pose offset from planning link
    int xyz[3];                         // planning frame cell (x, y, z)

    GoalType type;                      // type of goal constraint
};

} // namespace motion
} // namespace sbpl

#endif
