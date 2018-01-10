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

#include <stdlib.h>
#include <vector>

#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/intrusive_heap.h>

#include <smpl/search/lazy_search_interface.h>

namespace sbpl {

struct LazyARAStar;

bool Init(
    LazyARAStar& search,
    ILazySuccFun* succ_fun,
    motion::RobotHeuristic* heuristic);

int Replan(
    LazyARAStar& search,
    int start_id,
    int goal_id,
    std::vector<int>& solution,
    int& cost);

struct LazyARAStar
{
    struct State;

    struct StateCompare {
        const LazyARAStar* search_;
        bool operator()(const State& s1, const State& s2) const;
    };

    struct CandidatePred {
        const State* pred;
        int32_t g;
        bool true_cost;
    };

    struct State : public heap_element {
        using lazy_list_type = std::vector<CandidatePred>;

        lazy_list_type  cands;

        const State*    bp;             // current best predecessor
        const State*    ebp;            // best predecessor upon expansion

        int32_t         graph_state;    // graph state
        int32_t         h;              // heuristic value

        int32_t         g;              // current best cost-to-go
        int32_t         eg;             // cost-to-go at upon expansion

        int32_t         call_number;    // scenario when last reinitialized

        bool            true_cost;
        bool            closed;
    };

    ILazySuccFun*           succ_fun_ = nullptr;
    motion::RobotHeuristic* heuristic_ = nullptr;

    std::vector<State*>     states_;
    State*                  start_state_ = nullptr;
    State*                  goal_state_ = nullptr;

    using open_list_type = intrusive_heap<State, StateCompare>;
    open_list_type          open_;

    int32_t                 call_number_    = 0;
    double                  eps_            = 1.0;

    std::vector<int> succs_;
    std::vector<int> costs_;
    std::vector<bool> true_costs_;

    LazyARAStar() : open_(StateCompare{this}) { }
};

} // namespace sbpl
