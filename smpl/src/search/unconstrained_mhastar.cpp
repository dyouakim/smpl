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

#include <smpl/search/unconstrained_mhastar.h>

namespace sbpl {

UnconstrainedMHAStar::UnconstrainedMHAStar(
    DiscreteSpaceInformation* environment,
    Heuristic* hanchor,
    Heuristic** heurs,
    int hcount)
:
    MultiHeuristicAStarBase(environment, hanchor, heurs, hcount),
    m_max_fval_closed_anc(0)
{
}

void UnconstrainedMHAStar::reinitSearch()
{
    m_max_fval_closed_anc = m_start_state->od[0].f; //0;
}

void UnconstrainedMHAStar::on_closed_anchor(MHASearchState* s)
{
    if (s->od[0].f > m_max_fval_closed_anc) {
        m_max_fval_closed_anc = s->od[0].f;
    }
}

int UnconstrainedMHAStar::priority(MHASearchState* state)
{
    return state->g + m_eps * state->od[0].h;
}

bool UnconstrainedMHAStar::terminated() const
{
    return m_goal_state->g <= m_eps * get_minf(m_open[0]);
}

bool UnconstrainedMHAStar::satisfies_p_criterion(
        MHASearchState* state) const
{
    return true;
}

} // namespace sbpl
