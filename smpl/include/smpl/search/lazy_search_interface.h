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

#include <vector>

namespace sbpl {

struct ISuccFun {
    virtual ~ISuccFun() { }

    virtual void GetSuccs(
        int state_id,
        std::vector<int>& succs,
        std::vector<int>& costs) = 0;
};

struct IPredFun {
    virtual ~IPredFun() { }

    virtual void GetPreds(
        int state_id,
        std::vector<int>& preds,
        std::vector<int>& costs) = 0;
};

struct ILazySuccFun {
    virtual ~ILazySuccFun() { }

    virtual void GetLazySuccs(
        int state_id,
        std::vector<int>& succs,
        std::vector<int>& costs,
        std::vector<bool>& true_costs) = 0;

    virtual int GetSuccTrueCost(int state_id, int succ_id) = 0;
};

struct ILazyPredFun {
    virtual ~ILazyPredFun() { }

    virtual void GetLazyPreds(
        std::vector<int>& preds,
        std::vector<int>& costs,
        std::vector<bool>& true_costs) = 0;

    virtual int GetPredTrueCost(int state_id, int pred_id) = 0;
};

} // namespace sbpl
