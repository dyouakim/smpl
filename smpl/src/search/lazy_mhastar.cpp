#include <smpl/search/lazy_mhastar.h>

#include <smpl/console/console.h>

namespace sbpl {

using State = LazySMHAStar::State;
using StateCompare = LazySMHAStar::StateCompare;
using CandidatePred = LazySMHAStar::CandidatePred;

static const int g_infinite = 1000000000;
static const double g_w_mha = 5.0;

static const char* LOG = "search";

static size_t NumHeuristics(LazySMHAStar& search)
{
    return search.h_count_ + 1;
}

static State* GetStateFromHeapData(State::HeapData* d)
{
    return (State*)((char*)d - d->off);
}

static const State* GetStateFromHeapData(const State::HeapData* d)
{
    return (State*)((char*)d - d->off);
}

static int ComputeFVal(const LazySMHAStar& search, const State::HeapData& d)
{
    const State* s = GetStateFromHeapData(&d);
    return s->g + (int)(search.eps_ * (double)d.h);
}

static auto GetState(LazySMHAStar& search, int state_id) -> State*
{
    if (state_id >= (int)search.states_.size()) {
        search.states_.resize(state_id + 1, nullptr);
    }

    if (search.states_[state_id]) {
        return search.states_[state_id];
    }

    // size of the state, overallocated to store information for n additional
    // heuristics
    size_t state_size =
            sizeof(State) +
            sizeof(State::HeapData) * search.h_count_;

    State* state = (State*)malloc(state_size);

    // default construct the State since there are non-pod types in State
    new (state) State;

    // we only need to initialize these variables, as their values are
    // meaningful between searches, the rest will be initialized in ReinitState
    // during each search invocation.
    state->graph_state = state_id;
    state->call_number = 0;

    // initialize offset from each open data to the start of the state
    for (size_t i = 0; i < search.h_count_; ++i) {
        state->open_data[i].off = ((char*)&state->open_data[0] - (char*)state);
    }

    search.states_[state_id] = state;

    return state;
}

static void ReinitState(LazySMHAStar& search, State* state)
{
    if (state->call_number != search.call_number_) {
        state->cands.clear();

        state->bp = nullptr;
        state->ebp = nullptr;

        state->g = g_infinite;
        state->eg = g_infinite;

        state->call_number = search.call_number_;

        state->true_cost = false;
        state->closed_in_anc = false;
        state->closed_in_add = false;

        for (size_t i = 0; i < search.h_count_; ++i) {
            if (i == 0) {
                state->open_data[i].h = search.h_anchor_->GetGoalHeuristic(state->graph_state);
            } else {
                state->open_data[i].h = search.h_others_[i - 1]->GetGoalHeuristic(state->graph_state);
            }
        }
    }
}

// Test whether an existing predecessor candidate dominates a given candidate.
static bool IsPredDominated(const CandidatePred& cand, State* state)
{
    for (auto& c : state->cands) {
        if (c.pred != cand.pred && c.true_cost && c.g <= cand.g) {
            return true;
        }
    }
    return false;
}

static void RemoveFromAllOpenLists(LazySMHAStar& search, State* state)
{
    for (size_t hidx = 0; hidx < search.h_count_ + 1; ++hidx) {
        if (search.open_lists_[hidx].contains(&state->open_data[hidx])) {
            search.open_lists_[hidx].erase(&state->open_data[hidx]);
        }
    }
}

static void InsertOrUpdateInOpenList(
    LazySMHAStar& search,
    State* state,
    size_t hidx)
{
    if (search.open_lists_[hidx].contains(&state->open_data[hidx])) {
        search.open_lists_[hidx].push(&state->open_data[hidx]);
    } else {
        search.open_lists_[hidx].update(&state->open_data[hidx]);
    }
}

static void ExpandState(LazySMHAStar& search, State* state, size_t hidx)
{
    SMPL_DEBUG_NAMED(LOG, "Expand state %d", state->graph_state);

    assert(!closed_in_add_search(state) || !closed_in_anc_search(state));

    if (hidx == 0) {
        state->closed_in_anc = true;
    } else {
        state->closed_in_add = true;
    }

    state->ebp = state->bp;
    state->eg = state->g;

    // remove s from all open lists
    RemoveFromAllOpenLists(search, state);

    search.succs_.clear();
    search.costs_.clear();
    search.true_costs_.clear();
    search.succ_fun_->GetLazySuccs(
            state->graph_state,
            search.succs_,
            search.costs_,
            search.true_costs_);

    assert(search.succs.size() == search.costs.size());
    assert(search.succs.size() == search.true_costs.size());

    for (size_t i = 0; i < search.succs_.size(); ++i) {
        int succ_id = search.succs_[i];
        int cost = search.costs_[i];
        bool true_cost = search.true_costs_[i];

        State* succ_state = GetState(search, succ_id);
        ReinitState(search, succ_state);

        // nothing to do, closed in anchor search
        if (succ_state->closed_in_anc) {
            continue;
        }

        CandidatePred cand;
        cand.pred = state;
        cand.g = state->g + cost;
        cand.true_cost = true_cost;

        // "value" of state (or potential values) not updated
        if (IsPredDominated(cand, succ_state)) {
            continue;
        }

        auto& cands = succ_state->cands;
        cands.push_back(cand);

        auto better_cand = [&](const CandidatePred& a, const CandidatePred& b) {
            return a.g < b.g;
        };

        auto best_it = std::min_element(begin(cands), end(cands), better_cand);
        assert(best_it != end(cands));

        succ_state->bp = best_it->pred;
        succ_state->g = best_it->g;
        succ_state->true_cost = best_it->true_cost;

        InsertOrUpdateInOpenList(search, succ_state, 0);

        auto fanchor = ComputeFVal(search, succ_state->open_data[0]);

        if (!succ_state->closed_in_add) {
            for (size_t i = 1; i < NumHeuristics(search); ++i) {
                auto fn = ComputeFVal(search, succ_state->open_data[i]);
                if (fn <= g_w_mha * fanchor) {
                    InsertOrUpdateInOpenList(search, succ_state, i);
                }
            }
        }
    }
}

static void EvaluateState(LazySMHAStar& search, State* s, size_t hidx)
{
    assert(!s->true_cost);
    assert(!s->closed);
    assert(!s->cands.empty());
    assert(!m_open.contains(s->open_index));

    //

    // get the best candidate
    auto& cands = s->cands;
    auto better_cand = [&](const CandidatePred& a, const CandidatePred& b) {
        return a.g < b.g;
    };
    auto best_it = std::min_element(begin(cands), end(cands), better_cand);

    assert(!best_it->true_cost);

    SMPL_DEBUG_NAMED(LOG, "Evaluate transitions %d -> %d", s->bp->graph_state, s->graph_state);

    int32_t cost = search.succ_fun_->GetSuccTrueCost(
            s->bp->graph_state, s->graph_state);

    // remove invalid or now-dominated candidate preds
    if (cost < 0) {
        cands.erase(best_it);
    } else {
        best_it->true_cost = true;
        best_it->g = best_it->pred->g + cost;
        if (IsPredDominated(*best_it, s)) {
            cands.erase(best_it);
        }
    }

    best_it = std::min_element(begin(cands), end(cands), better_cand);
    s->bp = best_it->pred;
    s->g = best_it->g;
    s->true_cost = best_it->true_cost;

    // OPTIMIZATION if this element is the best, remove all elements except this
    // one from the lazy list. Also, we can probably also remove this element
    // and maintain the s's (bp,g,true) as the current best candidate

    if (best_it != end(cands)) {
        search.open_lists_[hidx].push(&s->open_data[hidx]);
    }
}

static void ReconstructPath(
    const LazySMHAStar& search,
    std::vector<int>& path,
    int& cost)
{
    for (const State* state = search.goal_state_; state; state = state->ebp) {
        path.push_back(state->graph_state);
    }

    std::reverse(begin(path), end(path));
    cost = search.goal_state_->g;
}

static void Clear(LazySMHAStar& search)
{
    for (size_t hidx = 0; hidx < search.h_count_ + 1; ++hidx) {
        search.open_lists_[hidx].clear();
    }

    for (auto* state : search.states_) {
        delete state;
    }

    search.states_.clear();

    search.start_state_ = nullptr;
    search.goal_state_ = nullptr;
}

bool Init(
    LazySMHAStar& search,
    ILazySuccFun* succ_fun,
    motion::RobotHeuristic* h_anchor,
    motion::RobotHeuristic* h_others[],
    size_t h_count)
{
    if (!succ_fun || !h_anchor || !h_anchor || !h_others) {
        SMPL_ERROR("Search component is null");
        return false;
    }

    for (size_t i = 0; i < h_count; ++i) {
        if (!h_others[i]) {
            SMPL_ERROR("Additional heuristic %zu is null", i);
            return false;
        }
    }

    search.succ_fun_ = succ_fun;
    search.h_anchor_ = h_anchor;
    search.h_others_ = h_others;
    search.h_count_ = h_count;

    search.open_lists_.reserve(h_count + 1);
    for (size_t i = 0; i < h_count + 1; ++i) {
        search.open_lists_.emplace_back(StateCompare{&search});
    }
    return true;
}

int Replan(
    LazySMHAStar& search,
    int start_id,
    int goal_id,
    std::vector<int>& solution,
    int& cost)
{
    assert(search.succ_fun_ && search.h_anchor_);

    // TODO: lazily initialize search for new/old start state ids
    Clear(search);
    search.call_number_++;

    search.start_state_ = GetState(search, start_id);
    search.goal_state_ = GetState(search, goal_id);
    ReinitState(search, search.start_state_);
    ReinitState(search, search.goal_state_);

    search.start_state_->g = 0;
    search.start_state_->true_cost = true;
    for (size_t hidx = 0; hidx < NumHeuristics(search); ++hidx) {
        search.open_lists_[hidx].push(&search.start_state_->open_data[hidx]);
        SMPL_DEBUG("Inserted start state %d into search %zu with f = %d",
                start_id,
                hidx,
                ComputeFVal(search, search.start_state_->open_data[hidx]));
    }

    auto& anchor_list = search.open_lists_[0];

    size_t hidx = 0; // initialize to 0, first value => 1
    while (!anchor_list.empty()) {
        if (anchor_list.empty()) {
            break;
        }

        // cycle to the next queue
        hidx %= search.h_count_;
        hidx++;

        auto* anchor_min = GetStateFromHeapData(anchor_list.min());
        auto anchor_fmin = ComputeFVal(search, anchor_min->open_data[0]);

        auto& open = search.open_lists_[hidx];
        if (!open.empty()) {
            auto* open_min = GetStateFromHeapData(open.min());

            auto fmin = ComputeFVal(search, open_min->open_data[hidx]);
            if (fmin <= g_w_mha * anchor_fmin) {
                if (search.goal_state_->true_cost &&
                    search.goal_state_->g <= fmin)
                {
                    search.goal_state_->ebp = search.goal_state_->bp;
                    search.goal_state_->eg = search.goal_state_->eg;
                    ReconstructPath(search, solution, cost);
                    return 0;
                }

                if (open_min->closed_in_add) {
                    RemoveFromAllOpenLists(search, open_min);
                    InsertOrUpdateInOpenList(search, open_min, 0);
                    continue;
                }

                if (open_min->true_cost) {
                    ExpandState(search, open_min, hidx);
                } else {
                    EvaluateState(search, open_min, hidx);
                }
            } else {
                // expand from anchor search
                if (search.goal_state_->g <= anchor_fmin &&
                    search.goal_state_->true_cost)
                {
                    search.goal_state_->ebp = search.goal_state_->bp;
                    search.goal_state_->eg = search.goal_state_->eg;
                    ReconstructPath(search, solution, cost);
                    return 0;
                }

                if (anchor_min->closed_in_anc) {
                    RemoveFromAllOpenLists(search, anchor_min);
                    continue;
                }

                if (anchor_min->true_cost) {
                    ExpandState(search, anchor_min, hidx);
                } else {
                    EvaluateState(search, anchor_min, hidx);
                }
            }
        } else {
            // expand from anchor search
            if (search.goal_state_->g <= anchor_fmin &&
                search.goal_state_->true_cost)
            {
                search.goal_state_->ebp = search.goal_state_->bp;
                search.goal_state_->eg = search.goal_state_->eg;
                ReconstructPath(search, solution, cost);
                return 0;
            }

            if (anchor_min->closed_in_anc) {
                RemoveFromAllOpenLists(search, anchor_min);
                continue;
            }

            if (anchor_min->true_cost) {
                ExpandState(search, anchor_min, hidx);
            } else {
                EvaluateState(search, anchor_min, hidx);
            }
        }
    }

    if (anchor_list.empty()) {
        SMPL_INFO("Exhausted anchor search");
        return 1;
    }

    return 1;
}

bool LazySMHAStar::StateCompare::operator()(
    const State::HeapData& s1,
    const State::HeapData& s2) const
{
    return ComputeFVal(*search_, s1) < ComputeFVal(*search_, s2);
}

} // namespace sbpl
