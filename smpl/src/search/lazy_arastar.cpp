#include <smpl/search/lazy_arastar.h>

#include <smpl/console/console.h>

namespace sbpl {

using State = LazyARAStar::State;
using CandidatePred = LazyARAStar::CandidatePred;

static const int g_infinite = 1000000000;

static const char* LOG = "search";

static auto GetState(LazyARAStar& search, int state_id) -> State*
{
    if (state_id >= (int)search.states_.size()) {
        search.states_.resize(state_id + 1, nullptr);
    }

    if (search.states_[state_id]) {
        return search.states_[state_id];
    }

    State* new_state = new State;
    new_state->cands.clear();
    new_state->graph_state = state_id;
    new_state->h = g_infinite;
    new_state->g = g_infinite;
    new_state->bp = nullptr;
    new_state->true_cost = false;
    new_state->ebp = nullptr;
    new_state->eg = g_infinite;
    new_state->call_number = 0;
    new_state->closed = false;
    search.states_[state_id] = new_state;

    return new_state;
}

static void ReinitState(LazyARAStar& search, State* state) {
    if (state->call_number != search.call_number_) {
        state->cands.clear();

        if (state == search.goal_state_) {
            state->h = 0;
        } else {
            int32_t goal = search.goal_state_->graph_state;
            state->h = search.heuristic_->GetGoalHeuristic(state->graph_state);
        }

        state->g = g_infinite;
        state->bp = nullptr;
        state->true_cost = false;
        state->ebp = nullptr;
        state->eg = g_infinite;
        state->call_number = search.call_number_;
        state->closed = false;
    }
}

static bool IsPredDominated(const CandidatePred& cand, State* state) {
    for (auto it = begin(state->cands); it != end(state->cands); ++it) {
        if (it->pred != cand.pred && it->true_cost && it->g <= cand.g) {
            return true;
        }
    }
    return false;
}

static void ExpandState(LazyARAStar& search, State* state) {
    SMPL_DEBUG_NAMED(LOG, "Expand state %d", state->graph_state);

    state->closed = true;

    state->ebp = state->bp;
    state->eg = state->g;

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

        if (succ_state->closed) {
            continue;
        }

        CandidatePred cand;
        cand.pred = state;
        cand.g = state->g + cost;
        cand.true_cost = true_cost;

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

        // insert/update succ_state in the OPEN list with the
        // values of its best candidate predecessor
        if (!search.open_.contains(succ_state)) {
            search.open_.push(succ_state);
        } else {
            search.open_.update(succ_state);
        }
    }
}

static void EvaluateState(LazyARAStar& search, State* s) {
    assert(!s->true_cost);
    assert(!s->closed);
    assert(!s->cands.empty());
    assert(!m_open.contains(s->open_index));

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
        search.open_.push(s);
    }
}

static void ReconstructPath(
    const LazyARAStar& search,
    std::vector<int>& path,
    int& cost)
{
    for (const State* state = search.goal_state_; state; state = state->ebp) {
        path.push_back(state->graph_state);
    }

    std::reverse(begin(path), end(path));
    cost = search.goal_state_->g;
}

static int ComputeFVal(
    const LazyARAStar& search,
    const State& s)
{
    return s.g + (int)(search.eps_ * (double)s.h);
}

static void Clear(LazyARAStar& search) {
    search.open_.clear();

    for (auto* state : search.states_) {
        delete state;
    }

    search.states_.clear();

    search.start_state_ = nullptr;
    search.goal_state_ = nullptr;
}

bool Init(
    LazyARAStar& search,
    ILazySuccFun* succ_fun,
    motion::RobotHeuristic* heuristic)
{
    if (!succ_fun || !heuristic) {
        return false;
    }

    search.succ_fun_ = succ_fun;
    search.heuristic_ = heuristic;
    return true;
}

int Replan(
    LazyARAStar& search,
    int start_id,
    int goal_id,
    std::vector<int>& solution,
    int& cost)
{
    assert(search.succ_fun_ && search.heuristic_);

    // TODO: lazily initialize search for new/old start state ids
    Clear(search);
    search.call_number_++;

    search.start_state_ = GetState(search, start_id);
    search.goal_state_ = GetState(search, goal_id);
    ReinitState(search, search.start_state_);
    ReinitState(search, search.goal_state_);

    search.start_state_->g = 0;
    search.start_state_->true_cost = true;
    search.open_.push(search.start_state_);

    while (!search.open_.empty()) {
        State* min_state = search.open_.min();
        search.open_.pop();

        int fs = ComputeFVal(search, *min_state);
        if (search.goal_state_->true_cost &&
            ComputeFVal(search, *search.goal_state_) <= fs)
        {
            search.goal_state_->ebp = search.goal_state_->bp;
            search.goal_state_->eg = search.goal_state_->g;

            ReconstructPath(search, solution, cost);
            return 0;
        }

        // a state may come up for expansion/evaluation twice
        if (min_state->closed) {
            continue;
        }

        if (min_state->true_cost) {
            ExpandState(search, min_state);
        } else {
            EvaluateState(search, min_state);
        }
    }

    return 1;
}

bool LazyARAStar::StateCompare::operator()(
    const State& s1,
    const State& s2) const
{
    return ComputeFVal(*search_, s1) < ComputeFVal(*search_, s2);
}

} // namespace sbpl
