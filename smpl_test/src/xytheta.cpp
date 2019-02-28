#include <cmath>
#include <chrono>

#include <smpl/collision_checker.h>
#include <smpl/occupancy_grid.h>
#include <smpl/robot_model.h>
#include <smpl/search/arastar.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/console/ansi.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/heuristic/attractor_heuristic.h>
#include <smpl/heuristic/joint_dist_heuristic.h>

namespace smpl = sbpl::motion;

template <class CharT, class Traits = std::char_traits<CharT>>
auto donothing(std::basic_ostream<CharT, Traits>& o)
    -> std::basic_ostream<CharT, Traits>&
{ return o; }

const bool g_colorize = true;
#define COLOR(o, color) if (g_colorize) { o << (color); }

/// \brief Defines a Robot Model for an (x, y) point robot
///
/// RobotModel base: basic requirements (variable types and limits)
///
/// ForwardKinematicsInterface: forward kinematics interface required by much
/// of smpl; trivial in this case to establish frame of reference
class KinematicVehicleModel :
//    public virtual smpl::RobotModel,
    public smpl::ForwardKinematicsInterface
{
public:

    KinematicVehicleModel() : smpl::RobotModel(), smpl::ForwardKinematicsInterface()
    {
        const std::vector<std::string> joint_names = { "x", "y" };
        setPlanningJoints(joint_names);
    }

    /// \name Required Public Functions from ForwardKinematicsInterface
    ///@{
    bool computeFK(
        const smpl::RobotState& state,
        const std::string& name,
        std::vector<double>& pose) override
    {
        return computePlanningLinkFK(state, pose);
    }

    bool computePlanningLinkFK(
        const smpl::RobotState& state,
        std::vector<double>& pose) override
    {
        pose = { state[0], state[1], 0.0, 0.0, 0.0, 0.0 };
        return true;
    }
    ///@}

    /// \name Required Public Functions from Robot Model
    ///@{
    double minPosLimit(int jidx) const override { return 0.0; }
    double maxPosLimit(int jidx) const override { return 0.0; }
    bool hasPosLimit(int jidx) const override { return false; }
    bool isContinuous(int jidx) const override { return false; }
    double velLimit(int jidx) const override { return 0.0; }
    double accLimit(int jidx) const override { return 0.0; }

    bool checkJointLimits(
        const smpl::RobotState& angles,
        bool verbose = false) override
    {
        return true;
    }
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override
    {
        if (class_code == smpl::GetClassCode<RobotModel>() ||
            class_code == smpl::GetClassCode<ForwardKinematicsInterface>())
        {
            return this;
        } else {
            return nullptr;
        }
    }
    ///@}
};

/// \brief Defines a collision checker for an (x,y) point robot in a grid world.
class GridCollisionChecker : public smpl::CollisionChecker
{
public:

    GridCollisionChecker(sbpl::OccupancyGrid* grid) :
        Extension(), m_grid(grid)
    { }

    /// \name Required Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override
    {
        if (class_code == smpl::GetClassCode<smpl::CollisionChecker>()) {
            return this;
        }
        return nullptr;
    }
    ///@}

    /// \name Required Functions from CollisionChecker
    ///@{
    bool isStateValid(const smpl::RobotState& state, bool verbose) override;

    bool isStateToStateValid(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        bool verbose) override;

    bool interpolatePath(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        std::vector<smpl::RobotState>& path) override;
    ///@}

private:

    // a bit heavy-weight for this, since it overlays a distance transform
    sbpl::OccupancyGrid* m_grid;
};

bool GridCollisionChecker::isStateValid(
    const smpl::RobotState& state,
    bool verbose)
{
    if (state.size() < 2) {
        SMPL_ERROR("State contains insufficient data");
        return false;
    }
    double x = state[0];
    double y = state[1];
    double z = 0.0;
    if (!m_grid->isInBounds(x, y, z)) {
        SMPL_DEBUG("state (%0.3f, %0.3f) is out of bounds", x, y);
        return false;
    }
    if (m_grid->getDistanceFromPoint(x, y, z) <= 0.0) {
        SMPL_DEBUG("state (%0.3f, %0.3f) is occupied", x, y);
        return false;
    }
    return true;
}

bool GridCollisionChecker::isStateToStateValid(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    bool verbose)
{
    std::vector<smpl::RobotState> path;
    if (!interpolatePath(start, finish, path)) {
        return false;
    }
    return std::all_of(
        path.begin(), path.end(),
        [&](const smpl::RobotState& state)
        {
            return isStateValid(state, false);
        });
}

bool GridCollisionChecker::interpolatePath(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    std::vector<smpl::RobotState>& path)
{
    m_grid->resolution();
    const Eigen::Vector2d vstart(start[0], start[1]);
    const Eigen::Vector2d vfinish(finish[0], finish[1]);
    int num_waypoints =
            (int)std::ceil((vfinish - vstart).norm() / m_grid->resolution());
    num_waypoints = std::max(num_waypoints, 2);
    SMPL_DEBUG("interpolate path with %d waypoints", num_waypoints);
    for (int i = 0; i < num_waypoints; ++i) {
        const double alpha = (double)i / (double)(num_waypoints - 1);
        Eigen::Vector2d vinterm = (1.0 - alpha) * vstart + alpha * vfinish;
        smpl::RobotState istate(2);
        istate[0] = vinterm.x();
        istate[1] = vinterm.y();
        path.push_back(std::move(istate));
    }
    return true;
}

/// Add a simple box obstacle in the center of the grid
void SetupOccupancyGrid(sbpl::OccupancyGrid& grid)
{
    const int x_count = grid.numCellsX();
    const int y_count = grid.numCellsY();
    const int z_count = grid.numCellsZ();

    std::vector<Eigen::Vector3d> points;

    // add horizontal strip down the middle, with holes on the ends
    for (int gx = 1; gx < x_count - 1; ++gx) {
        double cx, cy, cz;
        grid.gridToWorld(gx, y_count >> 1, 0, cx, cy, cz);
        points.emplace_back(cx, cy, cz);

        grid.gridToWorld(gx, (y_count >> 1) + 1, 0, cx, cy, cz);
        points.emplace_back(cx, cy, cz);

        grid.gridToWorld(gx, (y_count >> 1) - 1, 0, cx, cy, cz);
        points.emplace_back(cx, cy, cz);
    }

    SMPL_INFO("Add %zu points to grid", points.size());
    grid.addPointsToField(points);
}

void PrintGrid(std::ostream& o, sbpl::OccupancyGrid& grid)
{
    COLOR(o, sbpl::console::yellow);
    o << '+';
    for (int x = 0; x < grid.numCellsX(); ++x) {
        o << '-';
    }
    o << '+';
    COLOR(o, sbpl::console::reset);
    o << '\n';

    for (int y = grid.numCellsY() - 1; y >= 0; --y) {
        COLOR(o, sbpl::console::yellow);
        o << '|';
        COLOR(o, sbpl::console::reset);
        for (int x = 0; x < grid.numCellsX(); ++x) {
            if (grid.getDistance(x, y, 0) <= 0) {
                COLOR(o, sbpl::console::red);
                o << "X";
                COLOR(o, sbpl::console::reset);
            } else {
                o << " ";
            }
        }
        COLOR(o, sbpl::console::yellow);
        o << '|';
        COLOR(o, sbpl::console::reset);
        o << '\n';
    }

    COLOR(o, sbpl::console::yellow);
    o << '+';
    for (int x = 0; x < grid.numCellsX(); ++x) {
        o << '-';
    }
    o << '+';
    COLOR(o, sbpl::console::reset);
    o << '\n';
}

void PrintActionSpace(const smpl::ManipLatticeActionSpace& aspace)
{
    SMPL_INFO("Action Set:");
    for (int i = 0; i < smpl::MotionPrimitive::Type::NUMBER_OF_MPRIM_TYPES; ++i) {
        smpl::MotionPrimitive::Type prim((smpl::MotionPrimitive::Type)i);
        SMPL_INFO("  %s: %s @ %0.3f", to_cstring(prim), aspace.useAmp(prim) ? "true" : "false", aspace.ampThresh(prim));
    }
    for (auto ait = aspace.begin(); ait != aspace.end(); ++ait) {
        SMPL_INFO("  type: %s", to_cstring(ait->type));
        if (ait->type == sbpl::motion::MotionPrimitive::SNAP_TO_RPY) {
            SMPL_INFO("    enabled: %s", aspace.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_RPY) ? "true" : "false");
            SMPL_INFO("    thresh: %0.3f", aspace.ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_RPY));
        }
        else if (ait->type == sbpl::motion::MotionPrimitive::SNAP_TO_XYZ) {
            SMPL_INFO("    enabled: %s", aspace.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ) ? "true" : "false");
            SMPL_INFO("    thresh: %0.3f", aspace.ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ));
        }
        else if (ait->type == sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY) {
            SMPL_INFO("    enabled: %s", aspace.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY) ? "true" : "false");
            SMPL_INFO("    thresh: %0.3f", aspace.ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY));
        }
        else if (ait->type == sbpl::motion::MotionPrimitive::LONG_DISTANCE ||
                ait->type == sbpl::motion::MotionPrimitive::SHORT_DISTANCE)
        {
            SMPL_INFO_STREAM("    action: " << ait->action);
        }
    }
}

void PrintSolution(
    std::ostream& o,
    const sbpl::OccupancyGrid& grid,
    const std::vector<smpl::RobotState>& path)
{
    std::vector<std::pair<int, int>> discrete_states;
    for (auto& point : path) {
        int dx = (int)(point[0] / grid.resolution());
        int dy = (int)(point[1] / grid.resolution());
        discrete_states.push_back(std::make_pair(dx, dy));
    }

    COLOR(o, sbpl::console::yellow);
    o << '+';
    for (int x = 0; x < grid.numCellsX(); ++x) {
        o << '-';
    }
    o << '+';
    COLOR(o, sbpl::console::reset);
    o << '\n';

    for (int y = grid.numCellsY() - 1; y >= 0; --y) {
        COLOR(o, sbpl::console::yellow);
        o << '|';
        COLOR(o, sbpl::console::reset);
        for (int x = 0; x < grid.numCellsX(); ++x) {
            auto it = std::find(begin(discrete_states), end(discrete_states), std::make_pair(x, y));
            if (it != end(discrete_states)) {
                COLOR(o, sbpl::console::cyan);
                o << 'P';
                COLOR(o, sbpl::console::reset);
            } else {
                if (grid.getDistance(x, y, 0) <= 0) {
                    COLOR(o, sbpl::console::red);
                    o << 'X';
                    COLOR(o, sbpl::console::reset);
                } else {
                    o << " ";
                }
            }
        }
        COLOR(o, sbpl::console::yellow);
        o << '|';
        COLOR(o, sbpl::console::reset);
        o << '\n';
    }

    COLOR(o, sbpl::console::yellow);
    o << '+';
    for (int x = 0; x < grid.numCellsX(); ++x) {
        o << '-';
    }
    o << '+';
    COLOR(o, sbpl::console::reset);
    o << '\n';
}

int main(int argc, char* argv[])
{
    if (argc < 2) {
        printf("Usage: xytheta <mprim filepath>\n");
        return 1;
    }

    const char* mprim_path = argv[1];
    SMPL_INFO("Load motion primitives from %s", mprim_path);

    // 1. Create Robot Model
    KinematicVehicleModel robot_model;

    const double res = 1.0; //0.02; // match resolution of grid and state space

    // 2. Create and Initialize the Environment
    const double grid_res = res;
    const double world_size_x = 50.0;
    const double world_size_y = 50.0;
    const double world_size_z = 1.5 * grid_res;
    const double world_origin_x = 0.0;
    const double world_origin_y = 0.0;
    const double world_origin_z = 0.0;
    const double max_distance_m = 4.0;
    const bool ref_count = false;
    sbpl::OccupancyGrid grid(
            world_size_x, world_size_y, world_size_z,
            grid_res,
            world_origin_x, world_origin_y, world_origin_z,
            max_distance_m,
            ref_count);
    SetupOccupancyGrid(grid);
    PrintGrid(std::cout, grid);

    // 3. Create Collision Checker
    GridCollisionChecker cc(&grid);

    // 4. Define Parameters
    smpl::PlanningParams params;

    // 5. Create Action Space
    smpl::ManipLatticeActionSpace actions;

    // 6. Create Planning Space
    smpl::ManipLattice space;

    // 7. Initialize Manipulation Lattice with RobotModel, CollisionChecker,
    // PlanningParams, variable resolutions, and ActionSpace
    std::vector<double> resolutions = { res, res };
    if (!space.init(&robot_model, &cc, &params, resolutions, &actions)) {
        SMPL_ERROR("Failed to initialize Manip Lattice");
        return 1;
    }

    space.setVisualizationFrameId("map"); // for correct rviz visualization

    // 8. Initialize Manipulation Lattice Action Space

    // associate actions with planning space
    if (!actions.init(&space)) {
        SMPL_ERROR("Failed to initialize Manip Lattice Action Space");
        return 1;
    }

    // load primitives from file, whose path is stored on the param server
    if (!actions.load(mprim_path)) {
        return 1;
    }
//    PrintActionSpace(actions);

    // 9. Create Heuristic
    smpl::JointDistHeuristic h;
    if (!h.init(&space)) {
        SMPL_ERROR("Failed to initialize Joint Dist Heuristic");
        return 1;
    }

    // 10. Associate Heuristic with Planning Space. In this case, Manip Lattice
    // Action Space may use this to determine when to use adaptive motion
    // primitives.
    spact
.insertHeuristic(&h);

    // 11. Create Search, associated with the planning space and heuristic
    sbpl::ARAStar search(&space, &h);

    // 12. Configure Search Behavior
    const double epsilon = 5.0;
    search.set_initialsolution_eps(epsilon);
    search.set_search_mode(false);

    // 13. Set start state and goal condition in the Planning Space and
    // propagate state IDs to search
    double start_x = 0.5 * world_size_x;
    double start_y = 0.33 * world_size_y;
    const smpl::RobotState start_state = space.getDiscreteCenter({ start_x, start_y });

    double goal_x = 0.5 * world_size_x;
    double goal_y = 0.66 * world_size_y;
    const smpl::RobotState goal_state = space.getDiscreteCenter({ goal_x, goal_y });

    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::JOINT_STATE_GOAL;
    goal.angles = goal_state;
    goal.angle_tolerances = { res, res };

    if (!space.setGoal(goal)) {
        SMPL_ERROR("Failed to set goal");
        return 1;
    }

    if (!space.setStart(start_state)) {
        SMPL_ERROR("Failed to set start");
        return 1;
    }

    int start_id = space.getStartStateID();
    if (start_id < 0) {
        SMPL_ERROR("Start state id is invalid");
        return 1;
    }

    int goal_id = space.getGoalStateID();
    if (goal_id < 0)  {
        SMPL_ERROR("Goal state id is invalid");
        return 1;
    }

    if (search.set_start(start_id) == 0) {
        SMPL_ERROR("Failed to set planner start state");
        return 1;
    }

    if (search.set_goal(goal_id) == 0) {
        SMPL_ERROR("Failed to set planner goal state");
        return 1;
    }

    // 14. Plan a path

    ReplanParams search_params(10.0);
    search_params.initial_eps = epsilon;
    search_params.final_eps = 1.0;
    search_params.dec_eps = 0.2;
    search_params.return_first_solution = false;
    search_params.repair_time = 1.0;

    auto then = std::chrono::high_resolution_clock::now();
    std::vector<int> solution;
    int solcost;
    bool bret = search.replan(&solution, search_params, &solcost);
    if (!bret) {
        SMPL_ERROR("Search failed to find a solution");
        return 1;
    }
    auto now = std::chrono::high_resolution_clock::now();
    const double elapsed = std::chrono::duration<double>(now - then).count();

    // 15. Extract path from Planning Space

    std::vector<smpl::RobotState> path;
    if (!space.extractPath(solution, path)) {
        SMPL_ERROR("Failed to extract path");
    }

    SMPL_INFO("Path found!");
    SMPL_INFO("  Planning Time: %0.3f", elapsed);
    SMPL_INFO("  Expansion Count (total): %d", search.get_n_expands());
    SMPL_INFO("  Expansion Count (initial): %d", search.get_n_expands_init_solution());
    SMPL_INFO("  Solution (%zu)", solution.size());
//    for (int id : solution) {
//        SMPL_INFO("    %d", id);
//    }
    SMPL_INFO("  Path (%zu)", path.size());

    PrintSolution(std::cout, grid, path);

//    for (const smpl::RobotState& point : path) {
//        SMPL_INFO("    (x: %0.3f, y: %0.3f)", point[0], point[1]);
//    }

    return 0;
}
