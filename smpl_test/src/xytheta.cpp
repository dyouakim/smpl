#include <cmath>
#include <chrono>

#include <leatherman/print.h>
#include <ros/ros.h>
#include <sbpl/planners/araplanner.h>
#include <smpl/collision_checker.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/heuristic/joint_dist_heuristic.h>
#include <smpl/heuristic/euclid_dist_heuristic.h>
#include <smpl/occupancy_grid.h>
#include <smpl/robot_model.h>
#include <smpl/search/arastar.h>
#include <smpl/search/traplanner.h>
#include <smpl_test/moveObstacle.h>

namespace smpl = sbpl::motion;

/// \brief Defines a Robot Model for an (x, y) point robot
///
/// RobotModel base: basic requirements (variable types and limits)
///
/// ForwardKinematicsInterface: forward kinematics interface required by much
/// of smpl; trivial in this case to establish frame of reference
class KinematicVehicleModel :
    public virtual smpl::RobotModel,
    public virtual smpl::ForwardKinematicsInterface
{
public:

    KinematicVehicleModel() :
        smpl::RobotModel(),
        smpl::ForwardKinematicsInterface()
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

    void changeOccupancyGrid(sbpl::OccupancyGrid* grid)
    {
            m_grid = grid;
    }
    /// \name Required Functions from CollisionChecker
    ///@{
    bool isStateValid(
        const smpl::RobotState& state,
        bool verbose,
        bool visualize,
        double& dist) override;

    bool isStateToStateValid(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        int& path_length,
        int& num_checks,
        double& dist) override;

    bool interpolatePath(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        std::vector<smpl::RobotState>& path) override;
    ///@}

    /// \name Reimplemented Functions from CollisionChecker
    ///@{
    visualization_msgs::MarkerArray getCollisionModelVisualization(
        const smpl::RobotState& state)
    { return visualization_msgs::MarkerArray(); }

    visualization_msgs::MarkerArray getVisualization(
        const smpl::RobotState& state)
    { return visualization_msgs::MarkerArray(); }
    ///@}

private:

    // a bit heavy-weight for this, since it overlays a distance transform
    sbpl::OccupancyGrid* m_grid;
};

bool GridCollisionChecker::isStateValid(
    const smpl::RobotState& state,
    bool verbose,
    bool visualize,
    double& dist)
{
    if (state.size() < 2) {
        ROS_ERROR("State contains insufficient data");
        return false;
    }
    double x = state[0];
    double y = state[1];
    double z = 0.0;
    if (!m_grid->isInBounds(x, y, z)) {
        ROS_DEBUG("state (%0.3f, %0.3f) is out of bounds", x, y);
        return false;
    }
    if (m_grid->getDistanceFromPoint(x, y, z) <= 0.0) {
        ROS_DEBUG("state (%0.3f, %0.3f) is occupied", x, y);
        return false;
    }
    return true;
}

bool GridCollisionChecker::isStateToStateValid(
    const smpl::RobotState& start,
    const smpl::RobotState& finish,
    int& path_length,
    int& num_checks,
    double& dist)
{
    std::vector<smpl::RobotState> path;
    if (!interpolatePath(start, finish, path)) {
        return false;
    }
    return std::all_of(
        path.begin(), path.end(),
        [&](const smpl::RobotState& state)
        {
            double dist;
            return isStateValid(state, false, false, dist);
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
            std::ceil((vfinish - vstart).norm() / m_grid->resolution());
    num_waypoints = std::max(num_waypoints, 2);
    ROS_DEBUG("interpolate path with %d waypoints", num_waypoints);
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
void SetupOccupancyGrid(sbpl::OccupancyGrid& grid, int xStart, int xEnd, int yIndex)
{
    const int x_count = grid.numCellsX();
    const int y_count = grid.numCellsY();
    const int z_count = grid.numCellsZ();

    //ROS_ERROR_STREAM((y_count>>1));
    
    std::vector<Eigen::Vector3d> points;

    // add horizontal strip down the middle, with holes on the ends
    for (int gx = 0; gx <= x_count - 1; ++gx) 
    {
        double cx, cy, cz;
        if(gx>=xStart && gx<=xEnd)
        {
            grid.gridToWorld(gx, yIndex >> 1, 0, cx, cy, cz);
            points.emplace_back(cx, cy, cz);
                
            grid.gridToWorld(gx, (yIndex >> 1) - 1, 0, cx, cy, cz);
            points.emplace_back(cx, cy, cz);
        }
        /*else
        {
            grid.gridToWorld(gx, (ySize >> 1) - 1, 0, cx, cy, cz);
            points.emplace_back(cx, cy, cz);
        }*/
    }

    ROS_INFO("Add %zu points to grid", points.size());
    grid.addPointsToField(points);
}


void PrintGrid(std::ostream& o, sbpl::OccupancyGrid& grid)
{
    for (int y = grid.numCellsY() - 1; y >= 0; --y) {
        for (int x = 0; x < grid.numCellsX(); ++x) {
            double d = grid.getDistance(x, y, 0);
            
            /*if((grid.sizeX()-sX)/grid.resolution()==x && ((int)((grid.sizeY()-sY)/grid.resolution()))==y)
                o<<"-1 ";
            else if((grid.sizeX()-gX)/grid.resolution()==x && ((int)((grid.sizeY()-gY)/grid.resolution()))==y)
            {
                ROS_ERROR("Goal");
                o<<"2 ";
            }
            else */if (d <= 0) {
                o << "1 ";
            } 
            else {
                o << "0 ";
            }
        }
        o << '\n';
    }
}


void PrintPath(std::ostream& o, sbpl::OccupancyGrid& grid, std::vector<smpl::RobotState> path)
{
    std::vector<int> pathX;
    std::vector<int> pathY;

    for(int i=0;i<path.size();i++)
    {
        smpl::RobotState state = path[i]; 
        pathX.push_back((int)((state[0])/grid.resolution()));
        pathY.push_back((int)((state[1])/grid.resolution()));
    }
    for (int y = grid.numCellsY() - 1; y >= 0; --y) 
    {
        for (int x = 0; x < grid.numCellsX(); ++x) 
        {
            double d = grid.getDistance(x, y, 0);
            bool flag = false;
            for(int i=0;i<pathX.size();i++)
            {
                if(x==pathX[i] && y==pathY[i])
                {
                   // ROS_ERROR_STREAM("Path X Y"<<pathX[i]<<","<<pathY[i]);
               
                    flag = true;
                    o<<i<<" ";
                }
                
            } 
            if(!flag)
                if (d <= 0 ) 
                {
                    o << "1 ";
                }    
                else 
                {
                    o << "0 ";
                }
        }
        o << '\n';
    }
}

void PrintActionSpace(const smpl::ManipLatticeActionSpace& aspace)
{
    ROS_INFO("Action Set:");
    for (int i = 0; i < smpl::MotionPrimitive::Type::NUMBER_OF_MPRIM_TYPES; ++i) {
        smpl::MotionPrimitive::Type prim((smpl::MotionPrimitive::Type)i);
        ROS_INFO("  %s: %s @ %0.3f", to_string(prim).c_str(), aspace.useAmp(prim) ? "true" : "false", aspace.ampThresh(prim));
    }
    for (auto ait = aspace.begin(); ait != aspace.end(); ++ait) {
        ROS_INFO("  type: %s", to_string(ait->type).c_str());
        if (ait->type == sbpl::motion::MotionPrimitive::SNAP_TO_RPY) {
            ROS_INFO("    enabled: %s", aspace.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_RPY) ? "true" : "false");
            ROS_INFO("    thresh: %0.3f", aspace.ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_RPY));
        }
        else if (ait->type == sbpl::motion::MotionPrimitive::SNAP_TO_XYZ) {
            ROS_INFO("    enabled: %s", aspace.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ) ? "true" : "false");
            ROS_INFO("    thresh: %0.3f", aspace.ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ));
        }
        else if (ait->type == sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY) {
            ROS_INFO("    enabled: %s", aspace.useAmp(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY) ? "true" : "false");
            ROS_INFO("    thresh: %0.3f", aspace.ampThresh(sbpl::motion::MotionPrimitive::SNAP_TO_XYZ_RPY));
        }
        else if (ait->type == sbpl::motion::MotionPrimitive::LONG_DISTANCE ||
                ait->type == sbpl::motion::MotionPrimitive::SHORT_DISTANCE)
        {
            ROS_INFO("    action: %s", to_string(ait->action).c_str());
        }
    }
}

bool moveObstacle(
        smpl_test::moveObstacle::Request &req,
        smpl_test::moveObstacle::Response &res) 
{
    ROS_ERROR("Here in the service");
    //search->force_planning_from_scratch_and_free_memory();
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "xytheta");
    
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");


    ros::ServiceServer moveObstacleSrv = nh.advertiseService("/moveObstacle", &moveObstacle);

    // 1. Instantiate Robot Model
    KinematicVehicleModel robot_model;

    // 2. Instantiate the Environment
    const double grid_res = 0.1;
    const double world_size_x = 0.8;
    const double world_size_y = 0.8;
    const double world_size_z = 1.5 * grid_res;
    const double world_origin_x = 0.0;
    const double world_origin_y = 0.0;
    const double world_origin_z = 0.0;
    const double max_distance_m = 0.2;
    const bool ref_count = false;
    sbpl::OccupancyGrid* grid = new sbpl::OccupancyGrid(
            world_size_x, world_size_y, world_size_z,
            grid_res,
            world_origin_x, world_origin_y, world_origin_z,
            max_distance_m,
            ref_count);
   

    GridCollisionChecker cc(grid);

    // 3. Define Parameters
    smpl::PlanningParams params;
    // 4. Instantiate Planning Space
    auto pspace =
            std::make_shared<smpl::ManipLattice>(&robot_model, &cc, &params);

    //        auto pspace = std::make_shared<WorkspaceLattice>(&robot_model, &cc, &paramss);

    if (!pspace->init({ 0.02, 0.02 })) {
        ROS_ERROR("Failed to initialize Manip Lattice");
        return 1;
    }
    pspace->setVisualizationFrameId("map");

    // 5. Instantiate and Initialize Motion Primitives
    std::string mprim_path;
    if (!ph.getParam("mprim_path", mprim_path)) {
        ROS_ERROR("Failed to retrieve 'mprim_path' from the param server");
        return 1;
    }

    auto aspace = std::make_shared<smpl::ManipLatticeActionSpace>(pspace.get());
    
    if (!aspace->load(mprim_path)) {
        return 1;
    }
    PrintActionSpace(*aspace);
    // 6. Associate Action Space with Planning Space
    pspace->setActionSpace(aspace);

    // 7. Instantiate Heuristic
    auto h = std::make_shared<smpl::JointDistHeuristic>(pspace, grid);

    // 8. Associate Heuristic with Planning Space (for adaptive motion
    // primitives)
    pspace->insertHeuristic(h.get());

    // 9. Instantiate and Initialize Search (associated with Planning Space)
    const bool forward = true;
    auto search = std::make_shared<sbpl::TRAPlanner>(pspace.get(), h.get(), forward);

    const double epsilon = 5.0;
    search->set_initialsolution_eps(epsilon);
    search->set_search_mode(false);

    // 10. Set start state and goal condition in the Planning Space and
    // propagate state IDs to search
    double start_x = 0.2;
    double start_y = 0.2;
    const smpl::RobotState start_state = { start_x, start_y };

    double goal_x = 0.6;
    double goal_y = 0.6;
    const smpl::RobotState goal_state = { goal_x, goal_y };

    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::JOINT_STATE_GOAL;
    goal.angles = goal_state;
    goal.angle_tolerances = { 0.02, 0.02 };

    if (!pspace->setGoal(goal)) {
        ROS_ERROR("Failed to set goal");
        return 1;
    }

    if (!pspace->setStart(start_state)) {
        ROS_ERROR("Failed to set start");
        return 1;
    }

    int start_id = pspace->getStartStateID();

    if (start_id < 0) {
        ROS_ERROR("Start state id is invalid");
        return 1;
    }

    int goal_id = pspace->getGoalStateID();
    if (goal_id < 0)  {
        ROS_ERROR("Goal state id is invalid");
        return 1;
    }
   
    if (search->set_start(start_id) == 0) {
        ROS_ERROR("Failed to set planner start state");
        return 1;
    }

    if (search->set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return 1;
    }


    SetupOccupancyGrid(*grid,3,5,8);
    PrintGrid(std::cout, *grid);

     // 11. Plan a path

    ReplanParams search_params(10.0);
    search_params.initial_eps = epsilon;
    search_params.final_eps = 1.0;
    search_params.dec_eps = 0.2;
    search_params.return_first_solution = false;
    search_params.repair_time = 1.0;

    auto then = std::chrono::high_resolution_clock::now();
    std::vector<int> solution;
    int solcost;
    bool bret = search->replan(&solution, search_params, &solcost);
    if (!bret) {
        ROS_ERROR("Search failed to find a solution");
        return 1;
    }

    auto now = std::chrono::high_resolution_clock::now();
    const double elapsed = std::chrono::duration<double>(now - then).count();

    // 12. Extract path from Planning Space

    std::vector<smpl::RobotState> path;
    std::vector<geometry_msgs::PoseStamped> eePath;
    if (!pspace->extractPath(solution, path, eePath)) {
        ROS_ERROR("Failed to extract path");
    }

    ROS_INFO("Path found!");
    ROS_INFO("  Planning Time: %0.3f", elapsed);
    ROS_INFO("  Expansion Count (total): %d", search->get_n_expands());
    ROS_INFO("  Expansion Count (initial): %d", search->get_n_expands_init_solution());
    ROS_INFO("  Solution (%zu)", solution.size());
    for (int id : solution) {
        ROS_INFO("    %d", id);
    }
    ROS_INFO("  Path (%zu)", path.size());
    for (const smpl::RobotState& point : path) {
        ROS_INFO("    (x: %0.3f, y: %0.3f)", point[0], point[1]);
    }
    PrintPath(std::cout,*grid,path);

    /*grid = new sbpl::OccupancyGrid(
            world_size_x, world_size_y, world_size_z,
            grid_res,
            world_origin_x, world_origin_y, world_origin_z,
            max_distance_m,
            ref_count);


    SetupOccupancyGrid(*grid,23,27,48);

     cc.changeOccupancyGrid(grid);

    solution.clear();
    solcost = 0;
    path.clear();
    eePath.clear();
    search->costs_changed();
    //search->force_planning_from_scratch();
    bret = search->replan(&solution, search_params, &solcost);
    if (!bret) {
        ROS_ERROR("Search failed to find a solution");
        return 1;
    }
    
    now = std::chrono::high_resolution_clock::now();
    const double elapsed2 = std::chrono::duration<double>(now - then).count();

    // 12. Extract path from Planning Space

     if (!pspace->extractPath(solution, path, eePath)) {
        ROS_ERROR("Failed to extract path");
    }

    ROS_INFO("Path found!");
    ROS_INFO("  Planning Time: %0.3f", elapsed2);
    ROS_INFO("  Expansion Count (total): %d", search->get_n_expands());
    ROS_INFO("  Expansion Count (initial): %d", search->get_n_expands_init_solution());
    ROS_INFO("  Solution (%zu)", solution.size());
    for (int id : solution) {
        ROS_INFO("    %d", id);
    }
    ROS_INFO("  Path (%zu)", path.size());
    for (const smpl::RobotState& point : path) {
        ROS_INFO("    (x: %0.3f, y: %0.3f)", point[0], point[1]);
    }
    PrintPath(std::cout,*grid,path);*/
    return 0;
}


