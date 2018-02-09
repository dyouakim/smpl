#ifndef SMPL_MULTI_BFS_HEURISTIC_H
#define SMPL_MULTI_BFS_HEURISTIC_H

// standard includes
#include <memory>

// project includes
#include <smpl/occupancy_grid.h>
#include <smpl/bfs3d/bfs3d.h>
#include <smpl/debug/marker.h>
#include <smpl/heuristic/robot_heuristic.h>

#include <tf/transform_listener.h>

#include <tf_conversions/tf_eigen.h>
#include <smpl/debug/visualize.h>

namespace sbpl {
namespace motion {

class MultiBfsHeuristic : public RobotHeuristic
{
public:

    virtual ~MultiBfsHeuristic();

    bool init(RobotPlanningSpace* space, const OccupancyGrid* grid);

    double inflationRadius() const { return m_inflation_radius; }
    void setInflationRadius(double radius);
    double baseInflationRadius() const { return m_base_inflation_radius; }
    void setBaseInflationRadius(double radius);
    
    int costPerCell() const { return m_cost_per_cell; }
    void setCostPerCell(int cost);

    auto grid() const -> const OccupancyGrid* { return m_grid; }

    auto getWallsVisualization() const -> visual::Marker;
    auto getValuesVisualization() -> visual::Marker;

    /// \name Required Public Functions from RobotHeuristic
    ///@{
    double getMetricStartDistance(double x, double y, double z);
    double getMetricGoalDistance(double x, double y, double z);
    double getMetricGoalDistance(double x, double y, double z, GroupType planning_group);
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Reimplemented Public Functions from RobotPlanningSpaceObserver
    ///@{
    void updateGoal(const GoalConstraint& goal);
    ///@}

    /// \name Required Public Functions from Heuristic
    ///@{
    int GetGoalHeuristic(int state_id);
    int GetGoalHeuristic(int state_id, int planning_group, int base_heuristic_idx);
    int GetStartHeuristic(int state_id);
    int GetFromToHeuristic(int from_id, int to_id); 
    ///@}

private:

    const OccupancyGrid* m_grid = nullptr;

    std::vector<std::unique_ptr<BFS_3D>> m_bfs;
    PointProjectionExtension* m_pp = nullptr;

    double m_inflation_radius = 0.0;
    double m_base_inflation_radius = 0.0;
    int m_cost_per_cell = 1;

    int m_goal_x = -1;
    int m_goal_y = -1;
    int m_goal_z = -1;

    void syncGridAndBfs();
    //int getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z,GroupType planning_group) const;
    int getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const;

};

} // namespace motion
} // namespace sbpl

#endif
