#include <smpl/heuristic/multi_bfs_heuristic.h>

// project includes
#include <smpl/bfs3d/bfs3d.h>
#include <smpl/console/console.h>
#include <smpl/intrusive_heap.h>
#include <smpl/grid.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>

namespace sbpl {
namespace motion {

static const char* LOG = "heuristic.multi_bfs";

MultiBfsHeuristic::~MultiBfsHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool MultiBfsHeuristic::init(RobotPlanningSpace* space, const OccupancyGrid* grid)
{
    if (!grid) {
        return false;
    }

    if (!RobotHeuristic::init(space)) {
        return false;
    }

    m_grid = grid;

    m_pp = space->getExtension<PointProjectionExtension>();
    if (m_pp) {
        SMPL_INFO_NAMED(LOG, "Got Point Projection Extension!");
    }
    syncGridAndBfs();
    return true;
}

void MultiBfsHeuristic::setInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

void MultiBfsHeuristic::setBaseInflationRadius(double radius)
{
    m_base_inflation_radius = radius;
}

void MultiBfsHeuristic::setCostPerCell(int cost_per_cell)
{
    m_cost_per_cell = cost_per_cell;
}

void MultiBfsHeuristic::updateGoal(const GoalConstraint& goal)
{
    int gx, gy, gz;
    grid()->worldToGrid(
            goal.tgt_off_pose[0], goal.tgt_off_pose[1], goal.tgt_off_pose[2],
            gx, gy, gz);

    SMPL_DEBUG_NAMED(LOG, "Setting the BFS heuristic goal (%d, %d, %d)", gx, gy, gz);

    if (!m_bfs[4]->inBounds(gx, gy, gz)) {
        SMPL_ERROR_NAMED(LOG, "Heuristic goal is out of BFS bounds");
    }

    m_goal_x = gx;
    m_goal_y = gy;
    m_goal_z = gz;

    m_bfs[4]->run(gx, gy, gz);

    SMPL_WARN_STREAM("Origin_arm "<<gx<<","<<gy<<","<<gz);
     if (!m_pp) {
        return ;
    }
    
    
    /*tf::StampedTransform transform;
    tf::TransformListener listener;
    Eigen::Vector3d velInBodyFrame;
  
    try
    {
      listener.waitForTransform("/arm_base_link_yaw","/world",ros::Time(0),ros::Duration(0.2));

      listener.lookupTransform("/arm_base_link_yaw","/world",ros::Time(0),transform);
    }
   catch (tf::TransformException ex){
     ROS_ERROR("%s",ex.what());
     ros::Duration(1.0).sleep();
   }

   SMPL_WARN_STREAM("transform "<<transform.getOrigin()[0]<<","<<transform.getOrigin()[1]<<","<<transform.getOrigin()[2]);
    
    Eigen::Vector3d  translation_from_origin (transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
    Eigen::Translation< double, 3 > translation( translation_from_origin );
    tf::Quaternion q = transform.getRotation();
    Eigen::Quaterniond  rotation_from_origen (q.w(),q.x(),q.y(),q.z()); 

    Eigen::Transform< double, 3, Eigen::Affine > T = translation * rotation_from_origen;
   
    Eigen::Vector3d velTransformationVector (goal.pose[0],goal.pose[1],goal.pose[2]);
     // Create an homogeneous vector and multiply it by the TF
    Eigen::Vector4d velTransformationVectorHomogeneous = velTransformationVector.homogeneous();

    velTransformationVectorHomogeneous = T * velTransformationVectorHomogeneous;
    
    int origin_gz;
    //velInBodyFrame = aux*velTransformationVector;
    grid()->worldToGrid(velTransformationVectorHomogeneous[0],velTransformationVectorHomogeneous[1],velTransformationVectorHomogeneous[2],gx, gy, origin_gz);
    SMPL_WARN_STREAM("Origin5 before "<<velTransformationVectorHomogeneous[0]<<","<<velTransformationVectorHomogeneous[1]<<","<<velTransformationVectorHomogeneous[2]<<" after "<<gx<<","<<gy<<","<<origin_gz);

    double base_origin_x = goal.pose[0];
    double base_origin_y = goal.pose[1]-0.05;
    double base_origin_z = goal.pose[2]-0.4;
    grid()->worldToGrid(base_origin_x+0.2,base_origin_y,base_origin_z,gx, gy, gz);
    m_bfs[0]->run(gx,gy,gz);
    SMPL_WARN_STREAM("Origin1 "<<gx<<","<<gy<<","<<gz);
    grid()->worldToGrid(base_origin_x-0.2,base_origin_y,base_origin_z,gx, gy, gz);
    m_bfs[1]->run(gx,gy,gz);
    SMPL_WARN_STREAM("Origin2 "<<gx<<","<<gy<<","<<gz);
    grid()->worldToGrid(base_origin_x-0.2,base_origin_y-0.35,base_origin_z,gx, gy, gz);
    m_bfs[2]->run(gx,gy,gz);
    SMPL_WARN_STREAM("Origin3 "<<gx<<","<<gy<<","<<gz);
    grid()->worldToGrid(base_origin_x-0.2,base_origin_y+0.35,base_origin_z,gx, gy, gz);
    m_bfs[3]->run(gx,gy,gz);
    SMPL_WARN_STREAM("Origin4 "<<gx<<","<<gy<<","<<gz);*/

    //grid()->worldToGrid(goal.pose[0],goal.pose[1],goal.pose[2],gx, gy, gz);
    int listSize = std::ceil((double)(0.4/grid()->resolution()));
    std::vector<int> inputGoals(listSize*6+3);
    inputGoals[0] = gx;
    inputGoals[1] = gy;
    inputGoals[2] = gz;
    SMPL_ERROR_STREAM("Goal Region Size "<<listSize<<" resolution "<<grid()->resolution());
    for(int i=1;i<=listSize;i++)
    {
        inputGoals[i*3] = gx + i;
        SMPL_ERROR_STREAM("X element ["<<i*3<<"] is "<<inputGoals[i*3]);
    }
    for(int i=1;i<=listSize;i++)
    {
        inputGoals[listSize*3 + i*3] = gx - i;
        SMPL_ERROR_STREAM("X element negative  ["<<listSize*3+i*3<<"] is "<<inputGoals[listSize*3+i*3]);
    }
    for(int j=1;j<=listSize;j++)
    {
        inputGoals[j*3+1] = gy + j;
        SMPL_ERROR_STREAM("Y element ["<<j*3+1<<"] is "<<inputGoals[j*3+1]);
    }
    for(int j=1;j<=listSize;j++)
    {
        inputGoals[listSize*3 + j*3+1] = gy - j;
        SMPL_ERROR_STREAM("Y element negative ["<<listSize*3 + j*3+1<<"] is "<<inputGoals[listSize*3+j*3+1]);
    }
    for(int k=1; k<=listSize;k++)
    {
        inputGoals[k*3+2] = gz + k;
        SMPL_ERROR_STREAM("Z element ["<<k*3+2<<"] is "<<inputGoals[k*3+2]);
    }
    for(int k=1; k<=listSize;k++)
    {
        inputGoals[listSize*3 + k*3+2] = gz - k;
        SMPL_ERROR_STREAM("Z element negative ["<<listSize*3 + k*3+2<<"] is "<<inputGoals[listSize*3+k*3+2]);
    }
    m_bfs[0]->run < std::vector<int>::iterator >(inputGoals.begin(),inputGoals.end());
    m_bfs[1]->run(gx,gy,gz);
    m_bfs[2]->run(gx,gy,gz);
    m_bfs[3]->run(gx,gy,gz);

}

double MultiBfsHeuristic::getMetricStartDistance(double x, double y, double z)
{
    int start_id = planningSpace()->getStartStateID();

    if (!m_pp) {
        return 0.0;
    }

    Eigen::Vector3d p;
    if (!m_pp->projectToPoint(planningSpace()->getStartStateID(), p)) {
        return 0.0;
    }

    int sx, sy, sz;
    grid()->worldToGrid(p.x(), p.y(), p.z(), sx, sy, sz);

    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);

    // compute the manhattan distance to the start cell
    const int dx = sx - gx;
    const int dy = sy - gy;
    const int dz = sz - gz;
    return grid()->resolution() * (abs(dx) + abs(dy) + abs(dz));
}

double MultiBfsHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);
    if (!m_bfs[4]->inBounds(gx, gy, gz)) {
        return (double)BFS_3D::WALL * grid()->resolution();
    } else {
        return (double)m_bfs[4]->getDistance(gx, gy, gz) * grid()->resolution();
    }
}

double MultiBfsHeuristic::getMetricGoalDistance(double x, double y, double z, GroupType planning_group)
{
    /*int gx, gy, gz;
    if(planning_group==GroupType::BASE)
    {
        grid()->worldToGrid(x, y, z, gx, gy, gz);
        if (!m_base_bfs->inBounds(gx, gy, gz)) {
            return (double)BFS_3D::WALL * grid()->resolution();
        } else {
            return (double)m_base_bfs->getDistance(gx, gy, gz) * grid()->resolution();
        }
    }
    else
    {
        grid()->worldToGrid(x, y, z, gx, gy, gz);
        if (!m_bfs->inBounds(gx, gy, gz)) {
            return (double)BFS_3D::WALL * grid()->resolution();
        } else {
            return (double)m_bfs->getDistance(gx, gy, gz) * grid()->resolution();
        }
    }*/
}

Extension* MultiBfsHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

int MultiBfsHeuristic::GetGoalHeuristic(int state_id)
{
    if (!m_pp) {
        return 0;
    }

    Eigen::Vector3d p;
    if (!m_pp->projectToPoint(state_id, p)) {
        return 0;
    }

    Eigen::Vector3i dp;
    grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());

    return getBfsCostToGoal(*m_bfs[0], dp.x(), dp.y(), dp.z());
}


int MultiBfsHeuristic::GetGoalHeuristic(int state_id, int planning_group, int base_heuristic_idx)
{
    if (!m_pp) {
        return 0;
    }

    Eigen::Vector3d p;
    if(planning_group==GroupType::BASE)
    {
        if (!m_pp->projectToBasePoint(state_id, p)) {
            return 0;
        }
    }
    else
    {
        if (!m_pp->projectToPoint(state_id, p)) {
            return 0;
        }
    }

    Eigen::Vector3i dp;
    grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());
    
    //SMPL_WARN_STREAM("Point to be computed "<<dp.x()<<","<< dp.y()<<","<< dp.z());
    if(planning_group==GroupType::BASE)
        return getBfsCostToGoal(*m_bfs[base_heuristic_idx], dp.x(), dp.y(), dp.z());
    else
        return getBfsCostToGoal(*m_bfs[4], dp.x(), dp.y(), dp.z());
}

int MultiBfsHeuristic::GetStartHeuristic(int state_id)
{
    SMPL_WARN_ONCE("MultiBfsHeuristic::GetStartHeuristic unimplemented");
    return 0;
}

int MultiBfsHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    if (to_id == planningSpace()->getGoalStateID()) {
        return GetGoalHeuristic(from_id);
    }
    else {
        SMPL_WARN_ONCE("MultiBfsHeuristic::GetFromToHeuristic unimplemented for arbitrary state pair");
        return 0;
    }
}

auto MultiBfsHeuristic::getWallsVisualization() const -> visual::Marker
{
    /*std::vector<Eigen::Vector3d> centers;
    int dimX = grid()->numCellsX();
    int dimY = grid()->numCellsY();
    int dimZ = grid()->numCellsZ();
    for (int x = 0; x < dimX; x++) {
    for (int y = 0; y < dimY; y++) {
    for (int z = 0; z < dimZ; z++) {
        if (m_base_bfs->isWall(x, y, z)) {
            Eigen::Vector3d p;
            grid()->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            centers.push_back(p);
        }
    }
    }
    }

    SMPL_DEBUG_NAMED(LOG, "BFS Visualization contains %zu points", centers.size());

    visual::Color color;
    color.r = 100.0f / 255.0f;
    color.g = 149.0f / 255.0f;
    color.b = 238.0f / 255.0f;
    color.a = 1.0f;
    return visual::MakeCubesMarker(
            centers,
            grid()->resolution(),
            color,
            grid()->getReferenceFrame(),
            "bfs_walls");*/
}

auto MultiBfsHeuristic::getValuesVisualization() -> visual::Marker
{
    /*if (m_goal_x < 0 || m_goal_y < 0 || m_goal_z < 0) {
        return visual::MakeEmptyMarker();
    }

    if (m_bfs->isWall(m_goal_x, m_goal_y, m_goal_z)) {
        return visual::MakeEmptyMarker();
    }

    // hopefully this doesn't screw anything up too badly...this will flush the
    // bfs to a little past the start, but this would be done by the search
    // hereafter anyway
    int start_heur = GetGoalHeuristic(planningSpace()->getStartStateID());
    if (start_heur == Infinity) {
        return visual::MakeEmptyMarker();
    }

    SMPL_INFO("Start cell heuristic: %d", start_heur);

    const int max_cost = (int)(1.1 * start_heur);

    SMPL_INFO("Get visualization of cells up to cost %d", max_cost);

    // ...and this will also flush the bfs...

    const size_t max_points = 4 * 4096;

    std::vector<Eigen::Vector3d> points;
    std::vector<visual::Color> colors;

    struct CostCell
    {
        int x, y, z, g;
    };
    std::queue<CostCell> cells;
    Grid3<bool> visited(grid()->numCellsX(), grid()->numCellsY(), grid()->numCellsZ(), false);
    visited(m_goal_x, m_goal_y, m_goal_z) = true;
    cells.push({m_goal_x, m_goal_y, m_goal_z, 0});
    while (!cells.empty()) {
        CostCell c = cells.front();
        cells.pop();

        if (c.g > max_cost || points.size() >= max_points) {
            break;
        }

        {
            double cost_pct = (double)c.g / (double)max_cost;

            visual::Color color = visual::MakeColorHSV(300.0 - 300.0 * cost_pct);

            auto clamp = [](double d, double lo, double hi) {
                if (d < lo) {
                    return lo;
                } else if (d > hi) {
                    return hi;
                } else {
                    return d;
                }
            };

            color.r = clamp(color.r, 0.0f, 1.0f);
            color.g = clamp(color.g, 0.0f, 1.0f);
            color.b = clamp(color.b, 0.0f, 1.0f);

            Eigen::Vector3d p;
            grid()->gridToWorld(c.x, c.y, c.z, p.x(), p.y(), p.z());
            points.push_back(p);

            colors.push_back(color);
        }

//        visited(c.x, c.y, c.z) = true;

        const int d = m_cost_per_cell * m_bfs->getDistance(c.x, c.y, c.z);

        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
            if (!(dx | dy | dz)) {
                continue;
            }

            int sx = c.x + dx;
            int sy = c.y + dy;
            int sz = c.z + dz;

            // check if neighbor is valid
            if (!m_bfs->inBounds(sx, sy, sz) || m_bfs->isWall(sx, sy, sz)) {
                continue;
            }

            // check if cost can be improved
            if (visited(sx, sy, sz)) {
                continue;
            }

            visited(sx, sy, sz) = true;

            int dd = m_cost_per_cell * m_bfs->getDistance(sx, sy, sz);
            cells.push({sx, sy, sz, dd});
        }
        }
        }
    }

    return visual::MakeCubesMarker(
            std::move(points),
            0.5 * grid()->resolution(),
            std::move(colors),
            grid()->getReferenceFrame(),
            "bfs_values");*/
}

void MultiBfsHeuristic::syncGridAndBfs()
{
    const int xc = grid()->numCellsX();
    const int yc = grid()->numCellsY();
    const int zc = grid()->numCellsZ();
    std::unique_ptr<BFS_3D> temp;
    temp.reset(new BFS_3D(xc, yc, zc));
    m_bfs.push_back(std::move(temp));
    temp.reset(new BFS_3D(xc, yc, zc));
    m_bfs.push_back(std::move(temp));
    temp.reset(new BFS_3D(xc, yc, zc));
    m_bfs.push_back(std::move(temp));
    temp.reset(new BFS_3D(xc, yc, zc));
    m_bfs.push_back(std::move(temp));
    temp.reset(new BFS_3D(xc, yc, zc));
    m_bfs.push_back(std::move(temp));
    const int cell_count = xc * yc * zc;
    int wall_count = 0, base_wall_count = 0;
    for (int x = 0; x < xc; ++x) {
        for (int y = 0; y < yc; ++y) {
            for (int z = 0; z < zc; ++z) {
                const double radius = m_inflation_radius;
                if (grid()->getDistance(x, y, z) <= radius) {
                    m_bfs[4]->setWall(x, y, z);
                    ++wall_count;
                }
                if(grid()->getDistance(x, y, z) <= m_base_inflation_radius)
                {
                    m_bfs[0]->setWall(x,y,z);
                    m_bfs[1]->setWall(x,y,z);
                    m_bfs[2]->setWall(x,y,z);
                    m_bfs[3]->setWall(x,y,z);
                    base_wall_count+=4;
                }
            }
        }
    }

    SMPL_DEBUG_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
    SMPL_DEBUG_NAMED(LOG, "%d/%d (%0.3f%%) walls in the base bfs heuristic", base_wall_count, cell_count, 100.0 * (double)base_wall_count / cell_count);
}

int MultiBfsHeuristic::getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const
{
    if (!bfs.inBounds(x, y, z)) {
        return Infinity;
    }
    else if (bfs.getDistance(x, y, z) == BFS_3D::WALL) {
        return Infinity;
    }
    else 
    {
        return m_cost_per_cell * bfs.getDistance(x, y, z);
    }
}

bool MultiBfsHeuristic::isBaseGoalInBoundsObstacleFree(int x, int y, int z)  const
{
    /*std::vector<Eigen::Vector3d> bounding_box_corners;
    bounding_box_corners.push_back(Eigen::Vector3d (x+0.25, y+0.3, z+0.36));
    bounding_box_corners.push_back(Eigen::Vector3d (x-0.25, y+0.3, z+0.36));
    bounding_box_corners.push_back(Eigen::Vector3d (x+0.25, y+0.3, z-0.36));
    bounding_box_corners.push_back(Eigen::Vector3d (x-0.25, y+0.3, z-0.36));
    bounding_box_corners.push_back(Eigen::Vector3d (x+0.25, y-0.3, z+0.36));
    bounding_box_corners.push_back(Eigen::Vector3d (x-0.25, y-0.3, z+0.36));
    bounding_box_corners.push_back(Eigen::Vector3d (x+0.25, y-0.3, z-0.36));
    bounding_box_corners.push_back(Eigen::Vector3d (x-0.25, y-0.3, z-0.36));

    for(size_t i=0;i<bounding_box_corners.size();i++)
    {
        if (!m_base_bfs->inBounds(bounding_box_corners[i][0],bounding_box_corners[i][1],bounding_box_corners[i][2])) {
            return false;
        }
        else if (m_base_bfs->getDistance(bounding_box_corners[i][0],bounding_box_corners[i][1],bounding_box_corners[i][2]) == BFS_3D::WALL) {
            return false;
        }
    }*/
}
} // namespace motion
} // namespace sbpl
