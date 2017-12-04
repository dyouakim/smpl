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

#include <smpl/ros/propagation_distance_field.h>

// standard includes
#include <cmath>

namespace sbpl {

PropagationDistanceField::PropagationDistanceField(
    double origin_x, double origin_y, double origin_z,
    double size_x, double size_y, double size_z,
    double res,
    double max_distance,
    bool propagate_negative_distances)
:
    DistanceMapInterface(
            origin_x, origin_y, origin_z,
            size_x, size_y, size_z,
            res),
    m_df(size_x, size_y, size_z,
            res,
            origin_x, origin_y, origin_z,
            max_distance,
            propagate_negative_distances),
    m_max_distance(max_distance),
    m_propagate_negative_distances(propagate_negative_distances)
{
}

PropagationDistanceField::PropagationDistanceField(
    const octomap::OcTree& octree,
    const octomap::point3d& bbx_min,
    const octomap::point3d& bbx_max,
    double max_distance,
    bool propagate_negative_distances)
:
    DistanceMapInterface(
            bbx_min.x(), bbx_min.y(), bbx_min.z(),
            bbx_max.x() - bbx_min.x(), bbx_max.y() - bbx_min.y(), bbx_max.z() - bbx_min.z(),
            octree.getResolution()),
    m_df(octree, bbx_min, bbx_max, max_distance, propagate_negative_distances),
    m_max_distance(max_distance),
    m_propagate_negative_distances(propagate_negative_distances)
{
}

PropagationDistanceField::PropagationDistanceField(
    std::istream& stream,
    double max_distance,
    bool propagate_negative_distances)
:
    DistanceMapInterface(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    m_df(stream, max_distance, propagate_negative_distances),
    m_max_distance(max_distance),
    m_propagate_negative_distances(propagate_negative_distances)
{
    m_origin_x = m_df.getOriginX();
    m_origin_y = m_df.getOriginY();
    m_origin_z = m_df.getOriginZ();
    m_size_x = m_df.getSizeX();
    m_size_y = m_df.getSizeY();
    m_size_z = m_df.getSizeZ();
    m_res = m_df.getResolution();
}

DistanceMapInterface* PropagationDistanceField::clone() const
{
    PropagationDistanceField* df = new PropagationDistanceField(
            originX(), originY(), originZ(),
            sizeX(), sizeY(), sizeZ(),
            resolution(),
            m_max_distance,
            m_propagate_negative_distances);
    std::vector<Eigen::Vector3d> points;
    for (int x = 0; x < numCellsX(); ++x) {
    for (int y = 0; y < numCellsY(); ++y) {
    for (int z = 0; z < numCellsZ(); ++z) {
        if (m_df.getDistance(x, y, z) <= 0.0) {
            double wx, wy, wz;
            m_df.gridToWorld(x, y, z, wx, wy, wz);
            points.emplace_back(wx, wy, wz);
        }
    }
    }
    }
    df->addPointsToMap(points);
    return df;
}

void PropagationDistanceField::addPointsToMap(
    const std::vector<Eigen::Vector3d>& points)
{
    EigenSTL::vector_Vector3d pts = toAlignedVector(points);
    m_df.addPointsToField(pts);
}

void PropagationDistanceField::removePointsFromMap(
    const std::vector<Eigen::Vector3d>& points)
{
    EigenSTL::vector_Vector3d pts = toAlignedVector(points);
    m_df.removePointsFromField(pts);
}

void PropagationDistanceField::updatePointsInMap(
    const std::vector<Eigen::Vector3d>& old_points,
    const std::vector<Eigen::Vector3d>& new_points)
{
    EigenSTL::vector_Vector3d old_pts = toAlignedVector(old_points);
    EigenSTL::vector_Vector3d new_pts = toAlignedVector(new_points);
    m_df.updatePointsInField(old_pts, new_pts);
}

void PropagationDistanceField::reset()
{
    m_df.reset();
}

double PropagationDistanceField::getUninitializedDistance() const
{
    return m_df.getUninitializedDistance();
}

int PropagationDistanceField::numCellsX() const
{
    return m_df.getXNumCells();
}

int PropagationDistanceField::numCellsY() const
{
    return m_df.getYNumCells();
}

int PropagationDistanceField::numCellsZ() const
{
    return m_df.getZNumCells();
}

double PropagationDistanceField::getMetricDistance(
    double x, double y, double z) const
{
    return m_df.getDistance(x, y, z);
}

double PropagationDistanceField::getCellDistance(int x, int y, int z) const
{
    return m_df.getDistance(x, y, z);
}

bool PropagationDistanceField::isCellValid(int x, int y, int z) const
{
    return m_df.isCellValid(x, y, z);
}

void PropagationDistanceField::gridToWorld(
    int x, int y, int z,
    double& world_x, double& world_y, double& world_z) const
{
    (void)m_df.gridToWorld(x, y, z, world_x, world_y, world_z);
}

void PropagationDistanceField::worldToGrid(
    double world_x, double world_y, double world_z,
    int& x, int& y, int& z) const
{
    (void)m_df.worldToGrid(world_x, world_y, world_z, x, y, z);
}

EigenSTL::vector_Vector3d PropagationDistanceField::toAlignedVector(
    const std::vector<Eigen::Vector3d>& vin) const
{
    EigenSTL::vector_Vector3d vout(vin.size());
    std::copy(vin.begin(), vin.end(), vout.begin());
    return vout;
}

} // namespace sbpl
