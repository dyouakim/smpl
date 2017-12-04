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

#ifndef SMPL_PROPAGATION_DISTANCE_FIELD_H
#define SMPL_PROPAGATION_DISTANCE_FIELD_H

// system includes
#include <moveit/distance_field/propagation_distance_field.h>

// project includes
#include <smpl/distance_map/distance_map_interface.h>

namespace sbpl {

class PropagationDistanceField : public DistanceMapInterface
{
public:

    PropagationDistanceField(
        double origin_x, double origin_y, double origin_z,
        double size_x, double size_y, double size_z,
        double res,
        double max_distance,
        bool propagate_negative_distances = false);

    PropagationDistanceField(
        const octomap::OcTree& octree,
        const octomap::point3d& bbx_min,
        const octomap::point3d& bbx_max,
        double max_distance,
        bool propagate_negative_distances = false);

    PropagationDistanceField(
        std::istream& stream,
        double max_distance,
        bool propagate_negative_distances = false);

    DistanceMapInterface* clone() const override;

    void addPointsToMap(const std::vector<Eigen::Vector3d>& points) override;

    void removePointsFromMap(
        const std::vector<Eigen::Vector3d>& points) override;

    void updatePointsInMap(
        const std::vector<Eigen::Vector3d>& old_points,
        const std::vector<Eigen::Vector3d>& new_points) override;

    void reset() override;

    double getUninitializedDistance() const override;

    int numCellsX() const override;
    int numCellsY() const override;
    int numCellsZ() const override;

    double getMetricDistance(double x, double y, double z) const override;
    double getCellDistance(int x, int y, int z) const override;

    bool isCellValid(int x, int y, int z) const override;

    void gridToWorld(
        int x, int y, int z,
        double& world_x, double& world_y, double& world_z) const override;

    void worldToGrid(
        double world_x, double world_y, double world_z,
        int& x, int& y, int& z) const override;

private:

    distance_field::PropagationDistanceField m_df;
    bool m_propagate_negative_distances;
    double m_max_distance;

    EigenSTL::vector_Vector3d toAlignedVector(
        const std::vector<Eigen::Vector3d>& v) const;
};

} // namespace sbpl

#endif
