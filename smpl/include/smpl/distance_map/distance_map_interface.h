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

#ifndef SMPL_DISTANCE_MAP_INTERFACE_H
#define SMPL_DISTANCE_MAP_INTERFACE_H

// standard includes
#include <vector>
#include <map>
// system includes
#include <Eigen/Dense>
#include <Eigen/StdVector>
// project includes
#include <smpl/forward.h>

namespace sbpl {

SBPL_CLASS_FORWARD(DistanceMapInterface)

/// Abstract base class for Distance Map implementations. This class specifies
/// methods for returning distances to the nearest occupied cells, both in
/// cell units and metric units.
///
/// The distances computed for cells may differ from those computed for points
/// within the same cell. This distinction is made by the two functions,
/// getCellDistance() and getMetricDistance(), and similarly for their squared
/// variants. The getCellDistance() function return the distance from the center
/// of the query cell to the center of its nearest obstacle cell. This distance
/// is valid for the cell center but may be an approximation of the distance
/// from another point, even within the same cell, to its nearest obstacle cell.
/// The function getMetricDistance() may provide more accurate distances.
class DistanceMapInterface
{
public:

    DistanceMapInterface(
        double origin_x, double origin_y, double origin_z,
        double size_x, double size_y, double size_z,
        double res)
    :
        m_origin_x(origin_x),
        m_origin_y(origin_y),
        m_origin_z(origin_z),
        m_size_x(size_x),
        m_size_y(size_y),
        m_size_z(size_z),
        m_res(res)
    { }

    virtual ~DistanceMapInterface() { }

    virtual DistanceMapInterface* clone() const = 0;

    /// \name Modifiers
    ///@{
    virtual void addPointsToMap(const std::vector<Eigen::Vector3d>& points) = 0;
    virtual void removePointsFromMap(const std::vector<Eigen::Vector3d>& points) = 0;
    virtual void updatePointsInMap(
            const std::vector<Eigen::Vector3d>& old_points,
            const std::vector<Eigen::Vector3d>& new_points) = 0;
    virtual void reset() = 0;
    ///@}

    /// \name Properties
    ///@{
    double originX() const { return m_origin_x; }
    double originY() const { return m_origin_y; }
    double originZ() const { return m_origin_z; }
    double sizeX() const { return m_size_x; }
    double sizeY() const { return m_size_y; }
    double sizeZ() const { return m_size_z; }
    double resolution() const { return m_res; }
    virtual int numCellsX() const = 0;
    virtual int numCellsY() const = 0;
    virtual int numCellsZ() const = 0;

    virtual double getUninitializedDistance() const = 0;
    ///@}

    /// \name Distance Lookups
    ///@{
    virtual double getMetricDistance(double x, double y, double z) const = 0;
    virtual double getCellDistance(int x, int y, int z) const = 0;

    virtual double getMetricSquaredDistance(double x, double y, double z) const
    { double d = getMetricDistance(x, y, z); return d * d; }

    virtual double getCellSquaredDistance(int x, int y, int z) const
    { double d = getCellDistance(x, y, z); return d * d; }
    ///@}

    /// \name Conversions Between Cell and Metric Coordinates
    ///@{
    virtual void gridToWorld(
        int x, int y, int z,
        double& world_x, double& world_y, double& world_z) const = 0;

    virtual void worldToGrid(
        double world_x, double world_y, double world_z,
        int& x, int& y, int& z) const = 0;

    virtual bool isCellValid(int x, int y, int z) const = 0;

    virtual void markCellExpansionStep (double x, double y, double z, int expansion_step) = 0;
    
    virtual std::map<std::vector<int>,int> getAddedCells() = 0;

    virtual std::map<std::vector<int>,int> getRemovedCells() = 0;

    virtual void resetCellsMarking(int restore_step) = 0;
    ///@}


protected:

    double m_origin_x;
    double m_origin_y;
    double m_origin_z;
    double m_size_x;
    double m_size_y;
    double m_size_z;
    double m_res;
};

} // namespace sbpl

#endif
