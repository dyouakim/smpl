////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

#include <ros/ros.h>
#include <smpl/occupancy_grid.h>
#include <visualization_msgs/MarkerArray.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/visualizer_ros.h>
#include <smpl/geometry/voxelize.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "occupancy_grid_test");
    ros::NodeHandle nh;

    sbpl::VisualizerROS visualizer;
    sbpl::visual::set_visualizer(&visualizer);

    const double size_x = 2.0, size_y = 2.0, size_z = 2.0;
    const double res = 0.02;
    const double origin_x = -1.0, origin_y = -1.0, origin_z = 0.0;
    const double max_distance = 0.2;
    bool ref_counted = true;

    sbpl::OccupancyGrid grid(
            size_x, size_y, size_z,
            res,
            origin_x, origin_y, origin_z,
            max_distance,
            ref_counted);
    grid.setReferenceFrame("world");

    auto getOccupiedVoxelCount = [&]()
    {
        std::vector<Eigen::Vector3d> occupied_voxels;
        grid.getOccupiedVoxels(occupied_voxels);
        return occupied_voxels.size();
    };

    Eigen::Vector3d voxel_origin(origin_x, origin_y, origin_z);

    const double cube_size = 1.0;
    Eigen::Vector3d cube_pos;
    Eigen::Affine3d cube_pose;

    std::vector<Eigen::Vector3d> voxels;
    cube_pos = Eigen::Vector3d(-0.333, 0.0, 0.5);
    cube_pose = Eigen::Translation3d(cube_pos);
    sbpl::geometry::VoxelizeBox(
            cube_size, cube_size, cube_size,
            cube_pose,
            res,
            voxel_origin, voxels);

    grid.addPointsToField(voxels);

    cube_pos = Eigen::Vector3d(0.333, 0.0, 0.5);
    cube_pose = Eigen::Translation3d(cube_pos);
    sbpl::geometry::VoxelizeBox(
            cube_size, cube_size, cube_size,
            cube_pose,
            res,
            voxel_origin,
            voxels);

    grid.addPointsToField(voxels);

    grid.removePointsFromField(voxels);

    ROS_INFO("Occupancy Grid contains %zu occupied voxels", getOccupiedVoxelCount());

    auto markers = grid.getOccupiedVoxelsVisualization();

    ros::Duration(1.0).sleep(); // let ros and the publisher set up
    SV_SHOW_INFO(markers);
    ros::spinOnce();
    ros::Duration(1.0).sleep(); // let the publisher...publish

    SV_SHOW_INFO(grid.getBoundingBoxVisualization());
    ros::Duration(1.0).sleep();

    return 0;
}
