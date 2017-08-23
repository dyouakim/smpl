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

#include <sbpl_collision_checking/world_collision_model.h>

// standard includes
#include <map>

// system includes
#include <Eigen/Dense>
#include <boost/make_shared.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <leatherman/utils.h>
#include <moveit/collision_detection/world.h>
#include <octomap_msgs/conversions.h>

// project includes
#include "voxel_operations.h"
#include "collision_operations.h"

namespace sbpl {
namespace collision {

static const char* WCM_LOGGER = "world";

/////////////////////////////////////////
// WorldCollisionModelImpl Declaration //
/////////////////////////////////////////

class WorldCollisionModelImpl
{
public:

    WorldCollisionModelImpl(OccupancyGrid* grid);
    WorldCollisionModelImpl(
        const WorldCollisionModelImpl& o,
        OccupancyGrid* grid);

    OccupancyGrid* grid();
    const OccupancyGrid* grid() const;

    bool insertObject(const ObjectConstPtr& object);
    bool removeObject(const ObjectConstPtr& object);
    bool moveShapes(const ObjectConstPtr& object);
    bool insertShapes(const ObjectConstPtr& object);
    bool removeShapes(const ObjectConstPtr& object);

    bool processCollisionObject(const moveit_msgs::CollisionObject& object);
    bool insertOctomap(const octomap_msgs::OctomapWithPose& octomap);

    bool removeObject(const std::string& object_name);

    void reset();

    visualization_msgs::MarkerArray getWorldVisualization() const;
    visualization_msgs::MarkerArray getCollisionWorldVisualization() const;

    void setPadding(double padding);
    double padding() const;

private:

    OccupancyGrid* m_grid;

    // set of collision objects
    std::map<std::string, ObjectConstPtr> m_object_map;

    // voxelization of objects in the grid reference frame
    typedef std::vector<Eigen::Vector3d> VoxelList;
    std::map<std::string, std::vector<VoxelList>> m_object_voxel_map;

    double m_padding;

    ////////////////////
    // Generic Shapes //
    ////////////////////

    bool haveObject(const std::string& name) const;

    bool checkObjectInsert(const Object& object) const;
    bool checkObjectRemove(const Object& object) const;
    bool checkObjectRemove(const std::string& object_name) const;
    bool checkObjectMoveShape(const Object& object) const;
    bool checkObjectInsertShape(const Object& object) const;
    bool checkObjectRemoveShape(const Object& object) const;

    ///////////////////////
    // Collision Objects //
    ///////////////////////

    ObjectConstPtr convertOctomapToObject(
        const octomap_msgs::OctomapWithPose& octomap) const;

    // return whether or not to accept an incoming collision object
    bool checkCollisionObjectAdd(
        const moveit_msgs::CollisionObject& object) const;
    bool checkCollisionObjectRemove(
        const moveit_msgs::CollisionObject& object) const;
    bool checkCollisionObjectAppend(
        const moveit_msgs::CollisionObject& object) const;
    bool checkCollisionObjectMove(
        const moveit_msgs::CollisionObject& object) const;

    bool checkInsertOctomap(const octomap_msgs::OctomapWithPose& octomap) const;

    bool addCollisionObject(const moveit_msgs::CollisionObject& object);
    bool removeCollisionObject(const moveit_msgs::CollisionObject& object);
    bool appendCollisionObject(const moveit_msgs::CollisionObject& object);
    bool moveCollisionObject(const moveit_msgs::CollisionObject& object);

    void removeAllCollisionObjects();

    ///////////////////
    // Visualization //
    ///////////////////

    void getAllCollisionObjectVoxels(
        std::vector<geometry_msgs::Point>& points) const;

    visualization_msgs::MarkerArray getWorldObjectMarkerArray(
        const Object& object,
        std::vector<double>& hue,
        const std::string& ns,
        int id) const;
};

////////////////////////////////////////
// WorldCollisionModelImpl Definition //
////////////////////////////////////////

WorldCollisionModelImpl::WorldCollisionModelImpl(OccupancyGrid* grid) :
    m_grid(grid),
    m_object_map(),
    m_object_voxel_map(),
    m_padding(0.0)
{
}

WorldCollisionModelImpl::WorldCollisionModelImpl(
    const WorldCollisionModelImpl& o,
    OccupancyGrid* grid)
:
    m_grid(grid),
    m_object_voxel_map(o.m_object_voxel_map),
    m_padding(o.m_padding)
{
    // TODO: check for different voxel origin/resolution/etc here...if they
    // differ, need to do a deep copy + revoxelization of the objects over just
    // a simple deep copy
    *grid = *m_grid;
}

OccupancyGrid* WorldCollisionModelImpl::grid()
{
    return m_grid;
}

const OccupancyGrid* WorldCollisionModelImpl::grid() const
{
    return m_grid;
}

bool WorldCollisionModelImpl::insertObject(const ObjectConstPtr& object)
{
    if (!checkObjectInsert(*object)) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Rejecting addition of collision object '%s'", object->id_.c_str());
        return false;
    }

    assert(m_object_voxel_map.find(object->id_) == m_object_voxel_map.end());

    const double res = m_grid->resolution();
    const Eigen::Vector3d origin(
            m_grid->originX(), m_grid->originY(), m_grid->originZ());

    const Eigen::Vector3d gmin(
            m_grid->originX(), m_grid->originY(), m_grid->originZ());

    const Eigen::Vector3d gmax(
            m_grid->originX() + m_grid->sizeX(),
            m_grid->originY() + m_grid->sizeY(),
            m_grid->originZ() + m_grid->sizeZ());

    std::vector<std::vector<Eigen::Vector3d>> all_voxels;
    if (!VoxelizeObject(*object, res, origin, gmin, gmax, all_voxels)) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Failed to voxelize object '%s'", object->id_.c_str());
        return false;
    }

    auto vit = m_object_voxel_map.insert(
            std::make_pair(object->id_, std::vector<VoxelList>()));
    vit.first->second = std::move(all_voxels);
    assert(vit.second);

    m_object_map.insert(std::make_pair(object->id_, object));

    for (const auto& voxel_list : vit.first->second) {
        ROS_DEBUG_NAMED(WCM_LOGGER, "Adding %zu voxels from collision object '%s' to the distance transform",
                voxel_list.size(), object->id_.c_str());
        m_grid->addPointsToField(voxel_list);
    }

    return true;
}

bool WorldCollisionModelImpl::removeObject(const ObjectConstPtr& object)
{
    return removeObject(object->id_);
}

bool WorldCollisionModelImpl::removeObject(const std::string& object_name)
{
    if (!checkObjectRemove(object_name)) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Rejecting removal of collision object '%s'", object_name.c_str());
        return false;
    }

    auto oit = m_object_map.find(object_name);
    assert(oit != m_object_map.end());

    auto vit = m_object_voxel_map.find(object_name);
    assert(vit != m_object_voxel_map.end());

    for (const auto& voxel_list : vit->second) {
        ROS_DEBUG_NAMED(WCM_LOGGER, "Removing %zu grid cells from the distance transform", voxel_list.size());
        m_grid->removePointsFromField(voxel_list);
    }

    m_object_voxel_map.erase(vit);
    m_object_map.erase(oit);
    return true;
}

bool WorldCollisionModelImpl::moveShapes(const ObjectConstPtr& object)
{
    // TODO: optimized version
    return removeObject(object) && insertObject(object);
}

bool WorldCollisionModelImpl::insertShapes(const ObjectConstPtr& object)
{
    // TODO: optimized version
    return removeObject(object) && insertObject(object);
}

bool WorldCollisionModelImpl::removeShapes(const ObjectConstPtr& object)
{
    // TODO: optimized version
    return removeObject(object) && insertObject(object);
}

bool WorldCollisionModelImpl::processCollisionObject(
    const moveit_msgs::CollisionObject& object)
{
    if (object.operation == moveit_msgs::CollisionObject::ADD) {
        return addCollisionObject(object);
    }
    else if (object.operation == moveit_msgs::CollisionObject::REMOVE) {
        return removeCollisionObject(object);
    }
    else if (object.operation == moveit_msgs::CollisionObject::APPEND) {
        return appendCollisionObject(object);
    }
    else if (object.operation == moveit_msgs::CollisionObject::MOVE) {
        return moveCollisionObject(object);
    }
    else {
        ROS_ERROR_NAMED(WCM_LOGGER, "Collision object operation '%d' is not supported", object.operation);
        return false;
    }
}

bool WorldCollisionModelImpl::insertOctomap(
    const octomap_msgs::OctomapWithPose& octomap)
{
    if (!checkInsertOctomap(octomap)) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Rejecting addition of octomap '%s'", octomap.octomap.id.c_str());
        return false;
    }

    ObjectConstPtr op = convertOctomapToObject(octomap);
    if (!op) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Failed to convert octomap message to collision object");
        return false;
    }

    return insertObject(op);
}

void WorldCollisionModelImpl::reset()
{
    m_grid->reset();
    for (const auto& entry : m_object_voxel_map) {
        for (const auto& voxel_list : entry.second) {
            m_grid->addPointsToField(voxel_list);
        }
    }
}

visualization_msgs::MarkerArray
WorldCollisionModelImpl::getWorldVisualization() const
{
    visualization_msgs::MarkerArray ma;
    for (const auto& ent : m_object_map) {
        const ObjectConstPtr& object = ent.second;
        std::vector<double> hue(object->shapes_.size(), 200);
        visualization_msgs::MarkerArray ma1 =
                getWorldObjectMarkerArray(*object, hue, object->id_, 0);
        ma.markers.insert(ma.markers.end(), ma1.markers.begin(), ma1.markers.end());
    }
    return ma;
}

visualization_msgs::MarkerArray
WorldCollisionModelImpl::getCollisionWorldVisualization() const
{
    visualization_msgs::MarkerArray ma;

    std::vector<geometry_msgs::Point> voxels;
    getAllCollisionObjectVoxels(voxels);

    visualization_msgs::Marker marker;
    marker.header.seq = 0;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = m_grid->getReferenceFrame();
    marker.ns = "collision_object_voxels";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.0);
    marker.scale.x = m_grid->resolution();
    marker.scale.y = m_grid->resolution();
    marker.scale.z = m_grid->resolution();
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.points.resize(voxels.size());
    for (size_t i = 0; i < voxels.size(); ++i) {
        marker.points[i].x = voxels[i].x;
        marker.points[i].y = voxels[i].y;
        marker.points[i].z = voxels[i].z;
    }
    ma.markers.push_back(marker);

    return ma;
}

void WorldCollisionModelImpl::setPadding(double padding)
{
    m_padding = padding;
}

double WorldCollisionModelImpl::padding() const
{
    return m_padding;
}

bool WorldCollisionModelImpl::haveObject(const std::string& name) const
{
    return m_object_map.find(name) != m_object_map.end();
}

bool WorldCollisionModelImpl::checkObjectInsert(const Object& object) const
{
    if (haveObject(object.id_)) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Already have collision object '%s'", object.id_.c_str());
        return false;
    }

    if (object.shapes_.size() != object.shape_poses_.size()) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Mismatched sizes of shapes and shape poses");
        return false;
    }

    return true;
}

bool WorldCollisionModelImpl::checkObjectRemove(const Object& object) const
{
    return checkObjectRemove(object.id_);
}

bool WorldCollisionModelImpl::checkObjectRemove(const std::string& name) const
{
    return haveObject(name);
}

bool WorldCollisionModelImpl::checkObjectMoveShape(const Object& object) const
{
    return haveObject(object.id_);
}

bool WorldCollisionModelImpl::checkObjectInsertShape(const Object& object) const
{
    return haveObject(object.id_);
}

bool WorldCollisionModelImpl::checkObjectRemoveShape(const Object& object) const
{
    return haveObject(object.id_);
}

ObjectConstPtr WorldCollisionModelImpl::convertOctomapToObject(
    const octomap_msgs::OctomapWithPose& octomap) const
{
    // convert binary octomap message to octree
    octomap::AbstractOcTree* abstract_tree =
            octomap_msgs::binaryMsgToMap(octomap.octomap);
    if (!abstract_tree) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Failed to convert binary msg data to octomap");
        return ObjectConstPtr();
    }

    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(abstract_tree);
    if (!tree) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Abstract Octree from binary msg data must be a concrete OcTree");
        return ObjectConstPtr();
    }

    boost::shared_ptr<const octomap::OcTree> ot(tree);

    // wrap with a shape
    //shapes::ShapeConstPtr sp = boost::make_shared<shapes::OcTree>(ot);

    Eigen::Affine3d transform;
    tf::poseMsgToEigen(octomap.origin, transform);

    // construct the object
    auto o = std::make_shared<const Object>(octomap.octomap.id);

    /*o->shapes_.push_back(sp);
    o->shape_poses_.push_back(transform);*/

    return o;
}

bool WorldCollisionModelImpl::checkCollisionObjectAdd(
    const moveit_msgs::CollisionObject& object) const
{
    if (haveObject(object.id)) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Already have collision object '%s'", object.id.c_str());
        return false;
    }

    if (object.header.frame_id != m_grid->getReferenceFrame()) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Collision object must be specified in the grid reference frame (%s)", m_grid->getReferenceFrame().c_str());
        return false;
    }

    if (object.primitives.size() != object.primitive_poses.size()) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Mismatched sizes of primitives and primitive poses");
        return false;
    }

    if (object.meshes.size() != object.mesh_poses.size()) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Mismatches sizes of meshes and mesh poses");
        return false;
    }

    // check solid primitive for correct format
    for (const shape_msgs::SolidPrimitive& prim : object.primitives) {
        switch (prim.type) {
        case shape_msgs::SolidPrimitive::BOX:
        {
            if (prim.dimensions.size() != 3) {
                ROS_ERROR_NAMED(WCM_LOGGER, "Invalid number of dimensions for box of collision object '%s' (Expected: %d, Actual: %zu)",
                        object.id.c_str(), 3, prim.dimensions.size());
                return false;
            }
        }   break;
        case shape_msgs::SolidPrimitive::SPHERE:
        {
            if (prim.dimensions.size() != 1) {
                ROS_ERROR_NAMED(WCM_LOGGER, "Invalid number of dimensions for sphere of collision object '%s' (Expected: %d, Actual: %zu)",
                        object.id.c_str(), 1, prim.dimensions.size());
                return false;
            }
        }   break;
        case shape_msgs::SolidPrimitive::CYLINDER:
        {
            if (prim.dimensions.size() != 2) {
                ROS_ERROR_NAMED(WCM_LOGGER, "Invalid number of dimensions for cylinder of collision object '%s' (Expected: %d, Actual: %zu)",
                        object.id.c_str(), 2, prim.dimensions.size());
                return false;
            }
        }   break;
        case shape_msgs::SolidPrimitive::CONE:
        {
            if (prim.dimensions.size() != 2) {
                ROS_ERROR_NAMED(WCM_LOGGER, "Invalid number of dimensions for cone of collision object '%s' (Expected: %d, Actual: %zu)",
                        object.id.c_str(), 2, prim.dimensions.size());
                return false;
            }
        }   break;
        default:
            ROS_ERROR_NAMED(WCM_LOGGER, "Unrecognized SolidPrimitive type");
            return false;
        }
    }

    return true;
}

bool WorldCollisionModelImpl::checkCollisionObjectRemove(
    const moveit_msgs::CollisionObject& object) const
{
    return haveObject(object.id);
}

bool WorldCollisionModelImpl::checkCollisionObjectAppend(
    const moveit_msgs::CollisionObject& object) const
{
    return m_object_map.find(object.id) != m_object_map.end();
}

bool WorldCollisionModelImpl::checkCollisionObjectMove(
    const moveit_msgs::CollisionObject& object) const
{
    return m_object_map.find(object.id) != m_object_map.end();
}

bool WorldCollisionModelImpl::checkInsertOctomap(
    const octomap_msgs::OctomapWithPose& octomap) const
{
    if (haveObject(octomap.octomap.id)) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Already have collision object '%s'", octomap.octomap.id.c_str());
        return false;
    }

    if (octomap.header.frame_id != m_grid->getReferenceFrame()) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Octomap must be specified in the grid reference frame (%s)", m_grid->getReferenceFrame().c_str());
        return false;
    }

    if (!octomap.octomap.binary) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Octomap must be a binary octomap");
        return false;
    }

    return true;
}

bool WorldCollisionModelImpl::addCollisionObject(const moveit_msgs::CollisionObject& object)
{
    if (!checkCollisionObjectAdd(object)) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Rejecting addition of collision object '%s'", object.id.c_str());
        return false;
    }

    ObjectConstPtr op = ConvertCollisionObjectToObject(object);
    if (!op) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Failed to convert collision object to internal representation");
        return false;
    }

    return insertObject(op);
}

bool WorldCollisionModelImpl::removeCollisionObject(const moveit_msgs::CollisionObject& object)
{
    return removeObject(object.id);
}

bool WorldCollisionModelImpl::appendCollisionObject(const moveit_msgs::CollisionObject& object)
{
    if (!checkCollisionObjectAppend(object)) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Rejecting append to collision object '%s'", object.id.c_str());
        return false;
    }

    // TODO: implement
    ROS_ERROR_NAMED(WCM_LOGGER, "appendCollisionObject unimplemented");
    return false;
}

bool WorldCollisionModelImpl::moveCollisionObject(const moveit_msgs::CollisionObject& object)
{
    if (!checkCollisionObjectMove(object)) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Rejecting move of collision object '%s'", object.id.c_str());
        return false;
    }

    // TODO: implement
    ROS_ERROR_NAMED(WCM_LOGGER, "moveCollisionObject unimplemented");
    return false;
}

void WorldCollisionModelImpl::removeAllCollisionObjects()
{
    for (const auto& entry : m_object_map) {
        removeObject(entry.first);
    }
}

void WorldCollisionModelImpl::getAllCollisionObjectVoxels(
    std::vector<geometry_msgs::Point>& points) const
{
    for (const auto& entry : m_object_map) {
        const std::string& name = entry.first;
        const ObjectConstPtr& object = entry.second;
        auto vlsit = m_object_voxel_map.find(name);
        assert(vlsit != m_object_voxel_map.end());
        for (size_t sidx = 0; sidx < object->shapes_.size(); ++sidx) {
            const shapes::ShapeConstPtr& shape = object->shapes_[sidx];
            const bool is_collision_object = shape->type != shapes::OCTREE;
            if (is_collision_object) {
                assert(vlsit->second.size() > sidx);
                const VoxelList& vl = vlsit->second[sidx];
                for (const Eigen::Vector3d& voxel : vl) {
                    geometry_msgs::Point point;
                    point.x = voxel.x();
                    point.y = voxel.y();
                    point.z = voxel.z();
                    points.push_back(point);
                }
            }
        }
    }
}

visualization_msgs::MarkerArray WorldCollisionModelImpl::getWorldObjectMarkerArray(
    const Object& object,
    std::vector<double>& hue,
    const std::string& ns,
    int id) const
{
    visualization_msgs::MarkerArray ma;

    if (object.shapes_.size() != object.shape_poses_.size()) {
        ROS_ERROR_NAMED(WCM_LOGGER, "Mismatched sizes of shapes and shape poses");
        return ma;
    }

    std::vector<std::vector<double>> colors;
    colors.resize(hue.size(), std::vector<double>(4, 1.0));
    for (size_t i = 0; i < colors.size(); ++i) {
        leatherman::HSVtoRGB(
                &colors[i][0], &colors[i][1], &colors[i][2], hue[i], 1.0, 1.0);
    }

    for (size_t i = 0; i < object.shapes_.size(); ++i) {
        const shapes::ShapeConstPtr& shape = object.shapes_[i];
        const Eigen::Affine3d& pose = object.shape_poses_[i];

        // fill in type and scale
        visualization_msgs::Marker m;
        if (!shapes::constructMarkerFromShape(shape.get(), m)) {
            ROS_WARN_NAMED(WCM_LOGGER, "Failed to construct marker from shape");
        }

        m.header.seq = 0;
        m.header.stamp = ros::Time(0);
        m.header.frame_id = m_grid->getReferenceFrame();
        m.ns = ns;
        m.id = id + i;
        // m.type filled in above
        m.action = visualization_msgs::Marker::ADD;
        tf::poseEigenToMsg(pose, m.pose);
        m.color.r = colors[i][0];
        m.color.g = colors[i][1];
        m.color.b = colors[i][2];
        m.color.a = colors[i][3];
        m.lifetime = ros::Duration(0);
        m.frame_locked = false;

        ma.markers.push_back(m);
    }

    return ma;
}

////////////////////////////////////
// WorldCollisionModel Definition //
////////////////////////////////////

WorldCollisionModel::WorldCollisionModel(OccupancyGrid* grid) :
    m_impl(new WorldCollisionModelImpl(grid))
{
}

WorldCollisionModel::WorldCollisionModel(
    const WorldCollisionModel& o,
    OccupancyGrid* grid)
:
    m_impl(new WorldCollisionModelImpl(*o.m_impl, grid))
{
}

WorldCollisionModel::~WorldCollisionModel()
{
}

OccupancyGrid* WorldCollisionModel::grid()
{
    return m_impl->grid();
}

const OccupancyGrid* WorldCollisionModel::grid() const
{
    return m_impl->grid();
}

bool WorldCollisionModel::insertObject(const ObjectConstPtr& object)
{
    return m_impl->insertObject(object);
}

bool WorldCollisionModel::removeObject(const ObjectConstPtr& object)
{
    return m_impl->removeObject(object);
}

bool WorldCollisionModel::moveShapes(const ObjectConstPtr& object)
{
    return m_impl->moveShapes(object);
}

bool WorldCollisionModel::insertShapes(const ObjectConstPtr& object)
{
    return m_impl->insertShapes(object);
}

bool WorldCollisionModel::removeShapes(const ObjectConstPtr& object)
{
    return m_impl->removeShapes(object);
}

bool WorldCollisionModel::processCollisionObject(
    const moveit_msgs::CollisionObject& object)
{
    return m_impl->processCollisionObject(object);
}

bool WorldCollisionModel::insertOctomap(
    const octomap_msgs::OctomapWithPose& octomap)
{
    return m_impl->insertOctomap(octomap);
}

bool WorldCollisionModel::removeObject(const std::string& object_name)
{
    return m_impl->removeObject(object_name);
}

void WorldCollisionModel::reset()
{
    return m_impl->reset();
}

visualization_msgs::MarkerArray
WorldCollisionModel::getWorldVisualization() const
{
    return m_impl->getWorldVisualization();
}

visualization_msgs::MarkerArray
WorldCollisionModel::getCollisionWorldVisualization() const
{
    return m_impl->getCollisionWorldVisualization();
}

void WorldCollisionModel::setPadding(double padding)
{
    return m_impl->setPadding(padding);
}

double WorldCollisionModel::padding() const
{
    return m_impl->padding();
}

} // namespace collision
} // namespace sbpl
