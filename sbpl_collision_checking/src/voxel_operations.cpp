#include <sbpl_collision_checking/voxel_operations.h>

// standard includes
#include <utility>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <octomap/octomap.h>
#include <ros/console.h>
#include <smpl/geometry/voxelize.h>

namespace sbpl {
namespace collision {

std::vector<int> ConvertToVertexIndices(
    const std::vector<shape_msgs::MeshTriangle>& triangles)
{
    std::vector<int> triangle_indices(3 * triangles.size());
    for (int j = 0; j < triangles.size(); ++j) {
        triangle_indices[3 * j + 0] = triangles[j].vertex_indices[0];
        triangle_indices[3 * j + 1] = triangles[j].vertex_indices[1];
        triangle_indices[3 * j + 2] = triangles[j].vertex_indices[2];
    }
    return triangle_indices;
}

static
bool VoxelizeSolidPrimitives(
    const moveit_msgs::CollisionObject& object,
    double res,
    const Eigen::Vector3d& go,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels)
{
    for (size_t i = 0; i < object.primitives.size(); ++i) {
        const shape_msgs::SolidPrimitive& prim = object.primitives[i];
        const geometry_msgs::Pose& pose = object.primitive_poses[i];

        std::vector<Eigen::Vector3d> voxels;
        if (!VoxelizeSolidPrimitive(prim, pose, res, go, voxels)) {
            ROS_ERROR("Failed to voxelize solid primitive of collision object '%s'", object.id.c_str());
            return false;
        }
        all_voxels.push_back(std::move(voxels));
    }

    return true;
}

static
bool VoxelizeMeshes(
    const moveit_msgs::CollisionObject& object,
    double res,
    const Eigen::Vector3d& go,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels)
{
    for (size_t i = 0; i < object.meshes.size(); ++i) {
        const shape_msgs::Mesh& mesh = object.meshes[i];
        const geometry_msgs::Pose& pose = object.mesh_poses[i];
        std::vector<Eigen::Vector3d> voxels;
        if (!VoxelizeMesh(mesh, pose, res, go, voxels)) {
            ROS_ERROR("Failed to voxelize mesh of collision object '%s'", object.id.c_str());
            return false;
        }
        all_voxels.push_back(std::move(voxels));
    }

    return true;
}

static
bool VoxelizePlanes(
    const moveit_msgs::CollisionObject& object,
    double res,
    const Eigen::Vector3d& go,
    const Eigen::Vector3d& gmin,
    const Eigen::Vector3d& gmax,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels)
{
    for (size_t i = 0; i < object.planes.size(); ++i) {
        const shape_msgs::Plane& plane = object.planes[i];
        const geometry_msgs::Pose& pose = object.plane_poses[i];
        std::vector<Eigen::Vector3d> voxels;
        if (!VoxelizePlane(plane, pose, res, go, gmin, gmax, voxels)) {
            ROS_ERROR("Failed to voxelize plane of collision object '%s'", object.id.c_str());
            return false;
        }
        all_voxels.push_back(std::move(voxels));
    }

    return true;
}

static
bool VoxelizePlane(
    const shapes::Plane& plane,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    ROS_ERROR("Cannot voxelize plane without boundary information");
    return false;
}

static
bool VoxelizePlane(
    const shape_msgs::Plane& plane,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    ROS_ERROR("Cannot voxelize planes without boundary information");
    return false;
}

static
bool VoxelizeNonPlaneShape(
    const shapes::Shape& shape,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    switch (shape.type) {
    case shapes::SPHERE: {
        const shapes::Sphere* sphere =
                dynamic_cast<const shapes::Sphere*>(&shape);
        if (!sphere) {
            return false;
        }
        return VoxelizeSphere(*sphere, pose, res, go, voxels);
    }   break;
    case shapes::CYLINDER: {
        const shapes::Cylinder* cylinder =
                dynamic_cast<const shapes::Cylinder*>(&shape);
        if (!cylinder) {
            return false;
        }
        return VoxelizeCylinder(*cylinder, pose, res, go, voxels);
    }   break;
    case shapes::CONE: {
        const shapes::Cone* cone = dynamic_cast<const shapes::Cone*>(&shape);
        if (!cone) {
            return false;
        }
        return VoxelizeCone(*cone, pose, res, go, voxels);
    }   break;
    case shapes::BOX: {
        const shapes::Box* box = dynamic_cast<const shapes::Box*>(&shape);
        if (!box) {
            return false;
        }
        return VoxelizeBox(*box, pose, res, go, voxels);
    }   break;
    case shapes::PLANE: {
        const shapes::Plane* plane = dynamic_cast<const shapes::Plane*>(&shape);
        if (!plane) {
            return false;
        }
        return VoxelizePlane(*plane, pose, res, go, voxels);
    }   break;
    case shapes::MESH: {
        const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(&shape);
        if (!mesh) {
            return false;
        }
        return VoxelizeMesh(*mesh, pose, res, go, voxels);
    }   break;
    case shapes::OCTREE: {
        const shapes::OcTree* octree =
                dynamic_cast<const shapes::OcTree*>(&shape);
        if (!octree) {
            return false;
        }
        return VoxelizeOcTree(*octree, pose, res, go, voxels);
    }   break;
    }

    return false;
}

/// Voxelize an object composed of several shapes
///
/// This function disallows plane voxelization, which requires additional
/// boundary information.
bool VoxelizeObject(
    const Object& object,
    double res,
    const Eigen::Vector3d& go,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels)
{
    for (size_t i = 0; i < object.shapes_.size(); ++i) {
        const shapes::ShapeConstPtr& shape = object.shapes_[i];
        const Eigen::Affine3d& pose = object.shape_poses_[i];
        std::vector<Eigen::Vector3d> voxels;
        if (!VoxelizeShape(*shape, pose, res, go, voxels)) {
            all_voxels.clear();
            return false;
        }
        all_voxels.push_back(std::move(voxels));
    }

    return true;
}

/// Voxelize an message object composed of several shapes
///
/// This function disallows plane voxelization, which requires additional
/// boundary information.
bool VoxelizeCollisionObject(
    const moveit_msgs::CollisionObject& object,
    double res,
    const Eigen::Vector3d& go,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels)
{
    if (!object.planes.empty()) {
        ROS_ERROR("Failed to voxelize plane of collision object '%s'", object.id.c_str());
        return false;
    }

    if (!VoxelizeSolidPrimitives(object, res, go, all_voxels)) {
        all_voxels.clear();
        return false;
    }

    // gather voxels from all meshes
    if (!VoxelizeMeshes(object, res, go, all_voxels)) {
        all_voxels.clear();
        return false;
    }

    return true;
}

bool VoxelizeObject(
    const Object& object,
    double res,
    const Eigen::Vector3d& go,
    const Eigen::Vector3d& gmin,
    const Eigen::Vector3d& gmax,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels)
{
    for (size_t i = 0; i < object.shapes_.size(); ++i) {
        const shapes::ShapeConstPtr& shape = object.shapes_[i];
        const Eigen::Affine3d& pose = object.shape_poses_[i];
        std::vector<Eigen::Vector3d> voxels;
        if (!VoxelizeShape(*shape, pose, res, go, gmin, gmax, voxels)) {
            all_voxels.clear();
            return false;
        }
        all_voxels.push_back(std::move(voxels));
    }

    return true;
}

bool VoxelizeCollisionObject(
    const moveit_msgs::CollisionObject& object,
    double res,
    const Eigen::Vector3d& go,
    const Eigen::Vector3d& gmin,
    const Eigen::Vector3d& gmax,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels)
{
    if (!VoxelizePlanes(object, res, go, gmin, gmax, all_voxels)) {
        all_voxels.clear();
        return false;
    }

    if (!VoxelizeSolidPrimitives(object, res, go, all_voxels)) {
        all_voxels.clear();
        return false;
    }

    // gather voxels from all meshes
    if (!VoxelizeMeshes(object, res, go, all_voxels)) {
        all_voxels.clear();
        return false;
    }

    return true;
}

bool VoxelizeShape(
    const shapes::Shape& shape,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    return VoxelizeNonPlaneShape(shape, pose, res, go, voxels);
}

bool VoxelizeShape(
    const shapes::Shape& shape,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    const Eigen::Vector3d& gmin,
    const Eigen::Vector3d& gmax,
    std::vector<Eigen::Vector3d>& voxels)
{
    if (shape.type != shapes::PLANE) {
        return VoxelizeNonPlaneShape(shape, pose, res, go, voxels);
    } else {
        const shapes::Plane* plane = dynamic_cast<const shapes::Plane*>(&shape);
        if (!plane) {
            return false;
        }
        return VoxelizePlane(*plane, pose, res, go, gmin, gmax, voxels);
    }

    return false;
}

bool VoxelizeSphere(
    const shapes::Sphere& sphere,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double radius = sphere.radius;
    geometry::VoxelizeSphere(radius, pose, res, go, voxels, false);
    return true;
}

bool VoxelizeCylinder(
    const shapes::Cylinder& cylinder,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double height = cylinder.length;
    const double radius = cylinder.radius;
    geometry::VoxelizeCylinder(radius, height, pose, res, go, voxels, false);
    return true;
}

bool VoxelizeCone(
    const shapes::Cone& cone,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double height = cone.length;
    const double radius = cone.radius;
    geometry::VoxelizeCone(radius, height, pose, res, go, voxels, false);
    return true;
}

bool VoxelizeBox(
    const shapes::Box& box,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double length = box.size[0];
    const double width = box.size[1];
    const double height = box.size[2];
    geometry::VoxelizeBox(length, width, height, pose, res, go, voxels, false);
    return true;
}

bool VoxelizePlane(
    const shapes::Plane& plane,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    const Eigen::Vector3d& gmin,
    const Eigen::Vector3d& gmax,
    std::vector<Eigen::Vector3d>& voxels)
{
    // TODO: incorporate pose
    geometry::VoxelizePlane(
            plane.a, plane.b, plane.c, plane.d, gmin, gmax, res, go, voxels);
    return true;
}

bool VoxelizeMesh(
    const shapes::Mesh& mesh,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    std::vector<Eigen::Vector3d> vertices(mesh.vertex_count);
    for (unsigned int i = 0; i < mesh.vertex_count; ++i) {
        vertices[i] = Eigen::Vector3d(
                mesh.vertices[3 * i + 0],
                mesh.vertices[3 * i + 1],
                mesh.vertices[3 * i + 2]);
    }
    std::vector<int> indices(mesh.triangles, mesh.triangles + 3 * mesh.triangle_count);
    geometry::VoxelizeMesh(vertices, indices, pose, res, go, voxels, false);
    return true;
}

bool VoxelizeOcTree(
    const shapes::OcTree& octree,
    const Eigen::Affine3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    auto tree = octree.octree;
    for (auto lit = tree->begin_leafs(); lit != tree->end_leafs(); ++lit) {
        if (tree->isNodeOccupied(*lit)) {
            if (lit.getSize() <= res) {
                voxels.push_back(Eigen::Vector3d(lit.getX(), lit.getY(), lit.getZ()));
            } else {
                double ceil_val = ceil(lit.getSize() / res) * res;
                for (double x = lit.getX() - ceil_val; x < lit.getX() + ceil_val; x += res) {
                for (double y = lit.getY() - ceil_val; y < lit.getY() + ceil_val; y += res) {
                for (double z = lit.getZ() - ceil_val; z < lit.getZ() + ceil_val; z += res) {
                    Eigen::Vector3d pt(x, y, z);
                    pt = pose * pt;
                    voxels.push_back(pt);
                }
                }
                }
            }
        }
    }

    return true;
}

bool VoxelizeSolidPrimitive(
    const shape_msgs::SolidPrimitive& prim,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    switch (prim.type) {
    case shape_msgs::SolidPrimitive::BOX: {
        if (!VoxelizeBox(prim, pose, res, go, voxels)) {
            ROS_ERROR("Failed to voxelize box");
            return false;
        }
    }   break;
    case shape_msgs::SolidPrimitive::SPHERE: {
        if (!VoxelizeSphere(prim, pose, res, go, voxels)) {
            ROS_ERROR("Failed to voxelize sphere");
            return false;
        }
    }   break;
    case shape_msgs::SolidPrimitive::CYLINDER: {
        if (!VoxelizeCylinder(prim, pose, res, go, voxels)) {
            ROS_ERROR("Failed to voxelize cylinder");
            return false;
        }
    }   break;
    case shape_msgs::SolidPrimitive::CONE: {
        if (!VoxelizeCone(prim, pose, res, go, voxels)) {
            ROS_ERROR("Failed to voxelize cone");
            return false;
        }
    }   break;
    }

    return true;
}

bool VoxelizeBox(
    const shape_msgs::SolidPrimitive& box,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double length = box.dimensions[shape_msgs::SolidPrimitive::BOX_X];
    const double width = box.dimensions[shape_msgs::SolidPrimitive::BOX_Y];
    const double height = box.dimensions[shape_msgs::SolidPrimitive::BOX_Z];

    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(pose, eigen_pose);

    std::vector<Eigen::Vector3d> sbpl_voxels;
    geometry::VoxelizeBox(
            length, width, height, eigen_pose, res, go, sbpl_voxels, false);

    voxels.insert(voxels.end(), sbpl_voxels.begin(), sbpl_voxels.end());
    return true;
}

bool VoxelizeSphere(
    const shape_msgs::SolidPrimitive& sphere,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double radius =
            sphere.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS];

    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(pose, eigen_pose);

    std::vector<Eigen::Vector3d> sbpl_voxels;
    geometry::VoxelizeSphere(radius, eigen_pose, res, go, sbpl_voxels, false);

    voxels.insert(voxels.end(), sbpl_voxels.begin(), sbpl_voxels.end());
    return true;
}

bool VoxelizeCylinder(
    const shape_msgs::SolidPrimitive& cylinder,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double height = cylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
    const double radius = cylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];

    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(pose, eigen_pose);

    std::vector<Eigen::Vector3d> sbpl_voxels;
    geometry::VoxelizeCylinder(
            radius, height, eigen_pose, res, go, sbpl_voxels, false);

    voxels.insert(voxels.end(), sbpl_voxels.begin(), sbpl_voxels.end());
    return true;
}

bool VoxelizeCone(
    const shape_msgs::SolidPrimitive& cone,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    const double height = cone.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT];
    const double radius = cone.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS];

    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(pose, eigen_pose);

    std::vector<Eigen::Vector3d> sbpl_voxels;
    geometry::VoxelizeCone(radius, height, eigen_pose, res, go, sbpl_voxels, false);

    voxels.insert(voxels.end(), sbpl_voxels.begin(), sbpl_voxels.end());
    return true;
}

bool VoxelizeMesh(
    const shape_msgs::Mesh& mesh,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels)
{
    std::vector<Eigen::Vector3d> vertices;
    vertices.resize(mesh.vertices.size());
    for (size_t vidx = 0; vidx < mesh.vertices.size(); ++vidx) {
        const geometry_msgs::Point& vertex = mesh.vertices[vidx];
        vertices[vidx] = Eigen::Vector3d(vertex.x, vertex.y, vertex.z);
    }

    std::vector<int> indices = ConvertToVertexIndices(mesh.triangles);

    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(pose, eigen_pose);

    std::vector<Eigen::Vector3d> sbpl_voxels;
    geometry::VoxelizeMesh(vertices, indices, eigen_pose, res, go, sbpl_voxels, false);

    voxels.insert(voxels.end(), sbpl_voxels.begin(), sbpl_voxels.end());
    return true;
}

bool VoxelizePlane(
    const shape_msgs::Plane& plane,
    const geometry_msgs::Pose& pose,
    double res,
    const Eigen::Vector3d& go,
    const Eigen::Vector3d& gmin,
    const Eigen::Vector3d& gmax,
    std::vector<Eigen::Vector3d>& voxels)
{
    Eigen::Affine3d eigen_pose;
    tf::poseMsgToEigen(pose, eigen_pose);

    // TODO: incorporate pose
    geometry::VoxelizePlane(
            plane.coef[0], plane.coef[1], plane.coef[2], plane.coef[3],
            gmin, gmax, res, go, voxels);
    return true;
}

} // namespace collision
} // namespace sbpl
