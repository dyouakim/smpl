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

// standard includes
#include <sstream>

// system includes
#include <leatherman/print.h>

// project includes
#include <sbpl_collision_checking/base_collision_models.h>

namespace sbpl {
namespace collision {

namespace sphere_traits {

template <typename Sphere> struct radius { };
template <typename Sphere> struct center_x { };
template <typename Sphere> struct center_y { };
template <typename Sphere> struct center_z { };

template <>
struct radius<CollisionSphereConfig>
{
    static double get(const CollisionSphereConfig& sphere) {
        return sphere.radius;
    }
};

template <>
struct radius<CollisionSphereModel>
{
    static double get(const CollisionSphereModel& sphere) {
        return sphere.radius;
    }
};

template <>
struct center_x<CollisionSphereConfig>
{
    static double get(const CollisionSphereConfig& sphere) {
        return sphere.x;
    }
};

template <>
struct center_x<CollisionSphereModel>
{
    static double get(const CollisionSphereModel& sphere) {
        return sphere.center.x();
    }
};

template <>
struct center_y<CollisionSphereConfig>
{
    static double get(const CollisionSphereConfig& sphere) {
        return sphere.y;
    }
};

template <>
struct center_y<CollisionSphereModel>
{
    static double get(const CollisionSphereModel& sphere) {
        return sphere.center.y();
    }
};

template <>
struct center_z<CollisionSphereConfig>
{
    static double get(const CollisionSphereConfig& sphere) {
        return sphere.z;
    }
};

template <>
struct center_z<CollisionSphereModel>
{
    static double get(const CollisionSphereModel& sphere) {
        return sphere.center.z();
    }
};

} // namespace sphere_traits

template <typename Sphere>
inline double get_radius(const Sphere& sphere)
{
    return sphere_traits::radius<Sphere>::get(sphere);
}

template <typename Sphere>
inline double get_x(const Sphere& sphere)
{
    return sphere_traits::center_x<Sphere>::get(sphere);
}

template <typename Sphere>
inline double get_y(const Sphere& sphere)
{
    return sphere_traits::center_y<Sphere>::get(sphere);
}

template <typename Sphere>
inline double get_z(const Sphere& sphere)
{
    return sphere_traits::center_z<Sphere>::get(sphere);
}

template <typename Sphere>
struct SpherePartitionerX
{
    double x;
    SpherePartitionerX(double x) : x(x) { }
    bool operator()(const Sphere* s) const {
        return get_x(*s) < x;
    }
};

template <typename Sphere>
struct SpherePartitionerY
{
    double y;
    SpherePartitionerY(double y) : y(y) { }
    bool operator()(const Sphere* s) const {
        return get_y(*s) < y;
    }
};

template <typename Sphere>
struct SpherePartitionerZ
{
    double z;
    SpherePartitionerZ(double z) : z(z) { }
    bool operator()(const Sphere* s) const {
        return get_z(*s) < z;
    }
};

std::ostream& operator<<(std::ostream& o, const CollisionSphereModel& csm)
{
    o << "{ name: " << csm.name << ", center: (" << csm.center.x() << ", " <<
            csm.center.y() << ", " << csm.center.z() << "), radius: " <<
            csm.radius << ", priority: " << csm.priority << " }";
    return o;
}

CollisionSphereModelTree::CollisionSphereModelTree(CollisionSphereModelTree&& o) :
    m_tree(std::move(o.m_tree))
{
}

void CollisionSphereModelTree::buildFrom(
    const std::vector<CollisionSphereConfig>& spheres)
{
    m_tree.clear();

    // create array of pointers to configuration structures, to allow efficient
    // partitioning of the set
    std::vector<const CollisionSphereConfig*> sptrs(spheres.size());
    for (size_t i = 0; i < spheres.size(); ++i) {
        sptrs[i] = &spheres[i];
    }

    buildRecursive<CollisionSphereConfig>(sptrs.begin(), sptrs.end());

    // rewire the child pointers
    size_t leaf_count = 0;
    for (size_t i = 0; i < m_tree.size(); ++i) {
        CollisionSphereModel& sphere = m_tree[i];
        const size_t li = reinterpret_cast<size_t>(sphere.left);
        const size_t ri = reinterpret_cast<size_t>(sphere.right);
        if (li != std::numeric_limits<size_t>::max()) {
            sphere.left = &m_tree[li];
        }
        else {
            sphere.left = nullptr;
            ++leaf_count;
        }
        if (ri != std::numeric_limits<size_t>::max()) {
            sphere.right = &m_tree[ri];
        }
        else {
            sphere.right = nullptr;
        }
    }
    ROS_DEBUG("%zu leaves", leaf_count);

    // TODO: play around with the layout (ordering) of nodes in the compact
    // array to see if efficiency is affected
}

void CollisionSphereModelTree::buildFrom(
    const std::vector<CollisionSphereModel>& spheres)
{
    m_tree.clear();

    // duplicate vector to allow partitioning
    std::vector<const CollisionSphereModel*> sptrs(spheres.size());
    for (size_t i = 0; i < spheres.size(); ++i) {
        sptrs[i] = &spheres[i];
    }

    buildRecursive<CollisionSphereModel>(sptrs.begin(), sptrs.end());
    const CollisionSphereModel* root = &m_tree[0];
    size_t leaf_count = 0;
    for (size_t i = 0; i < m_tree.size(); ++i) {
        CollisionSphereModel& sphere = m_tree[i];
        const size_t li = reinterpret_cast<size_t>(sphere.left);
        const size_t ri = reinterpret_cast<size_t>(sphere.right);
        if (li != std::numeric_limits<size_t>::max()) {
            sphere.left = &m_tree[li];
        }
        else {
            sphere.left = nullptr;
            ++leaf_count;
        }
        if (ri != std::numeric_limits<size_t>::max()) {
            sphere.right = &m_tree[ri];
        }
        else {
            sphere.right = nullptr;
        }
    }
    ROS_DEBUG("%zu leaves", leaf_count);
}

void CollisionSphereModelTree::buildFrom(
    const std::vector<const CollisionSphereModel*>& spheres)
{
    ROS_DEBUG("Buliding Collision Sphere Model Tree with %zu model spheres", spheres.size());
    m_tree.clear();

    std::vector<const CollisionSphereModel*> sptrs = spheres;
    buildMetaRecursive(sptrs.begin(), sptrs.end());

    size_t leaf_count = 0;
    for (size_t i = 0; i < m_tree.size(); ++i) {
        CollisionSphereModel& sphere = m_tree[i];
        const size_t li = reinterpret_cast<size_t>(sphere.left);
        const size_t ri = reinterpret_cast<size_t>(sphere.right);
        ROS_DEBUG("Sphere [(%0.3f, %0.3f, %0.3f), %0.3f, %zu, %zu]",
                sphere.center.x(), sphere.center.y(), sphere.center.z(),
                sphere.radius, li, ri);
        if (li != ri) {
            sphere.left = &m_tree[li];
            sphere.right = &m_tree[ri];
        }
        else {
            // left/right point to the derived sphere model
            ++leaf_count;
        }
    }
    ROS_DEBUG("%zu leaves", leaf_count);
}

double CollisionSphereModelTree::maxRadius() const
{
    auto radius_comp = [](
        const CollisionSphereModel& s1,
        const CollisionSphereModel& s2)
    {
        return s1.radius < s2.radius;
    };

    auto it = std::max_element(m_tree.begin(), m_tree.end(), radius_comp);
    if (it == m_tree.end()) {
        return 0.0;
    }
    else {
        return it->radius;
    }
}

double CollisionSphereModelTree::maxLeafRadius() const
{
    auto radius_comp = [](
        const CollisionSphereModel& s1,
        const CollisionSphereModel& s2)
    {
        if (s1.isLeaf() && s2.isLeaf()) {
            return s1.radius < s2.radius;
        }
        else if (s1.isLeaf()) {
            return true;
        }
        else if (s2.isLeaf()) {
            return false;
        }
        else {
            return false;
        }
    };

    auto it = std::max_element(m_tree.begin(), m_tree.end(), radius_comp);
    if (it == m_tree.end()) {
        return 0.0;
    }
    else {
        return it->radius;
    }
}

/// \brief Compute the bounding sphere tree for a subset of model spheres
/// \return The index into \p m_tree where the top-most bounding sphere was stored
template <typename Sphere>
size_t CollisionSphereModelTree::buildRecursive(
    typename std::vector<const Sphere*>::iterator msfirst,
    typename std::vector<const Sphere*>::iterator mslast)
{
    if (mslast == msfirst) {
        ROS_DEBUG("Zero spheres base case");
        return -1;
    }

    const auto count = std::distance(msfirst, mslast);
    if (count == 1) {
        m_tree.emplace_back();
        CollisionSphereModel& cs = m_tree.back();
        const Sphere& s = **msfirst;
        cs.name = s.name; // ok, no i'm not making traits for these
        cs.center = Eigen::Vector3d(get_x(s), get_y(s), get_z(s));
        cs.radius = get_radius(s);
        cs.priority = s.priority; // ...or this
        reinterpret_cast<size_t&>(cs.left) = std::numeric_limits<size_t>::max();
        reinterpret_cast<size_t&>(cs.right) = std::numeric_limits<size_t>::max();
        ROS_DEBUG("Leaf sphere '%s'", cs.name.c_str());
        const size_t this_idx = m_tree.size() - 1;
        return this_idx;
    }

    ROS_DEBUG("Building hierarchy from %zu spheres", count);

    // compute the largest axis of the bounding box along which to split
    const int split_axis = computeLargestBoundingBoxAxis<Sphere>(msfirst, mslast);
    ROS_DEBUG("Splitting along axis %d", split_axis);

    // compute the average centroid of all spheres at this level
    Eigen::Vector3d compact_bounding_sphere_center = Eigen::Vector3d::Zero();
    for (auto it = msfirst; it != mslast; ++it) {
        const Sphere& s = **it;
        compact_bounding_sphere_center += Eigen::Vector3d(get_x(s), get_y(s), get_z(s));
    }
    compact_bounding_sphere_center /= count;

    // compute the radius required to encompass all model spheres at this level
    // with a sphere rooted at the model spheres centroid
    double compact_bounding_sphere_radius = 0.0;
    for (auto it = msfirst; it != mslast; ++it) {
        Eigen::Vector3d c(get_x(**it), get_y(**it), get_z(**it));
        double radius = (c - compact_bounding_sphere_center).norm() + get_radius(**it);
        if (radius > compact_bounding_sphere_radius) {
            compact_bounding_sphere_radius = radius;
        }
    }

    auto part = [&compact_bounding_sphere_center](
        int split_axis,
        typename std::vector<const Sphere*>::iterator first,
        typename std::vector<const Sphere*>::iterator last)
    {
        if (split_axis == 0) {
            return std::partition(first, last, SpherePartitionerX<Sphere>(compact_bounding_sphere_center.x()));
        }
        else if (split_axis == 1) {
            return std::partition(first, last, SpherePartitionerY<Sphere>(compact_bounding_sphere_center.y()));
        }
        else {
            return std::partition(first, last, SpherePartitionerZ<Sphere>(compact_bounding_sphere_center.z()));
        }
    };
    // split the tree along the largest axis by the centroid
    auto msmid = part(split_axis, msfirst, mslast);
    if (msfirst == msmid || msmid == mslast) {
        msmid = msfirst + (std::distance(msfirst, mslast) >> 1);
    }

    // recurse on both subtrees
    const size_t left_idx = buildRecursive<Sphere>(msfirst, msmid);
    const size_t right_idx = buildRecursive<Sphere>(msmid, mslast);

    const CollisionSphereModel& sl = m_tree[left_idx];
    const CollisionSphereModel& sr = m_tree[right_idx];

    // compute the optimal sphere to contain both child spheres; mathematical!
    Eigen::Vector3d greedy_bounding_sphere_center;
    double greedy_bounding_sphere_radius;
    computeOptimalBoundingSphere(
            sl, sr, greedy_bounding_sphere_center, greedy_bounding_sphere_radius);

    // create the collision sphere model
    m_tree.emplace_back();
    size_t this_idx = m_tree.size() - 1;

    ROS_DEBUG("child bounding sphere: (%0.3f, %0.3f, %0.3f), %0.3f", greedy_bounding_sphere_center.x(), greedy_bounding_sphere_center.y(), greedy_bounding_sphere_center.z(), greedy_bounding_sphere_radius);
    ROS_DEBUG("model bounding sphere: (%0.3f, %0.3f, %0.3f), %0.3f", compact_bounding_sphere_center.x(), compact_bounding_sphere_center.y(), compact_bounding_sphere_center.z(), compact_bounding_sphere_radius);

    CollisionSphereModel& sphere = m_tree[this_idx];
    if (greedy_bounding_sphere_radius < compact_bounding_sphere_radius) {
        sphere.center = greedy_bounding_sphere_center;
        sphere.radius = greedy_bounding_sphere_radius;
    }
    else {
        sphere.center = compact_bounding_sphere_center;
        sphere.radius = compact_bounding_sphere_radius;
    }
//    sphere.name;
    sphere.priority = 0;
    reinterpret_cast<size_t&>(sphere.left) = left_idx;
    reinterpret_cast<size_t&>(sphere.right) = right_idx;
    ROS_DEBUG("sptr: %p", &sphere);
    return this_idx;
}

size_t CollisionSphereModelTree::buildMetaRecursive(
    std::vector<const CollisionSphereModel*>::iterator msfirst,
    std::vector<const CollisionSphereModel*>::iterator mslast)
{
    if (mslast == msfirst) {
        ROS_DEBUG("Zero spheres base case");
        return -1;
    }

    const auto count = std::distance(msfirst, mslast);
    if (count == 1) {
        m_tree.emplace_back();
        CollisionSphereModel& cs = m_tree.back();
        const CollisionSphereModel& s = **msfirst;
        cs.name = s.name; // ok, no i'm not making traits for these
        cs.center = Eigen::Vector3d(get_x(s), get_y(s), get_z(s));
        cs.radius = get_radius(s);
        cs.priority = s.priority; // ...or this
        cs.left = *msfirst;
        cs.right = *msfirst;
        ROS_DEBUG("Leaf sphere '%s'", cs.name.c_str());
        const size_t this_idx = m_tree.size() - 1;
        return this_idx;
    }

    ROS_DEBUG("Building hierarchy from %zu spheres", count);

    // compute the largest axis of the bounding box along which to split
    const int split_axis = computeLargestBoundingBoxAxis<CollisionSphereModel>(msfirst, mslast);
    ROS_DEBUG("Splitting along axis %d", split_axis);

    // compute the average centroid of all spheres at this level
    Eigen::Vector3d compact_bounding_sphere_center = Eigen::Vector3d::Zero();
    for (auto it = msfirst; it != mslast; ++it) {
        const CollisionSphereModel& s = **it;
        compact_bounding_sphere_center += Eigen::Vector3d(get_x(s), get_y(s), get_z(s));
    }
    compact_bounding_sphere_center /= count;

    // compute the radius required to encompass all model spheres at this level
    // with a sphere rooted at the model spheres centroid
    double compact_bounding_sphere_radius = 0.0;
    for (auto it = msfirst; it != mslast; ++it) {
        Eigen::Vector3d c(get_x(**it), get_y(**it), get_z(**it));
        double radius = (c - compact_bounding_sphere_center).norm() + get_radius(**it);
        if (radius > compact_bounding_sphere_radius) {
            compact_bounding_sphere_radius = radius;
        }
    }

    // split the tree along the largest axis by the centroid
    auto pred = [&](const CollisionSphereModel* s)
    {
        switch (split_axis) {
        case 0:
            return get_x(*s) < compact_bounding_sphere_center.x();
        case 1:
            return get_y(*s) < compact_bounding_sphere_center.y();
        case 2:
            return get_z(*s) < compact_bounding_sphere_center.z();
        default:
            return get_x(*s) < compact_bounding_sphere_center.x();
        }
    };

    auto part = [&compact_bounding_sphere_center](
        int split_axis,
        typename std::vector<const CollisionSphereModel*>::iterator first,
        typename std::vector<const CollisionSphereModel*>::iterator last)
    {
        if (split_axis == 0) {
            return std::partition(first, last, SpherePartitionerX<CollisionSphereModel>(compact_bounding_sphere_center.x()));
        }
        else if (split_axis == 1) {
            return std::partition(first, last, SpherePartitionerY<CollisionSphereModel>(compact_bounding_sphere_center.y()));
        }
        else {
            return std::partition(first, last, SpherePartitionerZ<CollisionSphereModel>(compact_bounding_sphere_center.z()));
        }
    };
    // split the tree along the largest axis by the centroid
    auto msmid = part(split_axis, msfirst, mslast);
    if (msfirst == msmid || msmid == mslast) {
        msmid = msfirst + (std::distance(msfirst, mslast) >> 1);
    }

    // recurse on both subtrees
    const size_t left_idx = buildMetaRecursive(msfirst, msmid);
    const size_t right_idx = buildMetaRecursive(msmid, mslast);

    const CollisionSphereModel& sl = m_tree[left_idx];
    const CollisionSphereModel& sr = m_tree[right_idx];

    // compute the optimal sphere to contain both child spheres; mathematical!
    Eigen::Vector3d greedy_bounding_sphere_center;
    double greedy_bounding_sphere_radius;
    computeOptimalBoundingSphere(
            sl, sr, greedy_bounding_sphere_center, greedy_bounding_sphere_radius);

    // create the collision sphere model
    m_tree.emplace_back();
    size_t this_idx = m_tree.size() - 1;

    ROS_DEBUG("child bounding sphere: (%0.3f, %0.3f, %0.3f), %0.3f", greedy_bounding_sphere_center.x(), greedy_bounding_sphere_center.y(), greedy_bounding_sphere_center.z(), greedy_bounding_sphere_radius);
    ROS_DEBUG("model bounding sphere: (%0.3f, %0.3f, %0.3f), %0.3f", compact_bounding_sphere_center.x(), compact_bounding_sphere_center.y(), compact_bounding_sphere_center.z(), compact_bounding_sphere_radius);

    CollisionSphereModel& sphere = m_tree[this_idx];
    if (greedy_bounding_sphere_radius < compact_bounding_sphere_radius) {
        sphere.center = greedy_bounding_sphere_center;
        sphere.radius = greedy_bounding_sphere_radius;
    }
    else {
        sphere.center = compact_bounding_sphere_center;
        sphere.radius = compact_bounding_sphere_radius;
    }
//    sphere.name; // TODO: worthwhile to name intermediate spheres?
    sphere.priority = 0;
    reinterpret_cast<size_t&>(sphere.left) = left_idx;
    reinterpret_cast<size_t&>(sphere.right) = right_idx;
    ROS_DEBUG("sptr: %p", &sphere);
    return this_idx;
}

void CollisionSphereModelTree::computeOptimalBoundingSphere(
    const CollisionSphereModel& s1,
    const CollisionSphereModel& s2,
    Eigen::Vector3d& c,
    double& r)
{
    const Eigen::Vector3d& p = s1.center;
    const Eigen::Vector3d& q = s2.center;
    Eigen::Vector3d v = q - p;
    const double dist = v.norm();
    if (s1.radius > dist + s2.radius) { // s1 contains s2
        c = s1.center;
        r = s1.radius;
    } else if (s2.radius > dist + s1.radius) { // s2 contains s1
        c = s2.center;
        r = s2.radius;
    } else {
        Eigen::Vector3d vn = v.normalized();
        Eigen::Vector3d a = q + vn * s2.radius;
        Eigen::Vector3d b = p - vn * s1.radius;
        c = 0.5 * (a + b);
        r = 0.5 * (a - b).norm();
    }
}

template <typename Sphere>
int CollisionSphereModelTree::computeLargestBoundingBoxAxis(
    typename std::vector<const Sphere*>::iterator first,
    typename std::vector<const Sphere*>::iterator last)
{
    if (std::distance(first, last) == 0) {
        return 0;
    }

    Eigen::Vector3d min = Eigen::Vector3d(get_x(**first), get_y(**first), get_z(**first));
    Eigen::Vector3d max = Eigen::Vector3d(get_x(**first), get_y(**first), get_z(**first));
    for (auto it = first; it != last; ++it) {
        const Sphere& sphere = **it;
        // update bounding box
        if (get_x(sphere) < min.x()) {
            min.x() = get_x(sphere);
        }
        if (get_y(sphere) < min.y()) {
            min.y() = get_y(sphere);
        }
        if (get_z(sphere) < min.z()) {
            min.z() = get_z(sphere);
        }
        if (get_x(sphere) > max.x()) {
            max.x() = get_x(sphere);
        }
        if (get_y(sphere) > max.y()) {
            max.y() = get_y(sphere);
        }
        if (get_z(sphere) > max.z()) {
            max.z() = get_z(sphere);
        }
    }

    const double spanx = max.x() - min.x();
    const double spany = max.y() - min.y();
    const double spanz = max.z() - min.z();
    if (spanx > spany && spanx > spanz) {
        return 0;
    }
    else if (spany > spanz) {
        return 1;
    }
    else {
        return 2;
    }
    return 0; // all the same
}

std::ostream& operator<<(std::ostream& o, const CollisionSphereModelTree& tree)
{
    o << tree.m_tree;
    return o;
}

std::ostream& operator<<(std::ostream& o, const CollisionSpheresModel& csm)
{
    o << "{ link_index: " << csm.link_index << ", spheres: " << csm.spheres <<
            " }";
    return o;
}

std::ostream& operator<<(std::ostream& o, const CollisionVoxelsModel& cvm)
{
    o << "{ link_index: " << cvm.link_index << ", voxel_res: " <<
            cvm.voxel_res << ", voxels: [" << cvm.voxels.size() << "]" << " }";
    return o;
}

std::ostream& operator<<(std::ostream& o, const CollisionGroupModel& cgm)
{
    o << "{ name: " << cgm.name << ", link_indices: " << cgm.link_indices << " }";
    return o;
}

std::ostream& operator<<(std::ostream& o, const SphereIndex& i)
{
    o << "(" << i.ss << ", " << i.s <<")";
    return o;
}

std::string to_string(const SphereIndex& i)
{
    std::stringstream ss; ss << i; return ss.str();
}

} // namespace collision
} // namespace sbpl
