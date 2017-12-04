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

#ifndef SMPL_SPARSE_GRID_HPP
#define SMPL_SPARSE_GRID_HPP

#include "../sparse_grid.h"

// standard includes
#include <assert.h>
#include <limits>

namespace sbpl {

template <class T, class Allocator>
SparseGrid<T, Allocator>::SparseGrid() :
    m_tree(),
    m_max_depth(16)
{
    m_size[0] = m_size[1] = m_size[2] = 1u << 16;
}

template <class T, class Allocator>
SparseGrid<T, Allocator>::SparseGrid(const T& value) :
    m_tree(value),
    m_max_depth(16)
{
    m_size[0] = m_size[1] = m_size[2] = 1u << 16;
}

template <class T, class Allocator>
SparseGrid<T, Allocator>::SparseGrid(
    size_type size_x,
    size_type size_y,
    size_type size_z)
:
    m_tree(),
    m_max_depth(compute_max_depth(size_x, size_y, size_z))
{
    m_size[0] = size_x;
    m_size[1] = size_y;
    m_size[2] = size_z;
}

template <class T, class Allocator>
SparseGrid<T, Allocator>::SparseGrid(
    size_type size_x,
    size_type size_y,
    size_type size_z,
    const T& value)
:
    m_tree(value),
    m_max_depth(compute_max_depth(size_x, size_y, size_z))
{
    m_size[0] = size_x;
    m_size[1] = size_y;
    m_size[2] = size_z;
}

template <class T, class Allocator>
SparseGrid<T, Allocator>::SparseGrid(const Allocator& alloc) :
    m_tree(alloc),
    m_max_depth(16)
{
    m_size[0] = m_size[1] = m_size[2] = 1u << 16;
}

template <class T, class Allocator>
SparseGrid<T, Allocator>::SparseGrid(const T& value, const Allocator& alloc) :
    m_tree(value, alloc),
    m_max_depth(16)
{
    m_size[0] = m_size[1] = m_size[2] = 1u << 16;
}

template <class T, class Allocator>
SparseGrid<T, Allocator>::SparseGrid(
    size_type size_x, size_type size_y, size_type size_z,
    const Allocator& alloc)
:
    m_tree(alloc),
    m_max_depth(compute_max_depth(size_x, size_y, size_z))
{
    m_size[0] = size_x;
    m_size[1] = size_y;
    m_size[2] = size_z;
}

template <class T, class Allocator>
SparseGrid<T, Allocator>::SparseGrid(
    size_type size_x, size_type size_y, size_type size_z,
    const T& value, const Allocator& alloc)
:
    m_tree(value, alloc),
    m_max_depth(compute_max_depth(size_x, size_y, size_z))
{
    m_size[0] = size_x;
    m_size[1] = size_y;
    m_size[2] = size_z;
}

/// Return the size of the three-dimensional grid. This corresponds to the
/// maximum possible number of leaf nodes for the octree's size. OcTree::size_type OcTree::size() const
template <class T, class Allocator>
typename SparseGrid<T, Allocator>::size_type
SparseGrid<T, Allocator>::size() const
{
    return size_x() * size_y() * size_z();
}

/// Return the maximum possible size of the three-dimensional grid that can be
/// supported by this SparseGrid implementation. This corresponds to the maximum
/// possible number of leaf nodes.
template <class T, class Allocator>
typename SparseGrid<T, Allocator>::size_type
SparseGrid<T, Allocator>::max_size() const
{
    return std::numeric_limits<size_type>::max();
}

/// Return the size of the three-dimensional grid in the x dimension.
template <class T, class Allocator>
typename SparseGrid<T, Allocator>::size_type
SparseGrid<T, Allocator>::size_x() const
{
    return m_size[0];
}

/// Return the size of the three-dimensional grid in the y dimension.
template <class T, class Allocator>
typename SparseGrid<T, Allocator>::size_type
SparseGrid<T, Allocator>::size_y() const
{
    return m_size[1];
}

/// Return the size of the three-dimensional grid in the z dimension.
template <class T, class Allocator>
typename SparseGrid<T, Allocator>::size_type
SparseGrid<T, Allocator>::size_z() const
{
    return m_size[2];
}

template <class T, class Allocator>
int
SparseGrid<T, Allocator>::max_depth() const
{
    return m_max_depth;
}

template <class T, class Allocator>
auto SparseGrid<T, Allocator>::mem_usage() const -> size_type
{
    return m_tree.mem_usage();
}

// TODO: test efficiency of recursion using (x, y, z) vs index arguments
template <class T, class Allocator>
typename SparseGrid<T, Allocator>::const_reference
SparseGrid<T, Allocator>::operator()(index_type x, index_type y, index_type z) const
{
    return get_node(m_max_depth, m_tree.root(), x, y, z)->value;
}

template <class T, class Allocator>
typename SparseGrid<T, Allocator>::reference
SparseGrid<T, Allocator>::operator()(index_type x, index_type y, index_type z)
{
    return get_node_unique(m_max_depth, m_tree.root(), x, y, z)->value;
}

/// Reset so that all cells have identical values. Prunes the SparseGrid fully.
template <class T, class Allocator>
void
SparseGrid<T, Allocator>::reset(const T& value)
{
    m_tree.clear();
    m_tree.root()->value = value;
}

template <class T, class Allocator>
void
SparseGrid<T, Allocator>::assign(const T& value)
{
    return reset(value);
}

template <class T, class Allocator>
void
SparseGrid<T, Allocator>::set(
    index_type x, index_type y, index_type z, const T& data)
{
    set_node(m_max_depth, m_tree.root(), x, y, z, data);
}

template <class T, class Allocator>
void
SparseGrid<T, Allocator>::set_lazy(
    index_type x, index_type y, index_type z, const T& data)
{
    set_node_lazy(m_max_depth, m_tree.root(), x, y, z, data);
}

template <class T, class Allocator>
void SparseGrid<T, Allocator>::prune()
{
    prune(m_tree.root());
}

template <class T, class Allocator>
template <class UnaryPredicate>
void SparseGrid<T, Allocator>::prune(UnaryPredicate p)
{
    prune(m_tree.root(), p);
}

template <class T, class Allocator>
void
SparseGrid<T, Allocator>::resize(
    size_type size_x,
    size_type size_y,
    size_type size_z)
{
    m_tree.clear(); // TODO: non-destructive resize
    m_size[0] = size_x;
    m_size[1] = size_y;
    m_size[2] = size_z;
    m_max_depth = compute_max_depth(size_x, size_y, size_z);
}

template <class T, class Allocator>
void
SparseGrid<T, Allocator>::resize(
    size_type size_x,
    size_type size_y,
    size_type size_z,
    const T& value)
{
    resize(size_x, size_y, size_z);
    m_tree.root()->value = value;
}

template <class T, class Allocator>
typename SparseGrid<T, Allocator>::const_reference
SparseGrid<T, Allocator>::get(index_type x, index_type y, index_type z) const
{
    return get_node(m_max_depth, m_tree.root(), x, y, z)->value;
}

/// Return the approximate number of bytes used by an equivalent dense grid.
template <class T, class Allocator>
template <class Pred>
typename SparseGrid<T, Allocator>::size_type
SparseGrid<T, Allocator>::mem_usage_full(const Pred& pred)
{
    index_type min_coord = 0;
    index_type max_coord = 1 << m_max_depth;
    index_type xmin = max_coord, ymin = max_coord, zmin = max_coord;
    index_type xmax = min_coord, ymax = min_coord, zmax = min_coord;

    auto record_bbx = [&](
        const T& val,
        index_type xfirst, index_type yfirst, index_type zfirst,
        index_type xlast, index_type ylast, index_type zlast)
    {
        if (pred(val)) {
            xmin = std::min(xmin, xfirst);
            ymin = std::min(ymin, yfirst);
            zmin = std::min(zmin, zfirst);
            xmax = std::max(xmax, xlast);
            ymax = std::max(ymax, ylast);
            zmax = std::max(zmax, zlast);
        }
    };

    accept_coords(record_bbx);

    return (xmax - xmin) * (ymax - ymin) * (zmax - zmin) * sizeof(T);
}

template <class T, class Allocator>
template <typename Callable>
void SparseGrid<T, Allocator>::accept(Callable c)
{
    m_tree.accept(c);
}

template <class T, class Allocator>
template <typename Callable>
void SparseGrid<T, Allocator>::accept_coords(Callable c)
{
    int max_coord = 1 << m_max_depth;
    accept_coords(c, m_tree.root(), 0, 0, 0, max_coord, max_coord, max_coord);
}

template <class T, class Allocator>
int SparseGrid<T, Allocator>::compute_max_depth(
    size_type size_x,
    size_type size_y,
    size_type size_z) const
{
    int max_depth = 0;

    int depth_x = (int)std::log2(size_x);
    if (size_x != 1 << depth_x) {
        ++depth_x;
    }
    int depth_y = (int)std::log2(size_y);
    if (size_y != 1 << depth_y) {
        ++depth_y;
    }
    int depth_z = (int)std::log2(size_z);
    if (size_z != 1 << depth_z) {
        ++depth_z;
    }

    max_depth = std::max(max_depth, depth_x);
    max_depth = std::max(max_depth, depth_y);
    max_depth = std::max(max_depth, depth_z);
    return max_depth;
}

template <class T, class Allocator>
const typename SparseGrid<T, Allocator>::node_type*
SparseGrid<T, Allocator>::get_node(
    int rdepth,
    const node_type* n,
    index_type x,
    index_type y,
    index_type z) const
{
    if (!rdepth) {
        return n;
    }

    if (!n->children) {
        return n;
    }

    --rdepth;

    index_type x_loc = x >> rdepth;
    index_type y_loc = y >> rdepth;
    index_type z_loc = z >> rdepth;
    index_type cidx = x_loc << 2 | y_loc << 1 | z_loc;

    return get_node(
            rdepth, &n->children[cidx],
            x - (x_loc << rdepth),
            y - (y_loc << rdepth),
            z - (z_loc << rdepth));
}

template <class T, class Allocator>
typename SparseGrid<T, Allocator>::node_type*
SparseGrid<T, Allocator>::get_node_unique(
    int rdepth,
    node_type* n,
    index_type x,
    index_type y,
    index_type z)
{
    if (!rdepth) {
        return n;
    }

    --rdepth;

    index_type x_loc = x >> rdepth;
    index_type y_loc = y >> rdepth;
    index_type z_loc = z >> rdepth;
    index_type cidx = x_loc << 2 | y_loc << 1 | z_loc;

    if (!n->children) {
        m_tree.expand_node(n);
//        return &n->children[cidx];
    }

    return get_node_unique(
            rdepth, &n->children[cidx],
            x - (x_loc << rdepth),
            y - (y_loc << rdepth),
            z - (z_loc << rdepth));
}

/// Return whether the SparseGrid was modified, as in the following cases:
/// (1) The current node's value was updated
/// (2) The current node's children were collapsed (modifying the node's value)
template <class T, class Allocator>
bool
SparseGrid<T, Allocator>::set_node(
    int rdepth,
    node_type* n,
    index_type x,
    index_type y,
    index_type z,
    const T& value)
{
    if (!rdepth) {
        if (std::equal_to<T>()(n->value, value)) {
            return false;
        }
        n->value = value;
        return true;
    }

    --rdepth;

    index_type x_loc = x >> rdepth;
    index_type y_loc = y >> rdepth;
    index_type z_loc = z >> rdepth;
    index_type cidx = x_loc << 2 | y_loc << 1 | z_loc;

    if (n->children) {
        // need to recurse
    } else if (std::equal_to<T>()(value, n->value)) {
        return false;
    } else {
        m_tree.expand_node(n);
    }

    if (set_node(rdepth, &n->children[cidx],
            x - (x_loc << rdepth),
            y - (y_loc << rdepth),
            z - (z_loc << rdepth),
            value))
    {
        if (collapsible(n)) {
            m_tree.collapse_node(n, value);
            return true;
        }

        return false;
    }

    return false;
}

template <class T, class Allocator>
void
SparseGrid<T, Allocator>::set_node_lazy(
    int rdepth,
    node_type* n,
    index_type x,
    index_type y,
    index_type z,
    const T& value)
{
    if (!rdepth) {
        n->value = value;
        return;
    }

    --rdepth;

    index_type x_loc = x >> rdepth;
    index_type y_loc = y >> rdepth;
    index_type z_loc = z >> rdepth;
    index_type cidx = x_loc << 2 | y_loc << 1 | z_loc;

    if (n->children) {
        // need to recurse
    } else if (std::equal_to<T>()(value, n->value)) {
        return;
    } else {
        m_tree.expand_node(n);
    }

    set_node_lazy(
            rdepth,
            &n->children[cidx],
            x - (x_loc << rdepth),
            y - (y_loc << rdepth),
            z - (z_loc << rdepth),
            value);
}

template <class T, class Allocator>
typename SparseGrid<T, Allocator>::node_type*
SparseGrid<T, Allocator>::create_node(
    int rdepth,
    node_type* n,
    index_type x,
    index_type y,
    index_type z)
{
    if (!rdepth) {
        return n;
    }

    --rdepth;

    index_type x_loc = x >> rdepth;
    index_type y_loc = y >> rdepth;
    index_type z_loc = z >> rdepth;
    index_type cidx = x_loc << 2 | y_loc << 1 | z_loc;

    if (!n->children) {
        m_tree.expand_node(n);
    }

    return create_node(
            rdepth,
            &n->children[cidx],
            x - (x_loc << rdepth),
            y - (y_loc << rdepth),
            z - (z_loc << rdepth));
}

template <class T, class Allocator>
bool SparseGrid<T, Allocator>::collapsible(node_type* n) const
{
    assert(n->children);
    for (node_type* c = n->children + 1; c != n->children + 8; ++c) {
        if (!std::equal_to<T>()(c->value, n->children[0].value)) {
            return false;
        }
    }
    return true;
}

template <class T, class Allocator>
bool SparseGrid<T, Allocator>::prune(node_type* n)
{
    if (n->children) {
        bool all = true;
        for (node_type* c = n->children; c != n->children + 8; ++c) {
            all &= prune(c);
        }
        if (all && collapsible(n)) {
            m_tree.collapse_node(n, n->children[0].value);
            return true;
        } else {
            return false;
        }
    } else {
        return true;
    }
}

template <class T, class Allocator>
template <class UnaryPredicate>
bool SparseGrid<T, Allocator>::prune(node_type* n, UnaryPredicate p)
{
    if (n->children) {
        bool all = true;
        for (node_type* c = n->children; c != n->children + 8; ++c) {
            all &= prune(c, p);
        }
        if (all && collapsible(n) && p(n->value)) {
            m_tree.collapse_node(n, n->children[0].value);
            return true;
        } else {
            return false;
        }
    } else {
        return p(n->value);
    }
}

template <class T, class Allocator>
template <typename Callable>
void SparseGrid<T, Allocator>::accept_coords(
    Callable c, node_type* n,
    size_type first_x, size_type first_y, size_type first_z,
    size_type last_x, size_type last_y, size_type last_z)
{
    if (n->children) {
        size_type x_span = last_x - first_x;
        size_type y_span = last_y - first_y;
        size_type z_span = last_z - first_z;

        size_type x_shift = x_span >> 1;
        size_type y_shift = y_span >> 1;
        size_type z_shift = z_span >> 1;

        accept_coords(c, &n->children[0], first_x,           first_y,           first_z,           first_x + x_shift, first_y + y_shift, first_z + z_shift);
        accept_coords(c, &n->children[1], first_x,           first_y,           first_z + z_shift, first_x + x_shift, first_y + y_shift, last_z           );
        accept_coords(c, &n->children[2], first_x,           first_y + y_shift, first_z,           first_x + x_shift, last_y,            first_z + z_shift);
        accept_coords(c, &n->children[3], first_x,           first_y + y_shift, first_z + z_shift, first_x + x_shift, last_y,            last_z           );
        accept_coords(c, &n->children[4], first_x + x_shift, first_y,           first_z,           last_x,            first_y + y_shift, first_z + z_shift);
        accept_coords(c, &n->children[5], first_x + x_shift, first_y,           first_z + z_shift, last_x,            first_y + y_shift, last_z           );
        accept_coords(c, &n->children[6], first_x + x_shift, first_y + y_shift, first_z,           last_x,            last_y,            first_z + z_shift);
        accept_coords(c, &n->children[7], first_x + x_shift, first_y + y_shift, first_z + z_shift, last_x,            last_y,            last_z           );
    } else {
        c(n->value, first_x, first_y, first_z, last_x, last_y, last_z);
    }
}

} // namespace sbpl

#endif
