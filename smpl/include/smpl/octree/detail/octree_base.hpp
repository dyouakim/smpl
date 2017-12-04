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

#ifndef SMPL_OCTREE_BASE_HPP
#define SMPL_OCTREE_BASE_HPP

#include "octree_base.h"

namespace sbpl {
namespace detail {

template <class T, class Allocator>
const typename OcTreeBase<T, Allocator>::node_allocator_type&
OcTreeBase<T, Allocator>::get_node_allocator() const noexcept
{
    return *static_cast<const node_allocator_type*>(&m_impl);
}

template <class T, class Allocator>
typename OcTreeBase<T, Allocator>::node_allocator_type&
OcTreeBase<T, Allocator>::get_node_allocator() noexcept
{
    return *static_cast<node_allocator_type*>(&m_impl);
}

template <class T, class Allocator>
typename OcTreeBase<T, Allocator>::type_allocator_type
OcTreeBase<T, Allocator>::get_type_allocator() const noexcept
{
    return type_allocator_type(get_node_allocator());
}

template <class T, class Allocator>
void
OcTreeBase<T, Allocator>::alloc_children(node_type *n)
{
    n->children = natraits::allocate(m_impl, 8);
}

template <class T, class Allocator>
void
OcTreeBase<T, Allocator>::dealloc_children(node_type *n)
{
    natraits::deallocate(m_impl, n->children, 8);
}

template <class T, class Allocator>
template <typename... Args>
void
OcTreeBase<T, Allocator>::construct_node(node_type *n, Args&&... args)
{
    natraits::construct(m_impl, n, std::forward<Args>(args)...);
}

template <class T, class Allocator>
void
OcTreeBase<T, Allocator>::destroy_node(node_type *n)
{
    natraits::destroy(m_impl, n);
}

// TODO: this should require a mask indicating which children to construct
// since we may not care about all children. This entire loop can also be
// avoided for trivial types.
template <class T, class Allocator>
template <typename... Args>
void
OcTreeBase<T, Allocator>::construct_children(node_type *n, Args&&... args)
{
    node_type* nit = n->children;
    node_type* nend = nit + 8;
    for (; nit != nend; ++nit) {
        construct_node(nit, std::forward<Args>(args)...);
    }
}

template <class T, class Allocator>
void
OcTreeBase<T, Allocator>::destroy_children(node_type *n)
{
    node_type* nit = n->children;
    node_type* nend = nit + 8;
    for (; nit != nend; ++nit) {
        destroy_node(nit);
    }
}

/// Clear a node by recursively destroying and deallocating its children. Does
/// NOT destroy the root node.
template <class T, class Allocator>
void
OcTreeBase<T, Allocator>::clear_node(node_type *node)
{
    if (node->children) {
        for (node_type *c = node->children; c < node->children + 8; ++c) {
            clear_node(c);
            destroy_node(c);
        }

        dealloc_children(node);
    }
}

} // namespace detail
} // namespace sbpl

#endif
