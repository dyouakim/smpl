
#ifndef SMPL_MULTI_INDEX_INTRUSIVE_HEAP_H
#define SMPL_MULTI_INDEX_INTRUSIVE_HEAP_H

#include <cstdlib>
#include <vector>
#include <smpl/console/console.h>
namespace sbpl {

template <class T, class Compare>
class multi_index_intrusive_heap;

struct multi_index_heap_element
{

    multi_index_heap_element() : m_heap_index({0,0}) { }
        
        
private:
    std::size_t m_heap_index[3];

    template <class T, class Compare>
    friend class multi_index_intrusive_heap;
};

/// Provides an intrusive binary heap implementation. Objects inserted into the
/// heap must derive from the \p heap_element class to implement efficient
/// mutability. The implementation stores pointers to inserted objects, which
/// must remain valid throughout the lifetime of the heap.
///
/// The binary heap data structure provides constant time access to the minimum
/// element of the heap, logarithmic insertion and erasure of elements,
/// logarithmic updates of element priorities, and linear time construction from
/// a new set of elements. All times are proportional to the number of elements
/// in the heap.
///
/// Priorities of elements in the heap are not explicitly stored, but determined
/// from the result of calling the \p Compare function object on two elements.
/// If the priorities of multiple elements are implicitly changed (via external
/// modification of the function object), the heap may be reordered in-place
/// in linear time by calling the make() member function.
template <class T, class Compare>
class multi_index_intrusive_heap
{
public:

    static_assert(std::is_base_of<multi_index_heap_element, T>::value, "T must extend heap_element");

    typedef Compare compare;

    typedef std::vector<T*> container_type;
    typedef typename container_type::size_type size_type;

    typedef typename container_type::iterator iterator;
    typedef typename container_type::const_iterator const_iterator;

    multi_index_intrusive_heap(const compare& comp = compare());

    template <class InputIt>
    multi_index_intrusive_heap(InputIt first, InputIt last, int idx);

    template <class InputIt>
    multi_index_intrusive_heap(const compare& comp, InputIt first, InputIt last, int idx);

    multi_index_intrusive_heap(const multi_index_intrusive_heap&) = delete;

    multi_index_intrusive_heap(multi_index_intrusive_heap&& o);

    multi_index_intrusive_heap& operator=(const multi_index_intrusive_heap&) = delete;
    multi_index_intrusive_heap& operator=(multi_index_intrusive_heap&& rhs);

    T* min() const;

    const_iterator begin() const;
    const_iterator end() const;

    bool empty() const;
    size_type size() const;
    size_type max_size() const;
    void reserve(size_type new_cap);

    void clear(int idx);
    void push(T* e, int idx);
    void pop(int idx);
    bool contains(T* e, int idx);
    void update(T* e, int idx);
    void increase(T* e, int idx);
    void decrease(T* e, int idx);
    void erase(T* e, int idx);

    void make(int idx);

    void swap(multi_index_intrusive_heap& o);

    void print(int idx) const;
private:

    container_type m_data;
    Compare m_comp;

    size_type ipow2(size_type i);
    size_type ilog2(size_type i);
    bool ispow2(size_type val);

    template <class InputIt>
    void make_heap(InputIt first, InputIt last, int idx);

    template <class InputIt>
    void make_heap(InputIt first, InputIt last, size_type root, int idx);

    size_type parent(size_type index) const;
    size_type right_child(size_type index) const;
    size_type left_child(size_type index) const;

    void percolate_down(size_type pivot, int idx);
    void percolate_up(size_type pivot, int idx);

    bool is_internal(size_type index) const;
    bool is_external(size_type index) const;

    
};

template <class T, class Compare>
void swap(multi_index_intrusive_heap<T, Compare>& lhs, multi_index_intrusive_heap<T, Compare>& rhs);

} // namespace sbpl

#include "detail/multi_index_intrusive_heap.hpp"

#endif
