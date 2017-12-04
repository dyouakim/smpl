#ifndef SMPL_CONSOLE_NONSTD_H
#define SMPL_CONSOLE_NONSTD_H

#include <ostream>
#include <vector>

namespace std {

template <class T, class Allocator>
ostream& operator<<(ostream& o, const vector<T, Allocator>& v)
{
    o << "[ ";
    for (size_t i = 0; i < v.size(); ++i) {
        o << v[i];
        if (i != v.size() - 1) {
            o << ", ";
        }
    }
    o << " ]";
    return o;
}

} // namespace std

#endif
