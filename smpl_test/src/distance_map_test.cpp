#include <iomanip>
#include <iostream>
#include <ostream>
#include <utility>

#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/distance_map/sparse_distance_map.h>

/*
template <class T>
auto operator<<(std::ostream& o, const sbpl::DistanceMap<T>& d) -> std::ostream&
{
    for (int z = 0; z < d.numCellsZ(); ++z) {
    for (int y = d.numCellsY() - 1; y >= 0; --y) {
    for (int x = 0; x < d.numCellsX(); ++x) {
        double dist = d.getDistance(x, y, z);
        o << std::setprecision(1) << std::setw(3) << dist;
        if (x != d.numCellsX() - 1) {
            o << ' ';
        } else {
            o << '\n';
        }
    }
    }
    o << '\n';
    }
    return o;
}
*/

auto operator<<(std::ostream& o, const sbpl::DistanceMapInterface& d)
    -> std::ostream&
{
    for (int z = 0; z < d.numCellsZ(); ++z) {
    for (int y = d.numCellsY() - 1; y >= 0; --y) {
    for (int x = 0; x < d.numCellsX(); ++x) {
        double dist = d.getCellDistance(x, y, z);
        o << std::setprecision(1) << std::setw(3) << dist;
        if (x != d.numCellsX() - 1) {
            o << ' ';
        } else {
            o << '\n';
        }
    }
    }
    o << '\n';
    }
    return o;
}

bool operator==(
    const sbpl::DistanceMapInterface& d1,
    const sbpl::DistanceMapInterface& d2)
{
    if (std::make_tuple(d1.numCellsX(), d1.numCellsY(), d1.numCellsZ()) !=
        std::make_tuple(d2.numCellsY(), d2.numCellsY(), d2.numCellsZ()))
    {
        return false;
    }

    for (int x = 0; x < d1.numCellsX(); ++x) {
    for (int y = 0; y < d1.numCellsY(); ++y) {
    for (int z = 0; z < d1.numCellsZ(); ++z) {
        double dist1 = d1.getCellDistance(x, y, z);
        double dist2 = d2.getCellDistance(x, y, z);
        if (dist1 != dist2) {
            return false;
        }
    }
    }
    }

    return true;
}

bool operator!=(
    const sbpl::DistanceMapInterface& d1,
    const sbpl::DistanceMapInterface& d2)
{
    return !(d1 == d2);
}

template <class DistanceMap>
void TestSpecialMemberFunctions()
{
    const double origin_x = 0.0;
    const double origin_y = 0.0;
    const double origin_z = 0.0;
    const double size_x = 10.0;
    const double size_y = 10.0;
    const double size_z = 10.0;
    const double res = 0.5;
    const double max_dist = 2.0;

    // constructor
    DistanceMap d1(
            origin_x, origin_y, origin_z,
            size_x, size_y, size_z,
            res, max_dist);

    // copy constructor
    DistanceMap d2(d1);

    // copy assignment
    DistanceMap d3(
            origin_x, origin_y, origin_z,
            size_x, size_y, size_z,
            res, max_dist);
    d3 = d1;

    // move constructor
    DistanceMap d4(std::move(d1));

    // move assignment
    DistanceMap d5(
            origin_x, origin_y, origin_z,
            size_x, size_y, size_z,
            res, max_dist);

    d5 = std::move(d2);

    // d1 and d2 are moved from
    // d3 is a copy of a constructed distance map
    // d4 owns d1's data
    // d5 owns d2's data

    std::cout << "Start:\n" << d4;

    std::vector<Eigen::Vector3d> points;
    std::default_random_engine rng;
    std::uniform_real_distribution<double> dist(0.0, 10.0);
    for (int i = 0; i < 10; ++i) {
        points.emplace_back(dist(rng), dist(rng), dist(rng));
    }

    d4.addPointsToMap(points);
    d5.addPointsToMap(points);

    std::cout << "Post-Insertion:\n" << d4;

    d5.removePointsFromMap(points);

    points.resize(points.size() >> 1);
    d4.removePointsFromMap(points);

    std::cout << "Post-Removal:\n" << d4;

    if (d5 != d3) {
        printf("Distance maps are not equal\n");
    }
    if (d4 == d3) {
        printf("Distance maps should not be equal!\n");
    }
}

int main(int argc, char* argv[])
{
    TestSpecialMemberFunctions<sbpl::SparseDistanceMap>();
//    TestSpecialMemberFunctions<sbpl::EuclidDistanceMap>();
    return 0;
}
