#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/iterator/function_output_iterator.hpp>
#include <boost/geometry/index/rtree.hpp>

enum MobileType
{
    Forklift,
    AGV
};

struct Mobile
{
    MobileType type;
    double length, width, maxSpeed;
};

struct Interval
{
    double start, end;

    bool operator<(const Interval &other) const
    {
        if (start == other.start)
        {
            return end < other.end;
        }
        return start < other.start;
    }
};

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::d2::point_xy<double> Point;
typedef bg::model::box<Point> Box;
typedef bg::model::polygon<Point> Polygon;
typedef std::pair<Box, int> Value;
typedef bgi::rtree<Value, bgi::rstar<3>> RTree;

#endif // STRUCTS_HPP