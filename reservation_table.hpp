#ifndef RESERVATION_TABLE_HPP
#define RESERVATION_TABLE_HPP

#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xyz.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/iterator/function_output_iterator.hpp>

#include "graph.hpp"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::d2::point_xy<double> Point;
typedef bg::model::box<Point> Box;
typedef bg::model::polygon<Point> Polygon;
typedef std::pair<Box, int> Value;
typedef bgi::rtree<Value, bgi::rstar<3>> RTree;

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

class ReservationTable
{
private:
    Graph graph;
    const std::vector<Mobile> mobiles;
    std::unordered_map<int, Path> paths;
    std::unordered_map<int, std::tuple<Polygon, Polygon, Polygon> > polygons;
    std::unique_ptr<RTree> rtree;

    std::tuple<Polygon,Polygon,Polygon> getBoundingPolygons(int mobile, const TimedPosition &start, const TimedPosition &end);
    Interval getCollisionInterval(int mobile, int from, int to, const std::tuple<Polygon,Polygon,Polygon> &edgePolygons,
        const std::tuple<Polygon,Polygon,Polygon> &movePolygons, const TimedPosition &moveStart, const TimedPosition &moveEnd);
public:
    ReservationTable(const Graph &graph, const std::vector<Mobile> &mobiles);
    ~ReservationTable();

    void update(const std::unordered_map<int, Path> &paths);
    std::vector<Interval> getSafeIntervals(int mobile, int vertex);
    std::vector<Interval> getCollisionIntervals(int mobile, int from, int to);
};

#endif // RESERVATION_TABLE_HPP
