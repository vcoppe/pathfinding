#ifndef RESERVATION_TABLE_HPP
#define RESERVATION_TABLE_HPP

#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "graph.hpp"
#include "structs.hpp"
#include "utils.hpp"
#include "zone_capacity_constraint.hpp"

class ReservationTable
{
private:
    Graph graph;
    const std::vector<Mobile> mobiles;
    std::vector<ZoneCapacityConstraint> constraints;
    std::unordered_map<int, Path> paths;
    std::unordered_map<int, std::tuple<Polygon, Polygon, Polygon> > polygons;
    std::unique_ptr<RTree> rtree;

    std::tuple<Polygon, Polygon, Polygon> getBoundingPolygons(int mobile, const TimedPosition &start, const TimedPosition &end);
    Interval getCollisionInterval(int mobile, int from, int to, const std::tuple<Polygon, Polygon, Polygon> &edgePolygons,
        const std::tuple<Polygon, Polygon, Polygon> &movePolygons, const TimedPosition &moveStart, const TimedPosition &moveEnd);
public:
    ReservationTable(const Graph &graph, const std::vector<Mobile> &mobiles);
    ~ReservationTable();

    void addZoneCapacityConstraint(const std::vector<int> &weights, int capacity, const Polygon &polygon);
    void update(const std::unordered_map<int, Path> &paths);
    std::vector<Interval> getSafeIntervals(int mobile, int vertex);
    std::vector<Interval> getCollisionIntervals(int mobile, int from, int to);
};

#endif // RESERVATION_TABLE_HPP
