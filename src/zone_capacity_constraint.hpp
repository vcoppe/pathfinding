#ifndef ZONE_CAPACITY_CONSTRAINT_HPP
#define ZONE_CAPACITY_CONSTRAINT_HPP

#include <unordered_map>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include "graph.hpp"
#include "structs.hpp"
#include "utils.hpp"

class ZoneCapacityConstraint
{
private:
    std::shared_ptr<Graph> graph;
    const std::vector<Mobile> mobiles;
    const std::vector<int> &weights;
    const int capacity, z;
    std::vector<std::pair<double, int> > consumptions;
    std::vector<Interval> fullCapacityIntervals;
public:
    ZoneCapacityConstraint(std::shared_ptr<Graph> graph, const std::vector<Mobile> &mobiles, const std::vector<int> &weights, int capacity, const Polygon &polygon, double z);
    ~ZoneCapacityConstraint();

    const Polygon polygon;

    void reset();
    void add(int mobile, const TimedPosition &start, const TimedPosition &end, const std::tuple<Polygon, Polygon, Polygon> &polygons);
    void build();
    void addViolationIntervals(int mobile, int from, int to, const std::tuple<Polygon, Polygon, Polygon> &polygons, std::vector<Interval> &collisionIntervals);
};

#endif // ZONE_CAPACITY_CONSTRAINT_HPP