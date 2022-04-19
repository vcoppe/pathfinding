#include "zone_capacity_constraint.hpp"

ZoneCapacityConstraint::ZoneCapacityConstraint(const Graph &graph, const std::vector<Mobile> &mobiles, const std::vector<int> &weights, int capacity, const Polygon &polygon)
    : graph(graph)
    , mobiles(mobiles)
    , weights(weights)
    , capacity(capacity)
    , polygon(polygon)
{
}

ZoneCapacityConstraint::~ZoneCapacityConstraint()
{
}

void ZoneCapacityConstraint::reset()
{
    this->consumptions.clear();
}

void ZoneCapacityConstraint::add(int mobile, const TimedPosition &start, const TimedPosition &end, const std::tuple<Polygon, Polygon, Polygon> &polygons)
{
    std::deque<Polygon> collisionPolygons;
    bg::intersection(this->polygon, std::get<0>(polygons), collisionPolygons);

    if (collisionPolygons.empty())
    {
        return;
    }

    auto &collisionPolygon = collisionPolygons[0];

    auto distance = this->graph.distance(start.vertex, end.vertex);
    auto distanceBeforeZone = distance > 0 ? bg::distance(std::get<1>(polygons), collisionPolygon) : 0;
    auto distanceAfterZone = distance > 0 ? bg::distance(collisionPolygon, std::get<2>(polygons)) : 0;
    auto timeBeforeZone = distance > 0 ? (end.time - start.time) * distanceBeforeZone / distance : 0;
    auto timeAfterZone = distance > 0 ? (end.time - start.time) * distanceAfterZone / distance : 0;

    this->consumptions.push_back({start.time + timeBeforeZone, this->weights[mobile]});
    this->consumptions.push_back({end.time - timeAfterZone, - this->weights[mobile]});

}

void ZoneCapacityConstraint::build()
{
    std::sort(this->consumptions.begin(), this->consumptions.end());

    double current = 0;
    int consumption = 0;
    for (const auto &pair : this->consumptions)
    {
        if (pair.first > current)
        {
            if (consumption >= this->capacity)
            {
                this->fullCapacityIntervals.push_back({current, pair.first});
            }
            current = pair.first;
        }

        consumption += pair.second;
    }

    mergeIntervals(this->fullCapacityIntervals);
}

void ZoneCapacityConstraint::addViolationIntervals(int mobile, int from, int to, const std::tuple<Polygon, Polygon, Polygon> &polygons, std::vector<Interval> &collisionIntervals)
{
    std::deque<Polygon> collisionPolygons;
    bg::intersection(this->polygon, std::get<0>(polygons), collisionPolygons);

    if (collisionPolygons.empty())
    {
        return;
    }

    auto &collisionPolygon = collisionPolygons[0];

    auto distance = this->graph.distance(from, to);
    auto distanceBeforeZone = distance > 0 ? bg::distance(std::get<1>(polygons), collisionPolygon) : 0;
    auto cost = distance > 0 ? this->graph.getCost(from, to, this->mobiles[mobile]) : 0;
    auto timeBeforeZone = distance > 0 ? cost * distanceBeforeZone / distance : 0;

    for (const auto &interval : this->fullCapacityIntervals)
    {
        collisionIntervals.push_back({interval.start - timeBeforeZone, interval.end - timeBeforeZone});
    }
}