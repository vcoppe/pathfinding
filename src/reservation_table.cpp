#include <cmath>
#include <limits>

#include "reservation_table.hpp"

ReservationTable::ReservationTable(std::shared_ptr<Graph> graph, const std::vector<Mobile> &mobiles)
    : graph(graph)
    , mobiles(mobiles)
{
}

ReservationTable::~ReservationTable()
{
    this->paths.clear();
    this->polygons.clear();
    this->rtrees.clear();
}

void ReservationTable::addZoneCapacityConstraint(const std::vector<int> &weights, int capacity, const Polygon &polygon, double z)
{
    this->constraints.push_back(ZoneCapacityConstraint(this->graph, this->mobiles, weights, capacity, polygon, z));
}

void ReservationTable::update(const std::unordered_map<int, Path> &paths)
{
    this->paths = paths;
    this->values.clear();
    this->polygons.clear();
    Box box;

    for (auto &constraint : this->constraints)
    {
        constraint.reset();
    }

    for (const auto &pair : paths) {
        const auto mobile = pair.first;
        const auto &path = pair.second;
        for (int i = 0; i + 1 < path.timedPositions.size(); ++i)
        {
            auto polygons = this->getBoundingPolygons(mobile, path.timedPositions[i], path.timedPositions[i+1]);
            bg::envelope(std::get<0>(polygons), box);
            auto id = mobile + this->mobiles.size() * i;

            auto startZ = this->graph->getZ(path.timedPositions[i].vertex), endZ = this->graph->getZ(path.timedPositions[i+1].vertex);
            if (!this->values.contains(startZ))
            {
                this->values[startZ] = std::vector<Value>();
            }
            if (!this->values.contains(endZ))
            {
                this->values[endZ] = std::vector<Value>();
            }
            this->values[startZ].push_back({box, id});
            if (startZ != endZ)
            {
                this->values[endZ].push_back({box, id});
            }
            this->polygons[id] = polygons;

            for (auto &constraint : this->constraints)
            {
                constraint.add(mobile, path.timedPositions[i], path.timedPositions[i+1], polygons);
            }
        }
    }

    for (auto &constraint : this->constraints)
    {
        constraint.build();
    }

    for (const auto &pair : this->values)
    {
        this->rtrees[pair.first] = RTree(pair.second);
    }
}

std::vector<Interval> ReservationTable::getSafeIntervals(int mobile, int vertex)
{
    TimedPosition start{vertex, 0}, end{vertex, std::numeric_limits<double>::max()};
    auto z = this->graph->getZ(vertex);
    auto polygons = this->getBoundingPolygons(mobile, start, end);
    Box box;
    bg::envelope(std::get<0>(polygons), box);

    std::vector<Interval> collisionIntervals;
    if (this->rtrees.contains(z))
    {
        this->rtrees[z].query(bgi::intersects(box), boost::make_function_output_iterator([&](auto const& value){
            auto id = value.second;
            auto otherMobile = id % this->mobiles.size();
            if (mobile == otherMobile)
            {
                return;
            }
            auto i = id / this->mobiles.size();
            auto collisionInterval = this->getCollisionInterval(
                mobile, vertex, vertex, polygons,
                this->polygons[id], this->paths[otherMobile].timedPositions[i], this->paths[otherMobile].timedPositions[i+1]
            );
            if (collisionInterval.start < collisionInterval.end)
            {
                collisionIntervals.push_back(collisionInterval);
            }
        }));
    }

    for (auto &constraint : this->constraints)
    {
        constraint.addViolationIntervals(mobile, vertex, vertex, polygons, collisionIntervals);
    }

    std::sort(collisionIntervals.begin(), collisionIntervals.end());

    std::vector<Interval> safeIntervals;
    double current = 0;
    for (const auto &interval : collisionIntervals)
    {
        if (current < interval.start) {
            safeIntervals.push_back({current, interval.start});
        }
        current = interval.end;
    }

    if (current < std::numeric_limits<double>::max())
    {
        safeIntervals.push_back({current, std::numeric_limits<double>::max()});
    }

    return safeIntervals;
}

std::vector<Interval> ReservationTable::getCollisionIntervals(int mobile, int from, int to)
{
    TimedPosition start{from, 0}, end{to, std::numeric_limits<double>::max()};
    auto startZ = this->graph->getZ(from), endZ = this->graph->getZ(to);
    std::vector<double> zs = {startZ};
    if (startZ != endZ)
    {
        zs.push_back(endZ);
    }
    auto polygons = this->getBoundingPolygons(mobile, start, end);
    Box box;
    bg::envelope(std::get<0>(polygons), box);

    std::vector<Interval> collisionIntervals;
    for (auto z : zs)
    {
        if (this->rtrees.contains(z))
        {
            this->rtrees[z].query(bgi::intersects(box), boost::make_function_output_iterator([&](auto const& value){
                auto id = value.second;
                auto otherMobile = id % this->mobiles.size();
                if (mobile == otherMobile)
                {
                    return;
                }
                auto i = id / this->mobiles.size();
                auto collisionInterval = this->getCollisionInterval(
                    mobile, from, to, polygons,
                    this->polygons[id], this->paths[otherMobile].timedPositions[i], this->paths[otherMobile].timedPositions[i+1]
                );
                if (collisionInterval.start < collisionInterval.end)
                {
                    collisionIntervals.push_back(collisionInterval);
                }
            }));
        }
    }

    for (auto &constraint : this->constraints)
    {
        constraint.addViolationIntervals(mobile, from, to, polygons, collisionIntervals);
    }

    std::sort(collisionIntervals.begin(), collisionIntervals.end());

    mergeIntervals(collisionIntervals);

    return collisionIntervals;
}

std::tuple<Polygon, Polygon, Polygon> ReservationTable::getBoundingPolygons(int mobile, const TimedPosition &start, const TimedPosition &end)
{
    if (start.vertex == end.vertex)
    {
        auto position = this->graph->getPosition(start.vertex);

        Point point(position.x, position.y);

        auto distance = std::sqrt(this->mobiles[mobile].length * this->mobiles[mobile].length / 4 + this->mobiles[mobile].width * this->mobiles[mobile].width / 4);
        auto nPoints = 10;
        bg::strategy::buffer::distance_symmetric<double> distanceStrategy(distance);
        bg::strategy::buffer::join_round joinStrategy(nPoints);
        bg::strategy::buffer::end_round endStrategy(nPoints);
        bg::strategy::buffer::point_circle circleStrategy(nPoints);
        bg::strategy::buffer::side_straight sideStrategy;

        bg::model::multi_polygon<Polygon> circle;
        bg::buffer(point, circle, distanceStrategy, sideStrategy, joinStrategy, endStrategy, circleStrategy);

        return {circle[0], circle[0], circle[0]};
    }
    else
    {
        auto startPosition = this->graph->getPosition(start.vertex), endPosition = this->graph->getPosition(end.vertex);
        auto moveAngle = std::atan2(endPosition.y - startPosition.y, endPosition.x - startPosition.x);

        auto a = this->mobiles[mobile].length * std::cos(moveAngle) / 2;
        auto b = this->mobiles[mobile].width * std::sin(moveAngle) / 2;
        auto c = this->mobiles[mobile].length * std::sin(moveAngle) / 2;
        auto d = this->mobiles[mobile].width * std::cos(moveAngle) / 2;

        Point startRearRight(startPosition.x - a + b, startPosition.y - c - d);
        Point startRearLeft(startPosition.x - a - b, startPosition.y - c + d);
        Point startFrontRight(startPosition.x + a + b, startPosition.y + c - d);
        Point startFrontLeft(startPosition.x + a - b, startPosition.y + c + d);
        Point endRearRight(endPosition.x - a + b, endPosition.y - c - d);
        Point endRearLeft(endPosition.x - a - b, endPosition.y - c + d);
        Point endFrontRight(endPosition.x + a + b, endPosition.y + c - d);
        Point endFrontLeft(endPosition.x + a - b, endPosition.y + c + d);

        auto polygon = getPolygon({startRearRight, startRearLeft, endFrontLeft, endFrontRight});
        auto startPolygon = getPolygon({startRearRight, startRearLeft, startFrontLeft, startFrontRight});
        auto endPolygon = getPolygon({endRearRight, endRearLeft, endFrontLeft, endFrontRight});

        return {polygon, startPolygon, endPolygon};
    }
}

// get the interval for which starting to cross the edge will cause a collision with the given move
Interval ReservationTable::getCollisionInterval(int mobile, int from, int to, const std::tuple<Polygon, Polygon, Polygon> &edgePolygons,
    const std::tuple<Polygon, Polygon, Polygon> &movePolygons, const TimedPosition &moveStart, const TimedPosition &moveEnd)
{
    Interval result{std::numeric_limits<double>::max(), 0};
    std::deque<Polygon> collisionPolygons, startCollisionPolygons, afterStartCollisionPolygons, endCollisionPolygons, beforeEndCollisionPolygons;
    bg::intersection(std::get<0>(edgePolygons), std::get<0>(movePolygons), collisionPolygons);

    if (collisionPolygons.empty())
    {
        return result;
    }
    
    auto &collisionPolygon = collisionPolygons[0];

    auto moveStartPosition = this->graph->getPosition(moveStart.vertex);
    auto moveEndPosition = this->graph->getPosition(moveEnd.vertex);
    auto moveDistance = this->graph->distance(moveStart.vertex, moveEnd.vertex);
    auto moveDistanceBeforeCollision = moveDistance > 0 ? bg::distance(std::get<1>(movePolygons), collisionPolygon) : 0;
    auto moveDistanceAfterCollision = moveDistance > 0 ? bg::distance(collisionPolygon, std::get<2>(movePolygons)) : 0;
    auto moveTimeBeforeCollision = moveDistance > 0 ? (moveEnd.time - moveStart.time) * moveDistanceBeforeCollision / moveDistance : 0;
    auto moveTimeAfterCollision = moveDistance > 0 ? (moveEnd.time - moveStart.time) * moveDistanceAfterCollision / moveDistance : 0;
    auto moveAngle = moveDistance > 0 ? std::atan2(moveEndPosition.y - moveStartPosition.y, moveEndPosition.x - moveStartPosition.x) : 0;

    auto edgeStartPosition = this->graph->getPosition(from);
    auto egdeEndPosition = this->graph->getPosition(to);
    auto edgeDistance = this->graph->distance(from, to);
    auto edgeDistanceBeforeCollision = edgeDistance > 0 ? bg::distance(std::get<1>(edgePolygons), collisionPolygon) : 0;
    auto edgeDistanceAfterCollision = edgeDistance > 0 ? bg::distance(collisionPolygon, std::get<2>(edgePolygons)) : 0;
    auto edgeCost = edgeDistance > 0 ? this->graph->getCost(from, to, this->mobiles[mobile]) : 0;
    auto edgeTimeBeforeCollision = edgeDistance > 0 ? edgeCost * edgeDistanceBeforeCollision / edgeDistance : 0;
    auto edgeTimeAfterCollision = edgeDistance > 0 ? edgeCost * edgeDistanceAfterCollision / edgeDistance : 0;
    auto edgeAngle = edgeDistance > 0 ? std::atan2(egdeEndPosition.y - edgeStartPosition.y, egdeEndPosition.x - edgeStartPosition.x) : 0;
    
    if (moveDistance > 0 && edgeDistance > 0 && std::abs(moveAngle - edgeAngle) < 1e-3) // travelling in same direction
    {
        bg::intersection(std::get<1>(edgePolygons), collisionPolygon, startCollisionPolygons);
        if (startCollisionPolygons.empty())
        {
            bg::difference(collisionPolygon, std::get<1>(movePolygons), afterStartCollisionPolygons);
            if (afterStartCollisionPolygons.empty())
            {
                result.start = std::min(result.start, moveStart.time - edgeCost);
            }
            else
            {
                auto &afterStartCollisionPolygon = afterStartCollisionPolygons[0];
                auto edgeDistanceBeforeAfterStartCollision = bg::distance(std::get<1>(edgePolygons), afterStartCollisionPolygon);
                auto edgeTimeBeforeAfterStartCollision = edgeCost * edgeDistanceBeforeAfterStartCollision / edgeDistance;
                result.start = std::min(result.start, moveStart.time - edgeTimeBeforeAfterStartCollision);
            }
            result.end = std::max(result.end, moveStart.time - edgeTimeBeforeCollision);
        }
        else
        {
            bg::difference(collisionPolygon, std::get<1>(edgePolygons), afterStartCollisionPolygons);
            if (afterStartCollisionPolygons.empty())
            {
                result.end = std::max(result.end, moveEnd.time);
            }
            else
            {
                auto &afterStartCollisionPolygon = afterStartCollisionPolygons[0];
                auto moveDistanceBeforeAfterStartCollision = bg::distance(std::get<1>(movePolygons), afterStartCollisionPolygon);
                auto moveTimeBeforeAfterStartCollision = (moveEnd.time - moveStart.time) * moveDistanceBeforeAfterStartCollision / moveDistance;
                result.end = std::max(result.end, moveStart.time + moveTimeBeforeAfterStartCollision);
            }

            result.start = std::min(result.start, moveStart.time + moveTimeBeforeCollision);
        }

        bg::intersection(std::get<2>(edgePolygons), collisionPolygon, endCollisionPolygons);
        if (endCollisionPolygons.empty())
        {
            bg::difference(collisionPolygon, std::get<2>(movePolygons), beforeEndCollisionPolygons);
            if (beforeEndCollisionPolygons.empty())
            {
                result.end = std::max(result.end, moveEnd.time);
            }
            else
            {
                auto &beforeEndCollisionPolygon = beforeEndCollisionPolygons[0];
                auto edgeDistanceAfterBeforeEndCollision = bg::distance(beforeEndCollisionPolygon, std::get<2>(edgePolygons));
                auto edgeTimeAfterBeforeEndCollision = edgeCost * edgeDistanceAfterBeforeEndCollision / edgeDistance;
                result.end = std::max(result.end, moveEnd.time - (edgeCost - edgeTimeAfterBeforeEndCollision));
            }

            result.start = std::min(result.start, moveEnd.time - (edgeCost - edgeTimeAfterCollision));
        }
        else
        {
            bg::difference(collisionPolygon, std::get<2>(edgePolygons), beforeEndCollisionPolygons);
            if (beforeEndCollisionPolygons.empty())
            {
                result.start = std::min(result.start, moveEnd.time - edgeCost);
            }
            else
            {
                auto &beforeEndCollisionPolygon = beforeEndCollisionPolygons[0];
                auto moveDistanceAfterBeforeEndCollision = bg::distance(beforeEndCollisionPolygon, std::get<2>(movePolygons));
                auto moveTimeAfterBeforeEndCollision = (moveEnd.time - moveStart.time) * moveDistanceAfterBeforeEndCollision / moveDistance;
                result.start = std::min(result.start, moveEnd.time - moveTimeAfterBeforeEndCollision);
            }

            result.end = std::max(result.end, moveEnd.time - moveTimeAfterCollision);
        }
    }
    else
    {
        result.start = moveStart.time + moveTimeBeforeCollision - (edgeCost - edgeTimeAfterCollision);
        result.end = moveEnd.time - moveTimeAfterCollision - edgeTimeBeforeCollision;
    }

    return result;
}
