#include "reservation_table.hpp"

ReservationTable::ReservationTable(const Graph &graph, const std::vector<Mobile> &mobiles)
    : graph(graph)
    , mobiles(mobiles)
    , rtree(nullptr)
{
}

ReservationTable::~ReservationTable()
{
    this->paths.clear();
    this->polygons.clear();

    if (this->rtree)
    {
        this->rtree->clear();
    }
}

void ReservationTable::addZoneCapacityConstraint(const std::vector<int> &weights, int capacity, const Polygon &polygon)
{
    this->constraints.push_back(ZoneCapacityConstraint(this->graph, this->mobiles, weights, capacity, polygon));
}

void ReservationTable::update(const std::unordered_map<int, Path> &paths)
{
    this->paths = paths;
    this->polygons.clear();
    std::vector<Value> values;
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
            values.push_back({box, id});
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

    this->rtree = std::make_unique<RTree>(values);
}

std::vector<Interval> ReservationTable::getSafeIntervals(int mobile, int vertex)
{
    TimedPosition start{vertex, 0}, end{vertex, std::numeric_limits<double>::max()};
    auto polygons = this->getBoundingPolygons(mobile, start, end);
    Box box;
    bg::envelope(std::get<0>(polygons), box);

    std::vector<Interval> collisionIntervals;
    this->rtree->query(bgi::intersects(box), boost::make_function_output_iterator([&](auto const& value){
        auto id = value.second;
        auto otherMobile = id % this->mobiles.size();
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
    auto polygons = this->getBoundingPolygons(mobile, start, end);
    Box box;
    bg::envelope(std::get<0>(polygons), box);

    std::vector<Interval> collisionIntervals;
    this->rtree->query(bgi::intersects(box), boost::make_function_output_iterator([&](auto const& value){
        auto id = value.second;
        auto otherMobile = id % this->mobiles.size();
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

    for (auto &constraint : this->constraints)
    {
        constraint.addViolationIntervals(mobile, from, to, polygons, collisionIntervals);
    }

    std::sort(collisionIntervals.begin(), collisionIntervals.end());

    for (int i = collisionIntervals.size() - 2; i >= 0; i--)
    {
        while (i + 1 < collisionIntervals.size() && collisionIntervals[i].end >= collisionIntervals[i+1].start)
        {
            collisionIntervals[i].end = std::max(collisionIntervals[i].end, collisionIntervals[i+1].end);
            collisionIntervals.erase(collisionIntervals.begin() + i + 1);
        }
    }

    return collisionIntervals;
}

std::tuple<Polygon, Polygon, Polygon> ReservationTable::getBoundingPolygons(int mobile, const TimedPosition &start, const TimedPosition &end)
{
    if (start.vertex == end.vertex)
    {
        auto position = this->graph.getPosition(start.vertex);

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
        auto startPosition = this->graph.getPosition(start.vertex), endPosition = this->graph.getPosition(end.vertex);
        auto moveAngle = std::atan2(endPosition.y - startPosition.y, endPosition.x - startPosition.x);

        auto a = this->mobiles[mobile].length * std::cos(moveAngle) / 2;
        auto b = this->mobiles[mobile].width * std::sin(moveAngle) / 2;
        auto c = this->mobiles[mobile].length * std::sin(moveAngle) / 2;
        auto d = this->mobiles[mobile].width * std::cos(moveAngle) / 2;

        Polygon polygon, startPolygon, endPolygon;

        Point startRearRight(startPosition.x - a + b, startPosition.y - c - d);
        Point startRearLeft(startPosition.x - a - b, startPosition.y - c + d);
        Point startFrontRight(startPosition.x + a + b, startPosition.y + c - d);
        Point startFrontLeft(startPosition.x + a - b, startPosition.y + c + d);
        Point endRearRight(endPosition.x - a + b, endPosition.y - c - d);
        Point endRearLeft(endPosition.x - a - b, endPosition.y - c + d);
        Point endFrontRight(endPosition.x + a + b, endPosition.y + c - d);
        Point endFrontLeft(endPosition.x + a - b, endPosition.y + c + d);

        // construct polygons with vertices in CCW order
        bg::append(polygon.outer(), startRearRight);
        bg::append(polygon.outer(), startRearLeft);
        bg::append(polygon.outer(), endFrontLeft);
        bg::append(polygon.outer(), endFrontRight);
        bg::append(polygon.outer(), startRearRight);
        
        bg::append(startPolygon.outer(), startRearRight);
        bg::append(startPolygon.outer(), startRearLeft);
        bg::append(startPolygon.outer(), startFrontLeft);
        bg::append(startPolygon.outer(), startFrontRight);
        bg::append(startPolygon.outer(), startRearRight);
        
        bg::append(endPolygon.outer(), endRearRight);
        bg::append(endPolygon.outer(), endRearLeft);
        bg::append(endPolygon.outer(), endFrontLeft);
        bg::append(endPolygon.outer(), endFrontRight);
        bg::append(endPolygon.outer(), endRearRight);

        return {polygon, startPolygon, endPolygon};
    }
}

// get the interval for which starting to cross the edge will cause a collision with the given move
Interval ReservationTable::getCollisionInterval(int mobile, int from, int to, const std::tuple<Polygon, Polygon, Polygon> &edgePolygons,
    const std::tuple<Polygon, Polygon, Polygon> &movePolygons, const TimedPosition &moveStart, const TimedPosition &moveEnd)
{
    std::deque<Polygon> collisionPolygons;
    bg::intersection(std::get<0>(edgePolygons), std::get<0>(movePolygons), collisionPolygons);

    if (collisionPolygons.empty())
    {
        return {std::numeric_limits<double>::max(), 0};
    }
    
    auto &collisionPolygon = collisionPolygons[0];

    auto moveStartPosition = this->graph.getPosition(moveStart.vertex);
    auto moveEndPosition = this->graph.getPosition(moveEnd.vertex);
    auto moveDistance = this->graph.manhattanDistance(moveStart.vertex, moveEnd.vertex);
    auto moveDistanceBeforeCollision = moveDistance > 0 ? bg::distance(std::get<1>(movePolygons), collisionPolygon) : 0;
    auto moveDistanceAfterCollision = moveDistance > 0 ? bg::distance(collisionPolygon, std::get<2>(movePolygons)) : 0;
    auto moveTimeBeforeCollision = moveDistance > 0 ? (moveEnd.time - moveStart.time) * moveDistanceBeforeCollision / moveDistance : 0;
    auto moveTimeAfterCollision = moveDistance > 0 ? (moveEnd.time - moveStart.time) * moveDistanceAfterCollision / moveDistance : 0;
    auto moveAngle = moveDistance > 0 ? std::atan2(moveEndPosition.y - moveStartPosition.y, moveEndPosition.x - moveStartPosition.x) : 0;

    auto edgeStartPosition = this->graph.getPosition(from);
    auto egdeEndPosition = this->graph.getPosition(to);
    auto edgeDistance = this->graph.manhattanDistance(from, to);
    auto edgeDistanceBeforeCollision = edgeDistance > 0 ? bg::distance(std::get<1>(edgePolygons), collisionPolygon) : 0;
    auto edgeDistanceAfterCollision = edgeDistance > 0 ? bg::distance(collisionPolygon, std::get<2>(edgePolygons)) : 0;
    auto edgeCost = edgeDistance > 0 ? this->graph.getCost(from, to, this->mobiles[mobile]) : 0;
    auto edgeTimeBeforeCollision = edgeDistance > 0 ? edgeCost * edgeDistanceBeforeCollision / edgeDistance : 0;
    auto edgeTimeAfterCollision = edgeDistance > 0 ? edgeCost * edgeDistanceAfterCollision / edgeDistance : 0;
    auto edgeAngle = edgeDistance > 0 ? std::atan2(egdeEndPosition.y - edgeStartPosition.y, egdeEndPosition.x - edgeStartPosition.x) : 0;
    
    Interval result;
    if (moveDistance > 0 && edgeDistance > 0 && std::abs(moveAngle - edgeAngle) < 1e-3) // travelling in same direction
    {
        result.start = std::min(
            moveStart.time + moveTimeBeforeCollision - edgeTimeBeforeCollision, 
            moveEnd.time - moveTimeAfterCollision - (edgeCost - edgeTimeAfterCollision)
        );
        result.end = std::max(
            moveStart.time + moveTimeBeforeCollision - edgeTimeBeforeCollision, 
            moveEnd.time - moveTimeAfterCollision - (edgeCost - edgeTimeAfterCollision)
        );
    }
    else
    {
        result.start = moveStart.time + moveTimeBeforeCollision - (edgeCost - edgeTimeAfterCollision);
        result.end = moveEnd.time - moveTimeAfterCollision - edgeTimeBeforeCollision;
    }

    return result;
}
