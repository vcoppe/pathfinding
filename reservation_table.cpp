#include <cmath>

#include "reservation_table.hpp"

ReservationTable::ReservationTable(const Graph &graph, const std::vector<Mobile> &mobiles)
    : graph(graph)
    , mobiles(mobiles)
    , rtree(nullptr)
{
}

ReservationTable::~ReservationTable()
{
}

void ReservationTable::update(const std::unordered_map<int, Path> &paths)
{
    this->paths = paths;
    this->movePolygons.clear();
    this->startPolygons.clear();
    this->endPolygons.clear();
    std::vector<Value> values;
    Box box;

    for (const auto &pair : paths) {
        const auto mobile = pair.first;
        const auto &path = pair.second;
        for (int i = 0; i < path.timedPositions.size() - 1; ++i)
        {
            auto startPolygon = std::make_shared<Polygon>(), endPolygon = std::make_shared<Polygon>();
            auto polygon = this->getBoundingPolygon(mobile, path.timedPositions[i], path.timedPositions[i+1], startPolygon, endPolygon);
            bg::envelope(polygon, box);
            auto id = mobile + this->mobiles.size() * i;
            values.push_back({box, id});
            this->movePolygons[id] = polygon;
            this->startPolygons[id] = *startPolygon;
            this->endPolygons[id] = *endPolygon;
        }
    }

    this->rtree = std::make_unique<RTree>(values);
}

std::vector<Interval> ReservationTable::getSafeIntervals(int mobile, int vertex)
{
    TimedPosition start{vertex, 0}, end{vertex, std::numeric_limits<double>::max()};
    auto polygon = this->getBoundingPolygon(mobile, start, end);
    Box box;
    bg::envelope(polygon, box);

    std::vector<Interval> collisionIntervals;
    this->rtree->query(bgi::intersects(box), boost::make_function_output_iterator([&](auto const& value){
        auto id = value.second;
        auto otherMobile = id % this->mobiles.size();
        auto i = id / this->mobiles.size();
        auto collisionInterval = this->getCollisionInterval(
            polygon, this->movePolygons[id], this->startPolygons[id], this->endPolygons[id],
            this->paths[otherMobile].timedPositions[i], this->paths[otherMobile].timedPositions[i+1]
        );
        if (collisionInterval.start < collisionInterval.end)
        {
            collisionIntervals.push_back(collisionInterval);
        }
    }));
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
    auto polygon = this->getBoundingPolygon(mobile, start, end);
    Box box;
    bg::envelope(polygon, box);

    std::vector<Interval> collisionIntervals;
    this->rtree->query(bgi::intersects(box), boost::make_function_output_iterator([&](auto const& value){
        auto id = value.second;
        auto otherMobile = id % this->mobiles.size();
        auto i = id / this->mobiles.size();
        auto collisionInterval = this->getCollisionInterval(
            polygon, this->movePolygons[id], this->startPolygons[id], this->endPolygons[id],
            this->paths[otherMobile].timedPositions[i], this->paths[otherMobile].timedPositions[i+1]
        );
        if (collisionInterval.start < collisionInterval.end)
        {
            collisionIntervals.push_back(collisionInterval);
        }
    }));
    std::sort(collisionIntervals.begin(), collisionIntervals.end());

    for (int i = collisionIntervals.size() - 2; i >= 0; i--)
    {
        if (collisionIntervals[i].end >= collisionIntervals[i+1].start)
        {
            collisionIntervals[i].end = collisionIntervals[i+1].end;
            collisionIntervals.erase(collisionIntervals.begin() + i + 1);
        }
    }

    return collisionIntervals;
}

Polygon ReservationTable::getBoundingPolygon(int mobile, const TimedPosition &start, const TimedPosition &end) const
{
    return this->getBoundingPolygon(mobile, start, end, nullptr, nullptr);
}

Polygon ReservationTable::getBoundingPolygon(int mobile, const TimedPosition &start, const TimedPosition &end,
    std::shared_ptr<Polygon> startPolygon, std::shared_ptr<Polygon> endPolygon) const
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

        if (startPolygon)
        {
            *startPolygon = circle[0];
        }
        if (endPolygon)
        {
            *endPolygon = circle[0];
        }

        return circle[0];
    }
    else
    {
        auto startPosition = this->graph.getPosition(start.vertex), endPosition = this->graph.getPosition(end.vertex);
        auto moveAngle = std::atan2(endPosition.y - startPosition.y, endPosition.x - startPosition.x);

        auto a = this->mobiles[mobile].length * std::cos(moveAngle) / 2;
        auto b = this->mobiles[mobile].width * std::sin(moveAngle) / 2;
        auto c = this->mobiles[mobile].length * std::sin(moveAngle) / 2;
        auto d = this->mobiles[mobile].width * std::cos(moveAngle) / 2;

        Polygon polygon;
        bg::append(polygon.outer(), Point(
            startPosition.x - a + b,
            startPosition.y - c - d
        ));
        bg::append(polygon.outer(), Point(
            startPosition.x - a - b,
            startPosition.y - c + d
        ));
        bg::append(polygon.outer(), Point(
            endPosition.x + a - b,
            endPosition.y + c + d
        ));
        bg::append(polygon.outer(), Point(
            endPosition.x + a + b,
            endPosition.y + c - d
        ));
        bg::append(polygon.outer(), Point(
            startPosition.x - a + b,
            startPosition.y - c - d
        ));

        if (startPolygon)
        {
            bg::append(startPolygon->outer(), Point(
                startPosition.x - a + b,
                startPosition.y - c - d
            ));
            bg::append(startPolygon->outer(), Point(
                startPosition.x - a - b,
                startPosition.y - c + d
            ));
            bg::append(startPolygon->outer(), Point(
                startPosition.x + a - b,
                startPosition.y + c + d
            ));
            bg::append(startPolygon->outer(), Point(
                startPosition.x + a + b,
                startPosition.y + c - d
            ));
            bg::append(startPolygon->outer(), Point(
                startPosition.x - a + b,
                startPosition.y - c - d
            ));
        }
        if (endPolygon)
        {
            bg::append(endPolygon->outer(), Point(
                endPosition.x - a + b,
                endPosition.y - c - d
            ));
            bg::append(endPolygon->outer(), Point(
                endPosition.x - a - b,
                endPosition.y - c + d
            ));
            bg::append(endPolygon->outer(), Point(
                endPosition.x + a - b,
                endPosition.y + c + d
            ));
            bg::append(endPolygon->outer(), Point(
                endPosition.x + a + b,
                endPosition.y + c - d
            ));
            bg::append(endPolygon->outer(), Point(
                endPosition.x - a + b,
                endPosition.y - c - d
            ));
        }

        return polygon;
    }
}

Interval ReservationTable::getCollisionInterval(const Polygon &edgePolygon, const Polygon &movePolygon, const Polygon &startPolygon, const Polygon &endPolygon,
    const TimedPosition &start, const TimedPosition &end) const
{
    std::deque<Polygon> collisionPolygons;
    bg::intersection(edgePolygon, movePolygon, collisionPolygons);

    if (collisionPolygons.empty())
    {
        return {std::numeric_limits<double>::max(), 0};
    }

    auto &collisionPolygon = collisionPolygons[0];
    auto startPosition = this->graph.getPosition(start.vertex), endPosition = this->graph.getPosition(end.vertex);
    auto distanceBeforeCollision = bg::distance(startPolygon, collisionPolygon);
    auto distanceAfterCollision = bg::distance(collisionPolygon, endPolygon);
    auto moveDistance = std::sqrt((startPosition.x - endPosition.x) * (startPosition.x - endPosition.x) + (startPosition.y - endPosition.y) * (startPosition.y - endPosition.y));
    auto timeBeforeCollision = (end.time - start.time) * distanceBeforeCollision / moveDistance;
    auto timeAfterCollision = (end.time - start.time) * distanceAfterCollision / moveDistance;

    return {start.time + timeBeforeCollision, end.time - timeAfterCollision};
}
