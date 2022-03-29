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
    std::vector<Value> values;

    for (const auto &pair : paths) {
        const auto mobile = pair.first;
        const auto &path = pair.second;
        for (int i = 0; i < path.timedPositions.size() - 1; ++i)
        {
            auto box = this->getMinimumBoundingBox(mobile, path.timedPositions[i], path.timedPositions[i+1]);
            values.push_back({box, mobile + this->mobiles.size() * i});
        }
    }

    this->rtree = std::make_unique<RTree>(values);
}

std::vector<Interval> ReservationTable::getSafeIntervals(int mobile, int vertex)
{
    TimedPosition start{vertex, 0}, end{vertex, std::numeric_limits<double>::max()};
    Box box = this->getMinimumBoundingBox(mobile, start, end);

    std::vector<Interval> collisionIntervals;
    this->rtree->query(bgi::intersects(box), boost::make_function_output_iterator([&](auto const& value){
        int otherMobile = value.second % this->mobiles.size();
        int timedPositionIndex = value.second / this->mobiles.size();
        Interval collisionInterval = this->getCollisionInterval(
            mobile, start, end,
            otherMobile, this->paths[otherMobile].timedPositions[timedPositionIndex], this->paths[otherMobile].timedPositions[timedPositionIndex+1]
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
    Box box = this->getMinimumBoundingBox(mobile, start, end);

    std::vector<Interval> collisionIntervals;
    this->rtree->query(bgi::intersects(box), boost::make_function_output_iterator([&](auto const& value){
        int otherMobile = value.second % this->mobiles.size();
        int timedPositionIndex = value.second / this->mobiles.size();
        Interval collisionInterval = this->getCollisionInterval(
            mobile, start, end,
            otherMobile, this->paths[otherMobile].timedPositions[timedPositionIndex], this->paths[otherMobile].timedPositions[timedPositionIndex+1]
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

Box ReservationTable::getMinimumBoundingBox(int mobile, const TimedPosition &start, const TimedPosition &end) const
{
    auto startPosition = this->graph.getPosition(start.vertex), endPosition = this->graph.getPosition(end.vertex);
    auto moveAngle = std::atan2(endPosition.y - startPosition.y, endPosition.x - startPosition.x);
    auto xSize = this->mobiles[mobile].length * std::abs(std::cos(moveAngle)) + this->mobiles[mobile].width * std::abs(std::sin(moveAngle));
    auto ySize = this->mobiles[mobile].length * std::abs(std::sin(moveAngle)) + this->mobiles[mobile].width * std::abs(std::cos(moveAngle));
    return Box(
        Point(
            std::min(startPosition.x, endPosition.x) - xSize / 2,
            std::min(startPosition.y, endPosition.y) - ySize / 2,
            start.time
        ), 
        Point(
            std::max(startPosition.x, endPosition.x) + xSize / 2,
            std::max(startPosition.y, endPosition.y) + ySize / 2,
            end.time
        )
    );
}

Interval ReservationTable::getCollisionInterval(int mobile1, const TimedPosition &start1, const TimedPosition &end1,
                                                int mobile2, const TimedPosition &start2, const TimedPosition &end2) const
{
    // TODO
    return {0, 1};
}
