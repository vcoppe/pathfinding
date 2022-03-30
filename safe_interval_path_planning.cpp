#include <algorithm>

#include "safe_interval_path_planning.hpp"

SafeIntervalPathPlanning::SafeIntervalPathPlanning(const Graph &graph, const std::vector<Mobile> &mobiles)
    : graph(graph)
    , mobiles(mobiles)
    , reservationTable(graph, mobiles)
    , reverseResumableAStar(graph, mobiles)
{
}

SafeIntervalPathPlanning::~SafeIntervalPathPlanning()
{
}

Path SafeIntervalPathPlanning::plan(int mobile, int from, int to, double start)
{
    auto it = this->paths.find(mobile);
    if (it != this->paths.end())
    {
        this->paths.erase(it);
    }

    this->reservationTable.update(this->paths);
    this->reverseResumableAStar.init(mobile, from, to);

    auto path = this->findPath(mobile, from, to, start);
    this->reservePath(mobile, path);

    return path;
}

Path SafeIntervalPathPlanning::findPath(int mobile, int from, int to, double start)
{
    this->distance.clear();
    this->visited.clear();
    this->queue.clear();
    this->parent.clear();

    auto rootSafeIntervals = this->reservationTable.getSafeIntervals(mobile, from);
    auto it = std::lower_bound(rootSafeIntervals.rbegin(), rootSafeIntervals.rend(), start, [](const Interval& interval, double value){
        return interval.start > value;
    });

    if (it == rootSafeIntervals.rend())
    {
        return {};
    }

    State root{from, 0, *it, start, 0};

    this->queue.insert(root);
    this->distance[root] = start;

    while (!queue.empty())
    {
        auto it = this->queue.begin();
        const auto current = *it;
        this->queue.erase(it);

        if (this->visited.contains(current) || current.g > this->distance[current])
        {
            continue;
        }
        
        this->visited.insert(current);

        if (current.vertex == to)
        {
            return this->getPath(mobile, current);
        }

        this->getSuccessors(mobile, current);
        for (const auto &successor : this->successors)
        {
            this->distance[successor] = successor.g;
            this->queue.insert(successor);
        }
    }

    return {};
}

void SafeIntervalPathPlanning::getSuccessors(int mobile, const State &state)
{
    this->successors.clear();

    for (const auto &pair : this->graph.getEdges(state.vertex))
    {
        const auto &edge = pair.second;

        if (!edge.canCross(this->mobiles[mobile]))
        {
            continue;
        }

        auto edgeCost = this->graph.getCost(edge, this->mobiles[mobile]);
        auto h = this->reverseResumableAStar.getHeuristic(edge.to);

        auto safeIntervals = this->reservationTable.getSafeIntervals(mobile, edge.to);
        auto collisionIntervals = this->reservationTable.getCollisionIntervals(mobile, edge.from, edge.to);

        for (int intervalId = 0; intervalId < safeIntervals.size(); ++intervalId)
        {
            Interval safeInterval = safeIntervals[intervalId];
            double successorDistance = state.g + edgeCost;

            if (successorDistance > safeInterval.end)
            {
                continue;
            }

            if (successorDistance < safeInterval.start)
            {
                successorDistance = safeInterval.start;
                if (successorDistance - edgeCost > state.interval.end)
                {
                    continue; // cannot wait at parent to reach this safe interval
                }
            }

            State successor{edge.to, intervalId, safeInterval, successorDistance, h};

            if (this->visited.contains(successor))
            {
                continue;
            }

            auto key = successor.g - edgeCost;
            auto it = std::lower_bound(collisionIntervals.rbegin(), collisionIntervals.rend(), key, [](const Interval& interval, double value){
                return interval.start > value;
            });

            if (it != collisionIntervals.rend())
            {
                auto collisionInterval = *it;
                if (successor.g - edgeCost < collisionInterval.end) { // collision
                    successor.g = collisionInterval.end + edgeCost;

                    if (successor.g - edgeCost > state.interval.end || successor.g > safeInterval.end)
                    {
                        continue;
                    }
                }
            }

            if (this->distance.contains(successor) && successor.g >= this->distance[successor])
            {
                continue;
            }

            this->successors.push_back(successor);
            this->parent[successor] = state;
        }
    }
}

Path SafeIntervalPathPlanning::getPath(int mobile, const State &state)
{
    Path path;

    auto current = state;
    while (true)
    {
        if (!path.timedPositions.empty())
        {
            const auto &previous = path.timedPositions.back();
            auto edgeCost = this->graph.getCost(current.vertex, previous.vertex, this->mobiles[mobile]);
            if (previous.time > current.g + edgeCost)
            {
                path.timedPositions.push_back({current.vertex, previous.time - edgeCost});
            }
        }
        path.timedPositions.push_back({current.vertex, current.g});
        if (this->parent.contains(current))
        {
            current = this->parent[current];
        }
        else
        {
            std::reverse(path.timedPositions.begin(), path.timedPositions.end());
            return path;
        }
    }
}

void SafeIntervalPathPlanning::reservePath(int mobile, const Path &path)
{
    this->paths[mobile] = path;
}