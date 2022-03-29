#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <utility>
#include <vector>

#include <boost/function.hpp>
#include <boost/functional/hash.hpp>

#include "mobile.hpp"

struct Position
{
    double x, y, z;
};

struct TimedPosition
{
    int vertex;
    double time;

    bool operator<(const TimedPosition &other) const
    {
        return time < other.time;
    }
};

struct Edge
{
    int from, to;
    double distance;
    boost::function<bool (const Mobile &mobile)> crossingCondition;
    boost::function<double (const Mobile &mobile, double distance)> costFunction;

    bool canCross(Mobile mobile) const
    {
        return crossingCondition(mobile);
    }

    double getCost(Mobile mobile) const
    {
        return costFunction(mobile, distance);
    }

    Edge getReverse() const
    {
        return {to, from, distance, crossingCondition, costFunction};
    }
};

struct Path
{
    std::vector<TimedPosition> timedPositions;
};

class Graph
{
private:
    const std::vector<Position> positions;
    std::vector<std::vector<Edge> > edges, reverseEdges;
public:
    Graph(const std::vector<Position> &positions);
    ~Graph();

    int size() const;
    Position getPosition(int vertex) const;
    const std::vector<Edge> getEdges(int vertex) const;
    const std::vector<Edge> getReverseEdges(int vertex) const;
    double manhattanDistance(int from, int to) const;
    void add(Edge edge);
};

#endif // GRAPH_HPP