#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/function.hpp>
#include <boost/functional/hash.hpp>

#include "structs.hpp"

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
    boost::function<bool (const Mobile &mobile)> crossingCondition;
    boost::function<double (const Mobile &mobile, double distance)> costFunction;

    bool canCross(const Mobile &mobile) const
    {
        return crossingCondition(mobile);
    }

    Edge getReverse() const
    {
        return {to, from, crossingCondition, costFunction};
    }
};

struct Path
{
    std::vector<TimedPosition> timedPositions;
};

class Graph
{
private:
    std::unordered_map<int, Position> positions;
    std::unordered_map<int, std::unordered_map<int, Edge> > edges, reverseEdges;
public:
    Graph();
    ~Graph();

    void add(int vertex, const Position &position);
    void add(Edge edge);
    int size() const;
    Position getPosition(int vertex);
    const std::unordered_map<int, Edge> getEdges(int vertex);
    const std::unordered_map<int, Edge> getReverseEdges(int vertex);
    double manhattanDistance(int from, int to);
    double getCost(const Edge &edge, const Mobile &mobile);
    double getCost(int from, int to, const Mobile &mobile);
};

#endif // GRAPH_HPP