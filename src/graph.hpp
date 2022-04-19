#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <unordered_map>
#include <vector>

#include <boost/function.hpp>

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

typedef boost::function<bool (const Mobile &mobile)> EdgeCrossingCondition;
typedef boost::function<double (const Mobile &mobile, double distance)> EdgeCostFunction;

struct Edge
{
    int from, to;
    EdgeCrossingCondition crossingCondition;
    EdgeCostFunction costFunction;

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
    double distance(int from, int to);
    double getCost(const Edge &edge, const Mobile &mobile);
    double getCost(int from, int to, const Mobile &mobile);
    void setEdgeCrossingCondition(int from, int to, const EdgeCrossingCondition &crossingCondition);
    void setEdgeCostFunction(int from, int to, const EdgeCostFunction &costFunction);
};

#endif // GRAPH_HPP