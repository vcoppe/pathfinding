#include <cmath>

#include "graph.hpp"

Graph::Graph()
{
}

Graph::~Graph()
{
    this->edges.clear();
    this->reverseEdges.clear();
}

void Graph::add(int vertex, const Position &position)
{
    this->positions[vertex] = position;
}

void Graph::add(Edge edge)
{
    if (!this->edges.contains(edge.from))
    {
        this->edges[edge.from] = std::unordered_map<int, Edge>();
    }
    if (!this->reverseEdges.contains(edge.to))
    {
        this->reverseEdges[edge.to] = std::unordered_map<int, Edge>();
    }
    this->edges[edge.from][edge.to] = edge;
    this->reverseEdges[edge.to][edge.from] = edge.getReverse();
}

int Graph::size() const
{
    return this->edges.size();
}

Position Graph::getPosition(int vertex)
{
    return this->positions[vertex];
}

const std::unordered_map<int, Edge> Graph::getEdges(int vertex)
{
    return this->edges[vertex];
}

const std::unordered_map<int, Edge> Graph::getReverseEdges(int vertex)
{
    return this->reverseEdges[vertex];
}

double Graph::manhattanDistance(int from, int to)
{
    if (from == to)
    {
        return 0.0;
    }

    double dx = this->positions[from].x - this->positions[to].x;
    double dy = this->positions[from].y - this->positions[to].y;
    return std::sqrt(dx * dx + dy * dy);
}

double Graph::getCost(const Edge &edge, const Mobile &mobile)
{
    return edge.costFunction(mobile, this->manhattanDistance(edge.from, edge.to));
}

double Graph::getCost(int from, int to, const Mobile &mobile)
{
    if (from == to)
    {
        return 0;
    }
    else
    {
        return this->edges[from][to].costFunction(mobile, this->manhattanDistance(from, to));
    }
}

void Graph::setEdgeCrossingCondition(int from, int to, const EdgeCrossingCondition &crossingCondition)
{
    this->edges[from][to].crossingCondition = crossingCondition;
    this->reverseEdges[to][from].crossingCondition = crossingCondition;
}

void Graph::setEdgeCostFunction(int from, int to, const EdgeCostFunction &costFunction)
{
    this->edges[from][to].costFunction = costFunction;
    this->reverseEdges[to][from].costFunction = costFunction;
}