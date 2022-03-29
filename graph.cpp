#include <cmath>

#include "graph.hpp"

Graph::Graph(const std::vector<Position> &positions) : positions(positions)
{
    this->edges.resize(positions.size());
    this->reverseEdges.resize(positions.size());
}

Graph::~Graph()
{
    this->edges.clear();
    this->reverseEdges.clear();
}

int Graph::size() const
{
    return this->edges.size();
}

Position Graph::getPosition(int vertex) const
{
    return this->positions[vertex];
}

const std::vector<Edge> Graph::getEdges(int vertex) const
{
    return this->edges[vertex];
}

const std::vector<Edge> Graph::getReverseEdges(int vertex) const
{
    return this->reverseEdges[vertex];
}

double Graph::manhattanDistance(int from, int to) const
{
    double dx = this->positions[from].x - this->positions[to].x;
    double dy = this->positions[from].y - this->positions[to].y;
    return std::sqrt(dx * dx + dy * dy);
}

void Graph::add(Edge edge)
{
    this->edges[edge.from].push_back(edge);
    this->reverseEdges[edge.to].push_back(edge.getReverse());
}