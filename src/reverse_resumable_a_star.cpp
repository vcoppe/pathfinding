#include <limits>

#include "reverse_resumable_a_star.hpp"

ReverseResumableAStar::ReverseResumableAStar(const Graph &graph, const std::vector<Mobile> &mobiles)
    : graph(graph)
    , mobiles(mobiles)
{
}

ReverseResumableAStar::~ReverseResumableAStar()
{
    this->distance.clear();
    this->queue.clear();
    this->closed.clear();
}

void ReverseResumableAStar::init(int mobile, int from, int to)
{
    this->mobile = mobile;
    this->from = from;

    this->distance.clear();
    this->queue.clear();
    this->closed.clear();

    State root{to, 0, 0};

    this->queue.insert(root);
    this->distance[to] = root.g;
}

double ReverseResumableAStar::getHeuristic(int vertex)
{
    if (this->closed.contains(vertex))
    {
        return this->distance[vertex];
    }

    while (!this->queue.empty())
    {
        auto it = this->queue.begin();
        const auto current = *it;

        if (current.g > this->distance[current.vertex])
        {
            this->queue.erase(it);
            continue;
        }

        if (current.vertex == vertex)
        {
            return this->distance[vertex];
        }

        this->queue.erase(it);
        this->closed.insert(current.vertex);

        for (const auto &pair : this->graph.getReverseEdges(current.vertex))
        {
            const auto &edge = pair.second;

            if (!edge.canCross(this->mobiles[this->mobile]))
            {
                continue;
            }

            auto edgeCost = this->graph.getCost(edge, this->mobiles[this->mobile]);

            double successorDistance = current.g + edgeCost;

            auto it = this->distance.find(edge.to);
            if (it == this->distance.end() || successorDistance < it->second)
            {
                this->distance[edge.to] = successorDistance;
                auto h = this->graph.distance(this->from, edge.to) / this->mobiles[this->mobile].maxSpeed;
                this->queue.insert({edge.to, successorDistance, h});
            }
        }
    }

    return std::numeric_limits<double>::max();
}