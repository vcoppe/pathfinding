#include <iostream>

#include "mobile.hpp"
#include "graph.hpp"
#include "safe_interval_path_planning.hpp"

bool crossingCondition(const Mobile &mobile)
{
    return true;
}

double costFunction(const Mobile &mobile, double distance)
{
    switch (mobile.type)
    {
        case Forklift:
            return distance / 2;
            break;
        case AGV:
            return distance;
            break;
        default:
            break;
    }
}

signed main() {
    std::vector<Mobile> mobiles;
    for (int i = 0; i < 5; ++i)
    {
        mobiles.push_back({AGV, 1, 1, 1});
    }

    std::vector<Position> positions;
    for (int i = 0; i < 20; ++i)
    {
        positions.push_back({0, 1.0 * i, 0});
    }

    Graph graph(positions);

    for (int i = 0; i < positions.size() - 1; ++i)
    {
        graph.add({i, i+1, 1.0, &crossingCondition, &costFunction});
    }
    
    SafeIntervalPathPlanning planner(graph, mobiles);

    Path path = planner.plan(0, 0, positions.size() - 1, 0);
    for (auto it = path.timedPositions.begin(); it != path.timedPositions.end(); ++it)
    {
        std::cout << it->vertex << " " << it->time << std::endl;
    }

    return 0;
}