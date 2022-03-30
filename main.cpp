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

    Graph graph;
    for (int i = 0; i < 20; ++i)
    {
        graph.add(i, {0, 1.0 * i, 0});
    }
    graph.add(20, {2, 15, 0});

    for (int i = 0; i < 19; ++i)
    {
        graph.add({i, i+1, &crossingCondition, &costFunction});
        graph.add({i+1, i, &crossingCondition, &costFunction});
    }
    graph.add({15, 20, &crossingCondition, &costFunction});
    graph.add({20, 15, &crossingCondition, &costFunction});
    
    SafeIntervalPathPlanning planner(graph, mobiles);

    Path path = planner.plan(0, 0, 19, 0);
    for (auto it = path.timedPositions.begin(); it != path.timedPositions.end(); ++it)
    {
        std::cout << it->vertex << " " << it->time << std::endl;
    }

    path = planner.plan(1, 19, 0, 0);
    for (auto it = path.timedPositions.begin(); it != path.timedPositions.end(); ++it)
    {
        std::cout << it->vertex << " " << it->time << std::endl;
    }

    return 0;
}