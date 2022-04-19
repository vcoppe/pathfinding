#ifndef ALGORITHM_TEST_HPP
#define ALGORITHM_TEST_HPP

#include <gtest/gtest.h>

#include "../src/graph.hpp"
#include "../src/reservation_table.hpp"
#include "../src/reverse_resumable_a_star.hpp"
#include "../src/safe_interval_path_planning.hpp"
#include "../src/structs.hpp"

class AlgorithmTest : public ::testing::Test
{
protected:
    const int GRID_SIZE = 20, N_MOBILES = 20;
    std::shared_ptr<Graph> graph;
    std::vector<Mobile> mobiles;
    std::shared_ptr<SafeIntervalPathPlanning> planner;
    std::shared_ptr<ReverseResumableAStar> reverseResumableAStar;
    std::shared_ptr<ReservationTable> reservationTable;

    virtual void SetUp()
    {
        this->graph = std::make_shared<Graph>();
        for (int i = 0; i < GRID_SIZE; ++i)
        {
            for (int j = 0; j < GRID_SIZE; ++j)
            {
                this->graph->add(this->getVertexIndex(i, j), {static_cast<double>(i), static_cast<double>(j), 0});
            }
        }

        for (int i = 0; i < GRID_SIZE; ++i)
        {
            for (int j = 0; j < GRID_SIZE; ++j)
            {
                if (i-1 >= 0)
                {
                    this->graph->add({this->getVertexIndex(i, j), this->getVertexIndex(i-1, j), &AlgorithmTest::crossingCondition, &AlgorithmTest::costFunction});
                }
                if (i+1 < GRID_SIZE)
                {
                    this->graph->add({this->getVertexIndex(i, j), this->getVertexIndex(i+1, j), &AlgorithmTest::crossingCondition, &AlgorithmTest::costFunction});
                }
                if (j-1 >= 0)
                {
                    this->graph->add({this->getVertexIndex(i, j), this->getVertexIndex(i, j-1), &AlgorithmTest::crossingCondition, &AlgorithmTest::costFunction});
                }
                if (j+1 < GRID_SIZE)
                {
                    this->graph->add({this->getVertexIndex(i, j), this->getVertexIndex(i, j+1), &AlgorithmTest::crossingCondition, &AlgorithmTest::costFunction});
                }
            }
        }

        for (int i = 0; i < N_MOBILES; ++i)
        {
            this->mobiles.push_back({AGV, 0.8, 0.8, 1});
        }

        this->reservationTable = std::make_shared<ReservationTable>(graph, mobiles);
        this->reverseResumableAStar = std::make_shared<ReverseResumableAStar>(graph, mobiles);
        this->planner = std::make_shared<SafeIntervalPathPlanning>(graph, mobiles, this->reservationTable, this->reverseResumableAStar);
    }

    virtual void TearDown()
    {
    }

    int getVertexIndex(int i, int j)
    {
        return i * GRID_SIZE + j;
    }

    int getVertexI(int vertex)
    {
        return vertex / GRID_SIZE;
    }

    int getVertexJ(int vertex)
    {
        return vertex % GRID_SIZE;
    }

    static bool crossingCondition(const Mobile &mobile)
    {
        return true;
    }

    static double costFunction(const Mobile &mobile, double distance)
    {
        return distance;
    }
};

#endif // ALGORITHM_TEST_HPP