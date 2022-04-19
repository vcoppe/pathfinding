#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "algorithm_test.hpp"

using namespace testing;

TEST_F(AlgorithmTest, EmptyReservationTable) {
    for (int i = 0; i < GRID_SIZE; ++i)
    {
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            for (const auto &pair : graph->getEdges(getVertexIndex(i, j)))
            {
                const auto &edge = pair.second;
                auto collisionIntervals = reservationTable->getCollisionIntervals(0, edge.from, edge.to);
                EXPECT_THAT(collisionIntervals, IsEmpty());
            }
            auto safeIntervals = reservationTable->getSafeIntervals(0, getVertexIndex(i, j));
            EXPECT_THAT(safeIntervals, ElementsAre(FieldsAre(0, std::numeric_limits<double>::max())));
        }
    }
}

TEST_F(AlgorithmTest, ReservationTableSameMobile) {
    std::unordered_map<int, Path> paths;

    Path path;
    for (int j = 0; j < GRID_SIZE; ++j)
    {
        path.timedPositions.push_back({getVertexIndex(GRID_SIZE/2, j), static_cast<double>(10 + j)});
    }

    paths[0] = path;

    reservationTable->update(paths);

    for (int i = 0; i < GRID_SIZE; ++i)
    {
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            for (const auto &pair : graph->getEdges(getVertexIndex(i, j)))
            {
                const auto &edge = pair.second;
                auto collisionIntervals = reservationTable->getCollisionIntervals(0, edge.from, edge.to);
                EXPECT_THAT(collisionIntervals, IsEmpty());
            }
            auto safeIntervals = reservationTable->getSafeIntervals(0, getVertexIndex(i, j));
            EXPECT_THAT(safeIntervals, ElementsAre(FieldsAre(0, std::numeric_limits<double>::max())));
        }
    }
}

TEST_F(AlgorithmTest, ReservationTableDifferentMobile) {
    std::unordered_map<int, Path> paths;

    Path path;
    for (int j = 0; j < GRID_SIZE; ++j)
    {
        path.timedPositions.push_back({getVertexIndex(GRID_SIZE/2, j), static_cast<double>(10 + j)});
    }

    paths[0] = path;

    reservationTable->update(paths);

    for (int i = 0; i < GRID_SIZE; ++i)
    {
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            for (const auto &pair : graph->getEdges(getVertexIndex(i, j)))
            {
                const auto &edge = pair.second;
                auto collisionIntervals = reservationTable->getCollisionIntervals(1, edge.from, edge.to);
                if (getVertexI(edge.from) == GRID_SIZE/2 || getVertexI(edge.to) == GRID_SIZE/2)
                {
                    EXPECT_THAT(collisionIntervals, SizeIs(1));
                }
                else
                {
                    EXPECT_THAT(collisionIntervals, IsEmpty());
                }
            }
            auto safeIntervals = reservationTable->getSafeIntervals(1, getVertexIndex(i, j));
            if (i == GRID_SIZE/2)
            {
                EXPECT_THAT(safeIntervals, SizeIs(2));
            }
            else
            {
                EXPECT_THAT(safeIntervals, ElementsAre(FieldsAre(0, DoubleEq(std::numeric_limits<double>::max()))));
            }
        }
    }
}