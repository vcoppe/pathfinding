#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "algorithm_test.hpp"

using namespace testing;

TEST_F(AlgorithmTest, SafeIntervalPathPlanningEmptyReservationTable) {
    auto start = getVertexIndex(0, 0), goal = getVertexIndex(GRID_SIZE-1, GRID_SIZE-1);
    auto path = planner->plan(0, start, goal, 0);

    EXPECT_THAT(path.timedPositions, SizeIs(2*GRID_SIZE-1));
    EXPECT_THAT(path.timedPositions[0], FieldsAre(start, 0));
    EXPECT_THAT(path.timedPositions[path.timedPositions.size()-1], FieldsAre(goal, DoubleEq(2*(GRID_SIZE-1))));
}

TEST_F(AlgorithmTest, SafeIntervalPathPlanningNoSolution) {
    auto firstVertex = getVertexIndex(GRID_SIZE-1, GRID_SIZE-1) + 1;
    auto lastVertex = firstVertex + 20;
    for (int i = firstVertex; i <= lastVertex; ++i)
    {
        graph->add(i, {static_cast<double>(i), 0, 0});
    }
    for (int i = firstVertex; i < lastVertex; ++i)
    {
        graph->add({i, i+1, &AlgorithmTest::crossingCondition, &AlgorithmTest::costFunction});
        graph->add({i+1, i, &AlgorithmTest::crossingCondition, &AlgorithmTest::costFunction});
    }

    auto path1 = planner->plan(0, firstVertex, lastVertex, 0);

    EXPECT_THAT(path1.timedPositions, SizeIs(lastVertex - firstVertex + 1));
    EXPECT_THAT(path1.timedPositions[0], FieldsAre(firstVertex, 0));
    EXPECT_THAT(path1.timedPositions[path1.timedPositions.size()-1], FieldsAre(lastVertex, DoubleEq(lastVertex - firstVertex)));

    auto path2 = planner->plan(1, lastVertex, firstVertex, 0);

    EXPECT_THAT(path2.timedPositions, IsEmpty());
}

TEST_F(AlgorithmTest, SafeIntervalPathPlanningSolution) {
    auto firstVertex = getVertexIndex(GRID_SIZE-1, GRID_SIZE-1) + 1;
    auto lastVertex = firstVertex + 20;
    auto midVertexIndex = firstVertex + 15;
    auto additionalMidVertexIndex = lastVertex + 1;
    for (int i = firstVertex; i <= lastVertex; ++i)
    {
        graph->add(i, {static_cast<double>(i), 0, 0});
    }
    graph->add(additionalMidVertexIndex, {static_cast<double>(midVertexIndex), 1, 0});

    for (int i = firstVertex; i < lastVertex; ++i)
    {
        graph->add({i, i+1, &AlgorithmTest::crossingCondition, &AlgorithmTest::costFunction});
        graph->add({i+1, i, &AlgorithmTest::crossingCondition, &AlgorithmTest::costFunction});
    }
    graph->add({midVertexIndex, additionalMidVertexIndex, &AlgorithmTest::crossingCondition, &AlgorithmTest::costFunction});
    graph->add({additionalMidVertexIndex, midVertexIndex, &AlgorithmTest::crossingCondition, &AlgorithmTest::costFunction});

    auto path1 = planner->plan(0, firstVertex, lastVertex, 0);

    EXPECT_THAT(path1.timedPositions, SizeIs(lastVertex - firstVertex + 1));
    EXPECT_THAT(path1.timedPositions[0], FieldsAre(firstVertex, 0));
    EXPECT_THAT(path1.timedPositions[path1.timedPositions.size()-1], FieldsAre(lastVertex, DoubleEq(lastVertex - firstVertex)));

    auto path2 = planner->plan(1, lastVertex, firstVertex, 0);

    EXPECT_THAT(path2.timedPositions, SizeIs(Ge(lastVertex - firstVertex + 3)));
    EXPECT_THAT(path2.timedPositions[0], FieldsAre(lastVertex, 0));
    EXPECT_THAT(path2.timedPositions[path2.timedPositions.size()-1], FieldsAre(firstVertex, Ge(lastVertex - firstVertex + 2)));
    EXPECT_THAT(path2.timedPositions, Contains(FieldsAre(additionalMidVertexIndex, Gt(0))));
}