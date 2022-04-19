#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "algorithm_test.hpp"

using namespace testing;

TEST_F(AlgorithmTest, ReverseResumableAStar) {
    reverseResumableAStar->init(0, getVertexIndex(0, 0), getVertexIndex(GRID_SIZE-1, GRID_SIZE-1));

    for (int i = 0; i < GRID_SIZE; ++i)
    {
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            EXPECT_EQ(reverseResumableAStar->getHeuristic(getVertexIndex(i, j)), 2*(GRID_SIZE-1)-i-j);
        }
    }
}