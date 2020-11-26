#include <gtest/gtest.h>
#include <gtest/internal/gtest-port.h>
#include <vector>
#include <string>
#include <iostream>
#include "quintic_polynomial_planner/quintic_polynomial_planner.h"

using namespace std;
using namespace QuinticPolynomialPlanner;

TEST(QuinticPolynomialTest, Plan) {
    Input input;
    input.sx = 1.0;
    input.sy = 1.0;
    input.syaw = 0.0;
    input.sv = 0.0;
    input.sa = 0.0;
    input.gx = 9.0;
    input.gy = 5.0;
    input.gyaw = M_PI;
    input.min_t = 1.0;
    input.max_t = 10.0;
    input.dt = 0.2;
    auto output = plan(input);

	ASSERT_TRUE(output.has_value());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

