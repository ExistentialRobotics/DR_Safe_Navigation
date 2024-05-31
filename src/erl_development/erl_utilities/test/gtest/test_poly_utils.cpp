//
// Created by brent on 12/10/18.
//
#include <erl_utilities/erl_poly_utils.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>


using ::testing::UnorderedElementsAre;

using ::testing::DoubleEq;
using ::testing::DoubleNear;

// Test Quadratic Solver
TEST(TestQuadraticSolver, SolveQuadratics)
{
  // Test standard solution.
  auto solution = erl::quad(1, -3, 2); // X^2 -3X + 2 = (X-2)(X-1)
  EXPECT_THAT(solution, UnorderedElementsAre(1.0, 2.0));
  // Test repeated solution.
  solution = erl::quad(1, -2, 1); // X^2 -2X + 1 = (X-1)^2.
  EXPECT_THAT(solution, UnorderedElementsAre(1.0, 1.0));
  // Test Imaginary solution.
  solution = erl::quad(1, 1, 1); // X^2 + X + 1 = -1/2 + j * sqrt(3)/2.
  EXPECT_THAT(solution.size(), 0);
  // Test Linear Solution (single value returned).
  solution = erl::quad(0, 1, -1); // 0X^2 +X - 1 = (X-1)
  EXPECT_THAT(solution, UnorderedElementsAre(1.0));
  EXPECT_THAT(solution.size(), 1);
}

// Test Cubic Solver
TEST(TestCubicSolver, SolveCubic) {
  // Standard Cubic
  auto solution = erl::cubic(1, -6, 11, -6); // X^3 - 6X^2 + 11X -6 = (X-2)(X-1)(X-3)
  EXPECT_THAT(solution, UnorderedElementsAre(DoubleEq(1.0), DoubleEq(2.0), DoubleEq(3.0)));
}

// Test Quartic Solver
TEST(TestQuarticSolver, SolveQuartic) {
  // Standard Quartic
  auto solution = erl::quartic(1, -6, 13, -12, 4);
  EXPECT_THAT(solution, UnorderedElementsAre(DoubleEq(1.0), DoubleEq(1.0), DoubleEq(2.0), DoubleEq(2.0)));
}

// Test Quintic Solver
TEST(TestQuinticSolver, SolveQuintic) {
  // Standard Quintic
  auto solution = erl::solve(1, -6, 13, -12, 4, 0);
  EXPECT_THAT(solution, UnorderedElementsAre(DoubleNear(1.0, 1e-5), DoubleNear(1.0, 1e-5), DoubleEq(2.0), DoubleEq(2.0), DoubleNear(0.0, 1e-5)));
}
// Test 6-th Order Solver
TEST(TestSixthPoly, SolveSixth) {
  // Standard Quintic
  auto solution = erl::solve(1, -6, 13, -12, 4, 0, 0);
  EXPECT_THAT(solution, UnorderedElementsAre(DoubleNear(1.0, 1e-5), DoubleNear(1.0, 1e-5), DoubleNear(2.0, 1e-5), DoubleNear(2.0, 1e-5),
      DoubleNear(0.0, 1e-5), DoubleNear(0.0, 1e-5)));
}

// Test Power
TEST(test_power, compute_powers) {
  auto solution = erl::power(2.0, 2);
  EXPECT_NEAR(solution, 4.0, 1e-5);

  solution = erl::power(2.1, 3);
  EXPECT_NEAR(solution, 2.1 * 2.1 * 2.1, 1e-5);
}

// Test pseudoInverse
TEST(TestPseudoInverse, ComputeInverse) {

}
// Test SquareRoot
TEST(TestSquareRoot, ComputeSquareRoot) {


}

// Main Function.
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}