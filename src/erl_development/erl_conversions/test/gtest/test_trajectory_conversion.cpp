//
// Created by brent on 7/8/19.
//

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "erl_conversions/erl_msg_utils.h"
#include <erl_utilities/trajectories/piecewise_polynomial.h>
#include <erl_utilities/trajectories/random_piecewise_polynomial.h>

using ::testing::Test;

class TrajectoryTest : public ::testing::Test {
 protected:
  trajectories::PiecewisePolynomial<double> pp;

  void SetUp() override {
    int num_segments = 5; // 5 segments
    int num_coefficients = 7; // 6th order
    int rows = 3; // 3rd dimension
    int cols = 1;

    std::vector<double> segment_times{0, 1, 2, 3, 4};
    pp = trajectories::MakeRandomPiecewisePolynomial<double>(rows, cols, num_coefficients, segment_times);
  }
};


TEST_F(TrajectoryTest, TestRoundTripFromTraj)
{
  erl_msgs::PrimitiveTrajectory traj_msg = erl::toROS(pp);
  trajectories::PiecewisePolynomial<double> poly = erl::fromROS(traj_msg);
  EXPECT_THAT(poly.isApprox(pp, 1e-3), true);

}

int main(int argc, char **argv) {
  std::cout << argv[0] << std::endl;
  try {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  } catch (std::exception &e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
    return 1;
  }
}