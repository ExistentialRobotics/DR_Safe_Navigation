//
// Created by brent on 6/22/19.
//

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "erl_env/environments/environment_2d.h"

using ::testing::Test;
using ::testing::Contains;

class Environment2DTest : public ::testing::Test {
 protected:
  std::unique_ptr<erl::Environment2D<uint16_t>> env;
  void SetUp() override {
    erl::GridMap<uint16_t> map;
    map.loadLegacy("./data/small_map_2d.yaml");
    env = std::make_unique<erl::Environment2D<uint16_t>>(erl::Environment2D<uint16_t>(map));
  }

  // void TearDown() override {}
};


// Constructor Tests

// Test No Exception for Empty Map
TEST(Environment2D, EmptyMap) {
  std::vector<double> mapmin = {-10, -10};
  std::vector<double> mapmax = {10, 10};
  std::vector<double> mapres = {.25, .25};

  ASSERT_NO_FATAL_FAILURE(erl::Environment2D<uint16_t> env(mapmin, mapmax, mapres));
}

// Assert that Wrong Map Size is Detected.
TEST(Environment2D, WrongSizeMap) {
  std::vector<double> mapmin = {-10, -10};
  std::vector<double> mapmax = {10, 10};
  std::vector<double> mapres = {.25, .25};
  std::vector<uint16_t> cmap = {1, 0};
  try {
    erl::Environment2D<uint16_t> env(mapmin, mapmax, mapres, cmap);

  } catch (erl::BadCostMapSize const &e) {
    EXPECT_EQ(e.what(), "Error. Check Cost Map Size.");
  }
}

// Construct From File
TEST(Environment2D, MapFromFile) {
  erl::GridMap<uint16_t> map;
  map.loadLegacy("./data/small_map_2d.yaml");
  ASSERT_NO_FATAL_FAILURE(erl::Environment2D<uint16_t> env(map));
}

// Test Successors
TEST_F(Environment2DTest, GetSuccessorsCorner) {
  std::vector<int> start = {0, 0};
  std::vector<std::vector<int>> succ_list;
  std::vector<double> succ_cost;
  std::vector<int> action_idx;
  ASSERT_NO_FATAL_FAILURE(env->getSuccessors(start, succ_list, succ_cost, action_idx));

  // Expected Successors
  std::vector<std::vector<int>> successors = {{1, 0}, {1, 1}, {0, 1}};
  EXPECT_THAT(succ_list.size(), 3);
  for (const auto &succ : successors) {
    EXPECT_THAT(succ_list, Contains(succ));
  }
}


// Test Successors
TEST_F(Environment2DTest, GetSuccessorsMiddle) {
  std::vector<int> start = {7, 7};
  std::vector<std::vector<int>> succ_list;
  std::vector<double> succ_cost;
  std::vector<int> action_idx;
  ASSERT_NO_FATAL_FAILURE(env->getSuccessors(start, succ_list, succ_cost, action_idx));

  // Expected Successors
  std::vector<std::vector<int>> successors =
      {{6, 6}, {6, 7}, {7, 6}, {8, 8}, {7, 8}, {8, 7}, {8, 6}, {6, 8}};
  EXPECT_THAT(succ_list.size(), 8);
  for (const auto &succ : successors) {
    EXPECT_THAT(succ_list, Contains(succ));
  }
}

// Test Eigen Matrix
TEST_F(Environment2DTest, EigenMat) {
  for (int i = 0; i < 600; i++) {
    Eigen::MatrixXd mat(2, 1);
    ASSERT_NO_FATAL_FAILURE(mat.rows());
  }
  std::vector<Eigen::MatrixXd> vecvec(600, Eigen::MatrixXd(2,1));

}
// Test Trajectory Conversion
TEST_F(Environment2DTest, ConvertTrajectory) {

  unsigned n_pts = 583;
  std::vector<std::vector<double>> path;
  for (int i = 0; i < n_pts; i++)
  {
    path.push_back({(double) i, 0.0});
  }
  ASSERT_NO_FATAL_FAILURE(env->toTrajectory(path));
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
