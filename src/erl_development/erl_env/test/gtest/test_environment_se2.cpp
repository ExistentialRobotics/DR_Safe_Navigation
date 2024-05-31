
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "erl_env/environments/environment_se2.h"
// TODO Add More Tests Here!!

class EnvironmentSE2Test : public ::testing::Test {
 protected:
  std::unique_ptr<erl::EnvironmentSE2<uint16_t>> env;

  void SetUp() override {
    std::vector<double> mapmin = {-10, -10};
    std::vector<double> mapmax = {10, 10};
    std::vector<double> mapres = {.1, .1};
    auto map = std::make_shared<erl::GridMap<uint16_t>>(mapmin, mapmax, mapres);
    std::vector<double> vlist = {1.0, 1.0, 1.0, 3.0, 3.0, 3.0};
    std::vector<double> wlist = {-M_PI / 2, 0, M_PI / 2, -M_PI / 2, 0, M_PI / 2};
    std::vector<double> tlist = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    env = std::make_unique<erl::EnvironmentSE2<uint16_t>>(erl::EnvironmentSE2<uint16_t>(map, vlist, wlist, tlist));

  }
};


// Test Primitives from List Construction Method.
TEST_F(EnvironmentSE2Test, ListPrimitives) {
  std::vector<double> start_m = {0, 0, 0};
  std::vector<int> start = env->toState(start_m);
  std::vector<std::vector<int>> succ_list;
  std::vector<double> succ_cost;
  std::vector<int> action_idx;
  ASSERT_NO_FATAL_FAILURE(env->getSuccessors(start, succ_list, succ_cost, action_idx));
  EXPECT_THAT(succ_list.size(), 6);
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
