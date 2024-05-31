
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "erl_env/environments/environment_fas.h"

// Test the FAS system for 2 Dimensions with 3rd order inputs, i.e. 2-D Acceleration control inputs.
class EnvironmentFAS2D3OTest : public ::testing::Test {
 protected:
  std::unique_ptr<erl::EnvironmentFAS<2, 3, uint16_t>> env;
  void SetUp() override {
    std::vector<double> mapmin = {-10, -10};
    std::vector<double> mapmax = {10, 10};
    std::vector<double> mapres = {.1, .1};
    
    erl::GridMap<uint16_t> map(mapmin, mapmax, mapres);
    double min_vel = -1;
    double max_vel = 1;
    double min_acc = -1;
    double max_acc = 1;
    std::vector<double> dimmin =
        {mapmin[0], mapmin[1], min_vel, min_vel, min_acc, min_acc};
    std::vector<double> dimmax =
        {mapmin[0], mapmin[1], max_vel, max_vel, max_acc, max_acc};
    env = std::make_unique<erl::EnvironmentFAS<2, 3, uint16_t>>(
        erl::EnvironmentFAS<2, 3, uint16_t>(map, dimmin, dimmax, 0.9));
  }
  // void TearDown() override {}
};

// Test the FAS system for 3 Dimensions with 3rd order inputs, i.e. 3-D Acceleration control inputs
class EnvironmentFAS3D3OTest : public ::testing::Test {
 protected:
  std::unique_ptr<erl::EnvironmentFAS<3, 3, uint16_t>> env;
  void SetUp() override {
    std::vector<double> mapmin = {-10, -10, -2};
    std::vector<double> mapmax = {10, 10, 2};
    std::vector<double> mapres = {.1, .1, .1};
    erl::GridMap<uint16_t> map(mapmin, mapmax, mapres);
    double min_vel = -1;
    double max_vel = 1;
    double min_acc = -1;
    double max_acc = 1;
    std::vector<double> dimmin =
        {mapmin[0], mapmin[1], min_vel, min_vel, min_vel,  min_acc, min_acc, min_acc};
    std::vector<double> dimmax =
        {mapmin[0], mapmin[1], max_vel, max_vel, max_vel, max_acc, max_acc, max_acc};
    env = std::make_unique<erl::EnvironmentFAS<3, 3, uint16_t>>(
        erl::EnvironmentFAS<3, 3, uint16_t>(map, dimmin, dimmax, 0.9));
  }
  // void TearDown() override {}
};

// Test the FAS system for 2 Dimensions with 4th order inputs, i.e. 2-D Jerk control inputs.
class EnvironmentFAS2D4OTest : public ::testing::Test {
 protected:
  std::unique_ptr<erl::EnvironmentFAS<2, 4, uint16_t>> env;
  void SetUp() override {
    std::vector<double> mapmin = {-10, -10};
    std::vector<double> mapmax = {10, 10};
    std::vector<double> mapres = {.1, .1};
    erl::GridMap<uint16_t> map(mapmin, mapmax, mapres);
    double min_vel = -1;
    double max_vel = 1;
    double min_acc = -1;
    double max_acc = 1;
    double min_jrk = -1;
    double max_jrk = 1;
    std::vector<double> dimmin =
        {mapmin[0], mapmin[1], min_vel, min_vel, min_acc, min_acc, min_jrk, min_jrk};
    std::vector<double> dimmax =
        {mapmin[0], mapmin[1], max_vel, max_vel, max_acc, max_acc,  max_jrk, max_jrk};
    env = std::make_unique<erl::EnvironmentFAS<2, 4, uint16_t>>(
        erl::EnvironmentFAS<2, 4, uint16_t>(map, dimmin, dimmax, 0.9));
  }
  // void TearDown() override {}
};

// Test the FAS system for 3 Dimensions with 4th order inputs, i.e. 3-D Jerk control inputs
class EnvironmentFAS3D4OTest : public ::testing::Test {
 protected:
  std::unique_ptr<erl::EnvironmentFAS<3, 4, uint16_t>> env;
  void SetUp() override {
    std::vector<double> mapmin = {-10, -10, -2};
    std::vector<double> mapmax = {10, 10, 2};
    std::vector<double> mapres = {.1, .1, .1};
    erl::GridMap<uint16_t> map(mapmin, mapmax, mapres);
    double min_vel = -1;
    double max_vel = 1;
    double min_acc = -1;
    double max_acc = 1;
    double min_jrk = -1;
    double max_jrk = 1;
    std::vector<double> dimmin =
        {mapmin[0], mapmin[1], min_vel, min_vel, min_vel,
         min_acc, min_acc, min_acc, min_jrk, min_jrk, min_jrk};
    std::vector<double> dimmax =
        {mapmin[0], mapmin[1], max_vel, max_vel, max_vel,
         max_acc, max_acc, max_acc, max_jrk, max_jrk, max_jrk};
    env = std::make_unique<erl::EnvironmentFAS<3, 4, uint16_t>>(
        erl::EnvironmentFAS<3, 4, uint16_t>(map, dimmin, dimmax, 0.9));
  }
  // void TearDown() override {}
};

// Test Constructors
TEST_F(EnvironmentFAS2D3OTest, TestConstruction) {}
TEST_F(EnvironmentFAS3D3OTest, TestConstruction) {}
TEST_F(EnvironmentFAS2D4OTest, TestConstruction) {}
TEST_F(EnvironmentFAS3D4OTest, TestConstruction) {}


// Test 2D3O Trajectory Generation
TEST_F(EnvironmentFAS2D3OTest, TestTrajectory) {
  erl::Waypoint2D3O start;
  start.coord[0] = 0; // X pos
  start.coord[1] = 0; // Y pos
  start.coord[2] = 0; // X vel
  start.coord[3] = 0; // Y vel
  start.coord[4] = 0; // X acc
  start.coord[5] = 0; // Y acc

  std::vector<erl::Waypoint2D3O> succ_list;
  std::vector<double> succ_cost;
  std::vector<int> action_idx;
  ASSERT_NO_FATAL_FAILURE(env->getSuccessors(start, succ_list, succ_cost, action_idx));

  for (int i = 0; i < succ_list.size(); i++)
  {
    ASSERT_NO_FATAL_FAILURE(env->toTrajectorySingle(succ_list[i], {action_idx[i]}));
    ASSERT_NO_FATAL_FAILURE(env->toTrajectory({succ_list[i]}, {action_idx[i]}));
  }
}


// TODO Write More Extensive Tests here.

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
