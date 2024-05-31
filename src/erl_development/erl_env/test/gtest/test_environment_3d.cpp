
#include <gtest/gtest.h>
#include <gmock/gmock.h>


#include "erl_env/environments/environment_3d.h"

// Constructor Tests

// Test No Exception for Empty Map
TEST(Environment3D, EmptyMap) {
  std::vector<double> mapmin = {-10, -10, 0};
  std::vector<double> mapmax = {10, 10, 3};
  std::vector<double> mapres = {.25, .25, .1};

  ASSERT_NO_FATAL_FAILURE(erl::Environment3D<uint16_t> env(mapmin, mapmax, mapres));
}

// TODO Write More Tests here.

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