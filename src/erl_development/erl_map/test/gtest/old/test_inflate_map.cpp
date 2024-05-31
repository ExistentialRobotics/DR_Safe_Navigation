#include "erl_map/grid_map_utils.h"
#include <erl_utilities/erl_utils.h>
#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>


namespace
{

TEST(TestInflateMap, TestInflation)
{
  //Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> cmap(11,11);
  std::vector<int> map_size{11,11};
  std::vector<double> map_res(2,0.2);
  std::vector<uint16_t> mymap(map_size[0]*map_size[1]);
  Eigen::Map<Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic>> cmap(mymap.data(), map_size[0], map_size[1]);  
  cmap.setZero(); cmap(6,6) = 10; cmap(0,10) = 10;
  double rad = 0.5;
  
  std::cout << cmap << std::endl;
  std::cout << std::endl;
  
  auto imap = erl::inflate_map2d( cmap, map_res, rad);
  std::cout << std::endl;
  std::cout << imap << std::endl;
  
  std::vector<uint16_t> imap2 = erl::inflateMap( mymap, map_size, map_res, false, {0.5,0.5} );    
  for( size_t k = 0; k < imap2.size(); ++k)
    EXPECT_THAT(imap2[k], imap(k));
}

}

int main( int argc, char** argv)
{
  try {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  } catch (std::exception &e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
  return EXIT_SUCCESS;  
}

