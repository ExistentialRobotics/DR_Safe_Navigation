#include "erl_map/grid_map_utils.h"
#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace
{

TEST(TestBresenham, Test3D)
{
  double sx = 0.2, sy = 0.2, sz = 0.0;
  double ex = -21.0132, ey = 21.4132, ez = 0.0;
  double xmin = -0.1, ymin = -0.1, zmin = -0.1;
  double xres = 0.2, yres = 0.2, zres = 0.2;
  std::vector<int> xvec;
  std::vector<int> yvec;
  std::vector<int> zvec;
  
  erl::bresenham2D( sx,sy,ex,ey, xmin,ymin,
                  xres, yres, xvec, yvec );
  //for( unsigned k = 0; k < xvec.size(); ++k )
  //  std::cout << "(" << xvec[k] <<"," << yvec[k] << ")" << std::endl;
  
  erl::bresenham3D( sx,sy,sz, ex,ey,ez,
                    xmin,ymin,zmin, xres, yres, zres,
                    xvec, yvec, zvec );
  
  //for( unsigned k = 0; k < xvec.size(); ++k )
  //  std::cout << "(" << xvec[k] <<"," << yvec[k] << "," << zvec[k] << ")" << std::endl;
  
  
                  
  sx =-0.6743, sy=-1.2151, sz=3.1086;
  ex =-6.3607, ey=-3.0567, ez=-7.2978;
  xmin = -4.5, ymin = -4.5, zmin = -4.5;
  xres = 1.0, yres = 1.0, zres = 1.0;
  erl::bresenham3D( sx,sy,sz, ex,ey,ez,
                    xmin,ymin,zmin, xres, yres, zres,
                    xvec, yvec, zvec );
  
  //for( unsigned k = 0; k < xvec.size(); ++k )
  //  std::cout << "(" << xvec[k] <<"," << yvec[k] << "," << zvec[k] << ")" << std::endl;

  auto cells = erl::bresenham( {-0.6743,-1.2151,3.1086}, {-6.3607,-3.0567,-7.2978}, {-4.5,-4.5,-4.5}, {1.0,1.0,1.0} );
  for( unsigned k = 0; k < xvec.size(); ++k )
  {
    EXPECT_THAT(cells[k][0], xvec[k]);
    EXPECT_THAT(cells[k][1], yvec[k]);
    EXPECT_THAT(cells[k][2], zvec[k]);
  }
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

