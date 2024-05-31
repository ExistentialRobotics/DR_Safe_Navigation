#include <erl_utilities/erl_geom_utils.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace {
  TEST(TestGeometricUtilities, TestAngles)
  {
    // restrictAngle
    Eigen::VectorXd a(2);
    a << 270,45;
    Eigen::VectorXd b = a.unaryExpr([](double ang){ return erl::restrictAngle(erl::deg2rad(ang)); });
    ASSERT_TRUE(b.isApprox(Eigen::Vector2d(-1.57079633,0.785398),1e-6));    
  
    // angularDistance
    ASSERT_NEAR(erl::angularDistance(erl::deg2rad(45),erl::deg2rad(30)),erl::deg2rad(15),1e-6);
    ASSERT_NEAR(erl::angularDistance(erl::deg2rad(30),erl::deg2rad(45)),erl::deg2rad(15),1e-6);
    
    // angularMean
    a(0) = erl::deg2rad(45), a(1) = erl::deg2rad(30);
    b(0) = 0.5, b(1) = 0.5;    
    ASSERT_NEAR(erl::rad2deg(erl::angularMean(a,b)), 37.5, 1e-4);
  }
  
  TEST(TestGeometricUtilities, TestQuaternions)
  {
    Eigen::Vector4d q(0,1,0,0);
    auto R = erl::quat2rot(q);
    Eigen::Quaterniond eq(0,1,0,0);
    ASSERT_TRUE(R.isApprox(eq.toRotationMatrix(),1e-6));
    
    R = erl::quat2rot(Eigen::Vector4d(1,0,0,0));
    ASSERT_TRUE(R.isApprox(Eigen::Matrix3d::Identity()));
    
    for( unsigned k = 0; k < 5; ++k )
    {
      Eigen::Vector3d a = Eigen::Vector3d::Random(); 
      auto q = erl::quatExp(erl::axangle2tangentquat(a));
      auto R1 = erl::quat2rot(q);
      auto R2 = erl::axangle2rot(a);
      ASSERT_TRUE(R1.isApprox(R2,1e-6));
      
      Eigen::Matrix3d R3;
      R3 = Eigen::AngleAxisd(a.norm(),a/a.norm());
      ASSERT_TRUE(R1.isApprox(R3,1e-6));
      
      auto R4 = erl::skew2rot(erl::axangle2skew(a));
      ASSERT_TRUE(R1.isApprox(R4,1e-6));
    }
  }
  
  TEST(TestGeometricUtilities, TestRotations)
  {
    std::vector<double> a{erl::deg2rad(30),erl::deg2rad(45),erl::deg2rad(60),erl::deg2rad(90)};
    for( unsigned k = 0; k < a.size(); ++k )
    {
      auto Rx = erl::rotx(a[k]);
      ASSERT_TRUE(Rx.isApprox(Eigen::Matrix3d(Eigen::AngleAxisd(a[k],Eigen::Vector3d::UnitX())), 1e-6));
      
      auto Ry = erl::roty(a[k]);
      ASSERT_TRUE(Ry.isApprox(Eigen::Matrix3d(Eigen::AngleAxisd(a[k],Eigen::Vector3d::UnitY())), 1e-6));
      
      auto Rz = erl::rotz(a[k]);
      ASSERT_TRUE(Rz.isApprox(Eigen::Matrix3d(Eigen::AngleAxisd(a[k],Eigen::Vector3d::UnitZ())), 1e-6));
      
      auto R = erl::rot2d(a[k]);
      if(k == 0){ ASSERT_TRUE(R.isApprox( (Eigen::Matrix2d() << 0.8660,-0.5000,0.5000, 0.8660).finished(),1e-4)); }
      if(k == 1){ ASSERT_TRUE(R.isApprox( (Eigen::Matrix2d() << 0.7071,-0.7071,0.7071, 0.7071).finished(),1e-4)); }
    }
    
    auto R1 = erl::rot3d(a[0],a[1],a[2]);
    Eigen::Matrix3d R2;
    R2 = Eigen::AngleAxisd(a[2],Eigen::Vector3d::UnitZ())*
         Eigen::AngleAxisd(a[1],Eigen::Vector3d::UnitY())*
         Eigen::AngleAxisd(a[0],Eigen::Vector3d::UnitX());
   
    //std::cout << R1 << std::endl;
    //std::cout << R2 << std::endl;
    ASSERT_TRUE(R1.isApprox(R2,1e-6));

    auto S = erl::axangle2skew(Eigen::Vector3d(1,2,3));
    ASSERT_TRUE(S.isApprox( (Eigen::Matrix3d() << 0,-3,2,3,0,-1,-2,1,0).finished(), 1e-6 ));
  }
  
}


int main(int argc, char **argv) {
  std::srand((unsigned int) time(0));
  try {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  } catch (std::exception &e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }
  return 1;
}


