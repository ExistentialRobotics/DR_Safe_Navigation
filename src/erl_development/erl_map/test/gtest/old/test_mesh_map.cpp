
#include <erl_map/mesh_map.h>
#include <erl_map/map_conversions.h>
#include <erl_utilities/erl_utils.h>

//#include <CGAL/IO/STL_reader.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

/*
#include <string>
#include <limits.h>
#include <unistd.h>
#include <sys/stat.h>

std::string getexepath()
{
  char result[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
  return std::string( result, (count > 0) ? count : 0 );
}


inline bool exists_test3 (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

void display_cgal_version()
{
  std::cout << "My CGAL library is " <<  CGAL_VERSION_NR << " (1MMmmb1000)" << std::endl; 
  std::cout << std::endl;
  std::cout << "where MM is the major number release, mm is the minor number release, and "
            << "b is the bug fixing number release." << std::endl;
}
*/

using ::testing::ElementsAre;
using ::testing::UnorderedElementsAre;
using ::testing::DoubleEq;
using ::testing::DoubleNear;


/**
 * Testing Fixtures
 */
class MeshMapTest : public ::testing::Test {
  public:
    erl::MeshMap meshmap;
    Eigen::Vector3d y;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    Eigen::Vector2d rb;
    Eigen::Vector2d ab;
    Eigen::Vector2d eb;
    
    
    void SetUp() override {
      EXPECT_THAT(meshmap.loadOFF("./data/cube1.off"),true);
      y << 1.0, 1.0,0.5;
      p <<-1.0,-1.0,0.5;
      R << 1.0,0.0,0.0,
           0.0,1.0,0.0,
           0.0,0.0,1.0;
      rb << 0.1,5.0;
      ab << -2.35619,2.35619;
      eb << -1.0,1.0;
    }
    // void TearDown() override {}
};

/**
* Begin Tests
*/
TEST_F(MeshMapTest, mapConstruction) {
  auto& m = meshmap.mesh();
  EXPECT_THAT(m.number_of_vertices(), 8);
  EXPECT_THAT(m.number_of_faces(), 12);
}


TEST_F(MeshMapTest, mapConversion) {
  std::vector<double> resolution({0.1,0.1,0.1});
  int fp_pow = 6;
  
  const auto bbox = CGAL::Polygon_mesh_processing::bbox(meshmap.mesh());
  EXPECT_THAT(bbox.xmin(), -1);
  EXPECT_THAT(bbox.ymin(), -1);
  EXPECT_THAT(bbox.zmin(), 0);
  EXPECT_THAT(bbox.xmax(), 1);
  EXPECT_THAT(bbox.ymax(), 1);
  EXPECT_THAT(bbox.zmax(), 1.4142);
  
  //std::cout << "xmin = " << bbox.xmin() << std::endl;
  //std::cout << "ymin = " << bbox.ymin() << std::endl;
  //std::cout << "zmin = " << bbox.zmin() << std::endl;

  //std::cout << "xmax = " << bbox.xmax() << std::endl;
  //std::cout << "ymax = " << bbox.ymax() << std::endl;
  //std::cout << "zmax = " << bbox.zmax() << std::endl;
  
  erl::GridMap<int8_t> gridmap = erl::toGridMap<int8_t>( meshmap.mesh(), resolution, fp_pow );
  EXPECT_THAT(gridmap.totalSize(), 21*21*15);
  
  // Display a 2D slice
  int z = erl::meters2cells(0.5,gridmap.min()[2],gridmap.res()[2]);
  for(unsigned y = 0; y < gridmap.size()[1]; ++y) {
    for(unsigned x = 0; x < gridmap.size()[0]; ++x)
      std::cout << static_cast<int>(gridmap.map()[gridmap.subv2ind({x,y,z})]) << " ";
    std::cout << std::endl;
  }
  
  
  /*
  // Display the slice
  erl::GridMap<int8_t> gridmap2d = gridmap.getSlice( {2}, {erl::meters2cells(0.5,gridmap.min()[2],gridmap.res()[2])});
  for(unsigned y = 0; y < gridmap2d.size()[1]; ++y) {
    for(unsigned x = 0; x < gridmap2d.size()[0]; ++x)
      std::cout << gridmap2d.map()[gridmap2d.subv2ind({x,y})] << " ";
    std::cout << std::endl;
  }
  */
  
  // Find an intersection point 
  std::vector<double> yv(y.data(), y.data()+y.size());
  std::vector<double> pv(p.data(), p.data()+p.size());
  std::vector<double> intersection_point;  
  bool occluded = gridmap.intersectSegment(pv, yv, intersection_point);
  //std::cout << "intersection_point = " << intersection_point[0] << " "
  //                                     << intersection_point[1] << " "
  //                                     << intersection_point[2] << " " << std::endl;  
                                   
  ASSERT_THAT(intersection_point, ElementsAre( DoubleNear(-0.5,0.0001), 
                                               DoubleNear(-0.5,0.0001), 
                                               DoubleNear(0.5,0.0001)));
}


TEST_F(MeshMapTest, visibility) {
  EXPECT_THAT(meshmap.isPointVisibleFromPose(y,R,p,rb,ab,eb),false);
  
  Eigen::Vector3d ymp = y-p;
  erl::MeshMap::Point pt(p.x(),p.y(),p.z());
  erl::MeshMap::Vector v(ymp.x(),ymp.y(),ymp.z());
  erl::MeshMap::Ray ray_query(pt,v);
  erl::MeshMap::Point intersection_point;
  bool occluded = erl::intersectRayAABBTree( ray_query, meshmap.tree(), intersection_point);
  //std::cout << "intersection_point = " << intersection_point.x() << " "
  //                                     << intersection_point.y() << " "
  //                                     << intersection_point.z() << " " << std::endl;
  ASSERT_THAT(intersection_point.x(), DoubleNear(-0.5,0.0001));
  ASSERT_THAT(intersection_point.y(), DoubleNear(-0.5,0.0001));
  ASSERT_THAT(intersection_point.z(), DoubleNear(0.5,0.0001));
}


// Main Function.
int main(int argc, char **argv) {  
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



/*
  std::cout << getexepath() << std::endl;
  std::cout << exists_test3("./data/cube1.off") << std::endl;
  
  display_cgal_version();
  erl::MeshMap::Mesh m1;
  std::ifstream input1("./data/cube1.off");
  input1 >> m1;
  std::cout << m1.number_of_vertices() << "  "  << m1.number_of_faces() << std::endl;
*/ 

