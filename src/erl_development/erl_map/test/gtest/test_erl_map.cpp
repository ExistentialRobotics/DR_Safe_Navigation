#include <erl_map/grid_map.h>
#include <erl_map/mesh_map.h>
#include <erl_map/map_conversions.h>
#include <erl_utilities/erl_utils.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>


using ::testing::ElementsAre;
using ::testing::UnorderedElementsAre;
using ::testing::DoubleEq;
using ::testing::DoubleNear;

/**
 * Testing Fixtures
 */
class TestGridMap2D : public ::testing::Test {
  public:
    erl::GridMap<uint16_t> gridmap;
    
    void SetUp() override {
      std::vector<int> mapdim{11,11};
      std::vector<double> origin{0.0,0.0};
      std::vector<double> mapres{0.2,0.2};
      gridmap = erl::GridMap<uint16_t>(mapdim, origin, mapres);
      gridmap.initZeroMap();
      gridmap.setMapValueCell({6,6},10);
      gridmap.setMapValueCell({0,10},10);
    }
    // void TearDown() override {}
};

class TestGridMap2DYAML : public ::testing::Test {
  public:
    erl::GridMap<int8_t> map_row, map_col;
  
    void SetUp() override {
      ASSERT_THAT(map_row.load("./data/map_2d_row.yaml"),true);
      ASSERT_THAT(map_col.load("./data/map_2d_col.yaml"),true);
    }
};

class TestGridMap3DYAML : public ::testing::Test {
 public:
  erl::GridMap<int8_t> map_row, map_col;
  void SetUp() override {
    ASSERT_THAT(map_row.load("./data/map_3d_row.yaml"),true);
    ASSERT_THAT(map_col.load("./data/map_3d_col.yaml"),true);
  }
};

class TestGridMap3DCube : public ::testing::Test {
  public:
    erl::GridMap<int8_t> gridmap;
    std::vector<double> y;
    std::vector<double> p;
    
    void SetUp() override {
      ASSERT_THAT(gridmap.load("./data/cube.yaml"),true);
      y = {1.0, 1.0,0.5};
      p = {-1.0,-1.0,0.5};
    }
};

class TestMeshMap : public ::testing::Test {
  public:
    erl::MeshMap meshmap;
    Eigen::Vector3d y;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    Eigen::Vector2d rb;
    Eigen::Vector2d ab;
    Eigen::Vector2d eb;
    
    
    void SetUp() override {
      ASSERT_THAT(meshmap.loadOFF("./data/cube.off"),true);
      y << 1.0, 1.0,0.5;
      p <<-1.0,-1.0,0.5;
      R << 1.0,0.0,0.0,
           0.0,1.0,0.0,
           0.0,0.0,1.0;
      rb << 0.1,5.0;
      ab << -2.35619,2.35619;
      eb << -1.0,1.0;
    }
};

class TestPolyMeshMap : public ::testing::Test {
  public:
    erl::MeshMap meshmap;
    void SetUp() override {
      ASSERT_THAT(meshmap.loadOFF("./data/cube_polygonal.off"),true);
    }
};


/**
* Begin Tests
*/

TEST_F(TestGridMap2D, inflateMap) {
  double rad = 0.5;
  Eigen::Map<const Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic>> cmap( gridmap.map().data(),
                                                                            gridmap.size()[0],
                                                                            gridmap.size()[1] );
  auto imap1 = erl::inflate_map2d( cmap, gridmap.res(), rad );
  auto imap2 = erl::inflateMap( gridmap.map(), gridmap.size(), gridmap.res(), gridmap.rowmajor(), {rad,rad} );
  for( size_t k = 0; k < imap2.size(); ++k)
    EXPECT_THAT(imap1(k), imap2[k]);
}

TEST_F(TestGridMap2D, linearIndex) {
  EXPECT_THAT(gridmap.subv2ind({1, 0}), 1); // Linear index for column major.
}




TEST_F(TestGridMap2DYAML, mapConstruction) {
  // Check Min
  EXPECT_THAT(map_row.min()[0], -2.0);
  EXPECT_THAT(map_row.min()[1], -2.0);

  // Check Max
  EXPECT_THAT(map_row.max()[0], 1.0);
  EXPECT_THAT(map_row.max()[1], 1.0);

  // Check Res
  EXPECT_THAT(map_row.res()[0], 1.0);
  EXPECT_THAT(map_row.res()[1], 1.0);

  // Check Size
  EXPECT_THAT(map_row.size()[0], 3);
  EXPECT_THAT(map_row.size()[1], 3);

  // Check Total Size
  EXPECT_THAT(map_row.map().size(), 9);  
}


TEST_F(TestGridMap2DYAML, linearIndex) {
  EXPECT_THAT(map_row.subv2ind({0, 0}), 0);
  EXPECT_THAT(map_row.subv2ind({0, 1}), 1);
  EXPECT_THAT(map_row.subv2ind({0, 2}), 2);
  EXPECT_THAT(map_row.subv2ind({1, 0}), 3);
  EXPECT_THAT(map_row.subv2ind({1, 1}), 4);
  EXPECT_THAT(map_row.subv2ind({1, 2}), 5);
  EXPECT_THAT(map_row.subv2ind({2, 0}), 6);
  EXPECT_THAT(map_row.subv2ind({2, 1}), 7);
  EXPECT_THAT(map_row.subv2ind({2, 2}), 8);

  EXPECT_THAT(map_col.subv2ind({0, 0}), 0);
  EXPECT_THAT(map_col.subv2ind({0, 1}), 3);
  EXPECT_THAT(map_col.subv2ind({0, 2}), 6);
  EXPECT_THAT(map_col.subv2ind({1, 0}), 1);
  EXPECT_THAT(map_col.subv2ind({1, 1}), 4);
  EXPECT_THAT(map_col.subv2ind({1, 2}), 7);
  EXPECT_THAT(map_col.subv2ind({2, 0}), 2);
  EXPECT_THAT(map_col.subv2ind({2, 1}), 5);
  EXPECT_THAT(map_col.subv2ind({2, 2}), 8);
}

TEST_F(TestGridMap2DYAML, testCMap) {
  // Row map
  EXPECT_THAT(map_row.map().at(map_row.subv2ind({0, 0})), 0);
  EXPECT_THAT(map_row.map().at(map_row.subv2ind({0, 1})), 1);
  EXPECT_THAT(map_row.map().at(map_row.subv2ind({0, 2})), 0);
  EXPECT_THAT(map_row.map().at(map_row.subv2ind({1, 0})), 0);
  EXPECT_THAT(map_row.map().at(map_row.subv2ind({1, 1})), 0);
  EXPECT_THAT(map_row.map().at(map_row.subv2ind({1, 2})), 1);
  EXPECT_THAT(map_row.map().at(map_row.subv2ind({2, 0})), 1);
  EXPECT_THAT(map_row.map().at(map_row.subv2ind({2, 1})), 0);
  EXPECT_THAT(map_row.map().at(map_row.subv2ind({2, 2})), 0);

  // Column will be Transposed.
  EXPECT_THAT(map_col.map().at(map_col.subv2ind({0, 0})), 0);
  EXPECT_THAT(map_col.map().at(map_col.subv2ind({0, 1})), 0);
  EXPECT_THAT(map_col.map().at(map_col.subv2ind({0, 2})), 1);
  EXPECT_THAT(map_col.map().at(map_col.subv2ind({1, 0})), 1);
  EXPECT_THAT(map_col.map().at(map_col.subv2ind({1, 1})), 0);
  EXPECT_THAT(map_col.map().at(map_col.subv2ind({1, 2})), 0);
  EXPECT_THAT(map_col.map().at(map_col.subv2ind({2, 0})), 0);
  EXPECT_THAT(map_col.map().at(map_col.subv2ind({2, 1})), 1);
  EXPECT_THAT(map_col.map().at(map_col.subv2ind({2, 2})), 0);
}



///////////////////////////////////////////////////////////////
//////////////   3D  TESTS                 ////////////////////
///////////////////////////////////////////////////////////////


TEST_F(TestGridMap3DYAML, mapConstructionRow) {
  // Check Min
  EXPECT_THAT(map_row.min()[0], -2.0);
  EXPECT_THAT(map_row.min()[1], -2.0);
  EXPECT_THAT(map_row.min()[2], 0.0);

  // Check Max
  EXPECT_THAT(map_row.max()[0], 1.0);
  EXPECT_THAT(map_row.max()[1], 1.0);
  EXPECT_THAT(map_row.max()[2], 3.0);


  // Check Res
  EXPECT_THAT(map_row.res()[0], 1.0);
  EXPECT_THAT(map_row.res()[1], 1.0);
  EXPECT_THAT(map_row.res()[1], 1.0);

  // Check Size
  EXPECT_THAT(map_row.size()[0], 3);
  EXPECT_THAT(map_row.size()[1], 3);
  EXPECT_THAT(map_row.size()[2], 3);

  // Check Total Size
  EXPECT_THAT(map_row.map().size(), 27);
}

TEST_F(TestGridMap3DYAML, mapConstructionCol) {
  // Check Min
  EXPECT_THAT(map_col.min()[0], -2.0);
  EXPECT_THAT(map_col.min()[1], -2.0);
  EXPECT_THAT(map_col.min()[2], 0.0);

  // Check Max
  EXPECT_THAT(map_col.max()[0], 1.0);
  EXPECT_THAT(map_col.max()[1], 1.0);
  EXPECT_THAT(map_col.max()[2], 3.0);


  // Check Res
  EXPECT_THAT(map_col.res()[0], 1.0);
  EXPECT_THAT(map_col.res()[1], 1.0);
  EXPECT_THAT(map_col.res()[1], 1.0);

  // Check Size
  EXPECT_THAT(map_col.size()[0], 3);
  EXPECT_THAT(map_col.size()[1], 3);
  EXPECT_THAT(map_col.size()[2], 3);

  // Check Total Size
  EXPECT_THAT(map_col.map().size(), 27);
}

TEST_F(TestGridMap3DYAML, linearIndex) {
  // Get Linear index for row and column major. (row, col, depth)
  EXPECT_THAT(map_row.subv2ind({0, 0, 0}), 0);
  EXPECT_THAT(map_row.subv2ind({0, 1, 0}), 3);
  EXPECT_THAT(map_row.subv2ind({0, 2, 0}), 6);
  EXPECT_THAT(map_row.subv2ind({1, 0, 0}), 9);
  EXPECT_THAT(map_row.subv2ind({1, 1, 0}), 12);
  EXPECT_THAT(map_row.subv2ind({1, 2, 0}), 15);
  EXPECT_THAT(map_row.subv2ind({2, 0, 0}), 18);
  EXPECT_THAT(map_row.subv2ind({2, 1, 0}), 21);
  EXPECT_THAT(map_row.subv2ind({2, 2, 0}), 24);
  EXPECT_THAT(map_row.subv2ind({0, 0, 1}), 1);
  EXPECT_THAT(map_row.subv2ind({0, 1, 1}), 4);
  EXPECT_THAT(map_row.subv2ind({0, 2, 1}), 7);
  EXPECT_THAT(map_row.subv2ind({1, 0, 1}), 10);
  EXPECT_THAT(map_row.subv2ind({1, 1, 1}), 13);
  EXPECT_THAT(map_row.subv2ind({1, 2, 1}), 16);
  EXPECT_THAT(map_row.subv2ind({2, 0, 1}), 19);
  EXPECT_THAT(map_row.subv2ind({2, 1, 1}), 22);
  EXPECT_THAT(map_row.subv2ind({2, 2, 1}), 25);

  EXPECT_THAT(map_col.subv2ind({0, 0, 0}), 0);
  EXPECT_THAT(map_col.subv2ind({0, 1, 0}), 3);
  EXPECT_THAT(map_col.subv2ind({0, 2, 0}), 6);
  EXPECT_THAT(map_col.subv2ind({1, 0, 0}), 1);
  EXPECT_THAT(map_col.subv2ind({1, 1, 0}), 4);
  EXPECT_THAT(map_col.subv2ind({1, 2, 0}), 7);
  EXPECT_THAT(map_col.subv2ind({2, 0, 0}), 2);
  EXPECT_THAT(map_col.subv2ind({2, 1, 0}), 5);
  EXPECT_THAT(map_col.subv2ind({2, 2, 0}), 8);
  EXPECT_THAT(map_col.subv2ind({0, 0, 1}), 9);
  EXPECT_THAT(map_col.subv2ind({0, 1, 1}), 12);
  EXPECT_THAT(map_col.subv2ind({0, 2, 1}), 15);
  EXPECT_THAT(map_col.subv2ind({1, 0, 1}), 10);
  EXPECT_THAT(map_col.subv2ind({1, 1, 1}), 13);
  EXPECT_THAT(map_col.subv2ind({1, 2, 1}), 16);
  EXPECT_THAT(map_col.subv2ind({2, 0, 1}), 11);
  EXPECT_THAT(map_col.subv2ind({2, 1, 1}), 14);
}



TEST_F(TestGridMap3DYAML, testSlice) {
  auto slice_map = map_row.get2DSlice(0);
  //auto map_nd = map_pair.first;
  //auto slice = map_pair.second;

  // Check Map sizes equal
  EXPECT_THAT(slice_map.totalSize(), slice_map.map().size());

  // Check Zero Slice
  for (size_t k = 0; k < slice_map.map().size(); ++k)
    EXPECT_THAT(slice_map.map()[k], 0);
  
  // Check Middle Slice
  slice_map = map_row.get2DSlice(1);
  std::vector<int> expected_result = {1, 0, 0, 1, 1, 0, 1, 1, 1};
  for (size_t k = 0; k < slice_map.map().size(); ++k)
    EXPECT_THAT(slice_map.map()[k], expected_result[k]);
  
  // Check One Slice
  slice_map = map_row.get2DSlice(2);
  for (size_t k = 0; k < slice_map.map().size(); ++k)
    EXPECT_THAT(slice_map.map()[k], 1);
    
  // Compare to ND slicing
  slice_map = map_row.getSlice({2},{1});
  for (size_t k = 0; k < slice_map.map().size(); ++k)
    EXPECT_THAT(slice_map.map()[k], expected_result[k]);  
}


TEST_F(TestGridMap3DCube, visibility) {
  EXPECT_THAT(gridmap.totalSize(), 21*21*15);
  
  // Raytracing
  std::vector<double> intersection_point;  
  bool occluded = gridmap.intersectSegment(p, y, intersection_point);
  EXPECT_THAT(occluded,true);
  EXPECT_THAT(intersection_point, ElementsAre( DoubleNear(-0.5,0.0001), 
                                               DoubleNear(-0.5,0.0001), 
                                               DoubleNear(0.5,0.0001)));
}

TEST_F(TestGridMap3DCube, testSlice) {
  int z_slice = erl::meters2cells(0.5,gridmap.min()[2],gridmap.res()[2]);
  erl::GridMap<int8_t> gridmap2d = gridmap.getSlice( {2}, {static_cast<unsigned>(z_slice)});
  for(int y = 0; y < gridmap2d.size()[1]; ++y)
    for(int x = 0; x < gridmap2d.size()[0]; ++x)
      EXPECT_THAT(gridmap2d.map()[gridmap2d.subv2ind({x,y})], gridmap.map()[gridmap.subv2ind({x,y,z_slice})]);  
}



///////////////////////////////////////////////////////////////
//////////////   MeshMap  TESTS           ////////////////////
//////////////////////////////////////////////////////////////

TEST_F(TestMeshMap, mapConstruction) {
  auto& m = meshmap.mesh();
  EXPECT_THAT(m.number_of_vertices(), 8);
  EXPECT_THAT(m.number_of_faces(), 12);
}

TEST_F(TestMeshMap, mapConversion) {
  const auto bbox = CGAL::Polygon_mesh_processing::bbox(meshmap.mesh());
  EXPECT_THAT(bbox.xmin(), -1);
  EXPECT_THAT(bbox.ymin(), -1);
  EXPECT_THAT(bbox.zmin(), 0);
  EXPECT_THAT(bbox.xmax(), 1);
  EXPECT_THAT(bbox.ymax(), 1);
  EXPECT_THAT(bbox.zmax(), 1.4142);
    
  std::vector<double> resolution{0.1,0.1,0.1};
  int fp_pow = 6;
  erl::GridMap<int8_t> gridmap = erl::toGridMap<int8_t>( meshmap.mesh(), resolution, fp_pow );
  EXPECT_THAT(gridmap.totalSize(), 21*21*15);
    
  erl::GridMap<int8_t> gridmap2;
  gridmap2.load("./data/cube.yaml");
  ASSERT_THAT(gridmap2.totalSize(),gridmap.totalSize()); 
  
  for( unsigned k = 0; k < gridmap.totalSize(); ++k )
    EXPECT_THAT(gridmap.map()[k], gridmap2.map()[k]);
}

TEST_F(TestMeshMap, visibility) {
  EXPECT_THAT(meshmap.isPointVisibleFromPose(y,R,p,rb,ab,eb),false);
  
  Eigen::Vector3d ymp = y-p;
  erl::MeshMap::Point pt(p.x(),p.y(),p.z());
  erl::MeshMap::Vector v(ymp.x(),ymp.y(),ymp.z());
  erl::MeshMap::Ray ray_query(pt,v);
  erl::MeshMap::Point intersection_point;
  bool occluded = erl::intersectRayAABBTree( ray_query, meshmap.tree(), intersection_point);
  EXPECT_THAT(occluded,true);
  EXPECT_THAT(intersection_point.x(), DoubleNear(-0.5,0.0001));
  EXPECT_THAT(intersection_point.y(), DoubleNear(-0.5,0.0001));
  EXPECT_THAT(intersection_point.z(), DoubleNear(0.5,0.0001));
}

TEST_F(TestPolyMeshMap, mapConstruction) {
  auto& m = meshmap.mesh();
  EXPECT_THAT(m.number_of_vertices(), 8);
  EXPECT_THAT(m.number_of_faces(), 6);
}





// Main Function.
int main(int argc, char **argv)
{
  try {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  } catch (std::exception &e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}

