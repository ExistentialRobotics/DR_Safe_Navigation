
#include <erl_map/grid_map.h>
#include <erl_utilities/erl_utils.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

using ::testing::UnorderedElementsAre;
using ::testing::DoubleEq;
using ::testing::DoubleNear;

/**
 * Testing Fixtures
 */
class MapTest2D : public ::testing::Test {
 public:
  std::vector<double> mapmin;
  std::vector<double> mapmax;
  std::vector<double> mapres;
  erl::GridMap<uint16_t> map;

  void SetUp() override {
    mapmin = {-2, -2};
    mapmax = {1, 1};
    mapres = {1, 1};
    map = erl::GridMap<uint16_t>(mapmin, mapmax, mapres);
  }
  // void TearDown() override {}
};

class MapTest2DYAML : public ::testing::Test {
 public:
  erl::GridMap<uint16_t> map_row, map_col;
  void SetUp() override {
    map_row.loadLegacy("./data/map_2d_row.yaml");
    map_col.loadLegacy("./data/map_2d_col.yaml");
  }
};

class MapTest3DYAML : public ::testing::Test {
 public:
  erl::GridMap<uint16_t> map_row, map_col;
  void SetUp() override {
    map_row.loadLegacy("./data/map_3d_row.yaml");
    map_col.loadLegacy("./data/map_3d_col.yaml");
  }
};


/**
* Begin Tests
*/
TEST_F(MapTest2D, mapConstruction) {
  // Check Min
  EXPECT_THAT(map.min()[0], -2.0);
  EXPECT_THAT(map.min()[1], -2.0);

  // Check Max
  EXPECT_THAT(map.max()[0], 1.0);
  EXPECT_THAT(map.max()[1], 1.0);

  // Check Res
  EXPECT_THAT(map.res()[0], 1.0);
  EXPECT_THAT(map.res()[1], 1.0);

  // Check Size
  EXPECT_THAT(map.size()[0], 3);
  EXPECT_THAT(map.size()[1], 3);

  // Check Total Size
  EXPECT_THAT(map.map().size(), 0);
}

TEST_F(MapTest2D, linearIndex) {
  // Get Linear index for column major.
  EXPECT_THAT(map.subv2ind({1, 0}), 1);
}

TEST_F(MapTest2DYAML, mapConstruction) {
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

TEST_F(MapTest2DYAML, linearIndex) {
  // Get Linear index for row and column major.
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

TEST_F(MapTest2DYAML, testCMap) {

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

TEST_F(MapTest3DYAML, mapConstructionRow) {
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


TEST_F(MapTest3DYAML, mapConstructionCol) {
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

TEST_F(MapTest3DYAML, linearIndex) {
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


TEST_F(MapTest3DYAML, testSlice) {
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


// Main Function.
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


