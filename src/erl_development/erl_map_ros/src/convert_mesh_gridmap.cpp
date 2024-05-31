#include <ros/ros.h>
#include <ros/console.h>

#include <erl_map/grid_map.h>
#include <erl_conversions/erl_msg_utils.h>
#include <erl_msgs/GridMap.h>
#include <erl_map/map_conversions.h>

template<typename T>
inline void convertMap(const std::string &fname, double res_xy, double res_z, bool use_2d) {
  // Load the meshmap
  // Code Here
  erl::MeshMap meshmap;
  meshmap.loadSTL(fname);


  // Convert meshmap to Gridmap
  std::vector<double> resolution{res_xy, res_xy, res_z};
  int fp_pow = 6;
  erl::GridMap<T> newmap = erl::toGridMap<T>(meshmap.mesh(), resolution, fp_pow);

//
//  erl::GridMap<T> oldmap;
//  ROS_INFO("Loading map from file \"%s\"", fname.c_str());
//  if( !oldmap.loadLegacy( fname ) )
//  {
//    ROS_ERROR("gridmap_server could not open %s.", fname.c_str());
//    exit(-1);
//  }
//
//  // Force map to be colmajor
//  //oldmap.toColmajor();
//
//  // Convert the map
//  // oldmap.rowmajor()
//  erl::GridMap<int8_t> newmap(oldmap.size(), oldmap.origin(), oldmap.res(), oldmap.rowmajor());
//  std::vector<int8_t> map(oldmap.map().size());
//  for(unsigned k = 0; k < map.size(); ++k) {
//    if (std::is_same<T, char>::value)
//      map[k] = oldmap.map()[k] == '1' ? int8_t(1) : int8_t(0);
//    else
//      map[k] = static_cast<int8_t>(oldmap.map()[k]);
//  }
//
//  newmap.setMap(map);
//
//  /*
//  // Set edges to occupied
//  for(unsigned k = 0; k < oldmap.size()[0]; ++k ) {
//    newmap.setMapValueCell({k,0},int8_t(1));
//    newmap.setMapValueCell({k,1},int8_t(1));
//  }
//  for(unsigned k = 0; k < oldmap.size()[1]; ++k ) {
//    newmap.setMapValueCell({0,k},int8_t(1));
//    newmap.setMapValueCell({1,k},int8_t(1));
//  }
//  */
//
  if (use_2d) {

    auto cells = newmap.meters2cells({0.0, 0.0, 0.0});
    newmap = newmap.get2DSlice(cells[2]);
    boost::filesystem::path bfp(fname);
    std::string parentpath(bfp.parent_path().string());
    std::string filestem(bfp.stem().string());
    newmap.save(parentpath + "/" + filestem + "2d");
  }
  else {
    // Save the new map
    boost::filesystem::path bfp(fname);
    std::string parentpath(bfp.parent_path().string());
    std::string filestem(bfp.stem().string());
    newmap.save(parentpath + "/" + filestem);
  }

  switch(newmap.dim())
  {
    case 1:
      ROS_INFO("Read a map with %d cells at %.3lf m/cell",
        newmap.size()[0], newmap.res()[0]);
      break;
    case 2:
      ROS_INFO("Read a %d x %d map @ %.3lf x %.3lf m/cell",
        newmap.size()[0], newmap.size()[1],
        newmap.res()[0], newmap.res()[1]);
      break;
    case 3:
      ROS_INFO("Read a %d x %d x %d map @ %.3lf x %.3lf x %.3lf m/cell",
        newmap.size()[0], newmap.size()[1], newmap.size()[2],
        newmap.res()[0], newmap.res()[1], newmap.res()[2]);
      break;
    default:
      ROS_INFO("Read a %d dimensional map", static_cast<int>(newmap.dim()));
  }
}

int main(int argc, char **argv) {

  // Init ros node
  ros::init(argc, argv, "convert_mesh_gridmap", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Publisher map_pub = nh.advertise<erl_msgs::GridMap>("map", 1, true);

  // Convert map to new format
  std::string fname(argv[1]);
  std::string dtype;
  double res_xy, res_z;
  bool make_2d {false};
  private_nh.param("data_type", dtype, std::string("char"));
  private_nh.param("res_xy", res_xy, 0.1);
  private_nh.param("res_z", res_z, 0.1);
  private_nh.param("make_2d", make_2d, false);

  if (dtype.compare("char") == 0)
    convertMap<char>(fname, res_xy, res_z, make_2d);
  if (dtype.compare("int") == 0)
    convertMap<int>(fname, res_xy, res_z, make_2d);
  if (dtype.compare("int16_t") == 0)
    convertMap<int16_t>(fname, res_xy, res_z, make_2d);
  if (dtype.compare("uint16_t") == 0)
    convertMap<uint16_t>(fname, res_xy, res_z, make_2d);

  // Load new map
  boost::filesystem::path bfp(fname);
  std::string parentpath(bfp.parent_path().string());
  std::string filestem(bfp.stem().string());
  erl::GridMap<int8_t> newmap;
  newmap.load(parentpath + "/" + filestem + ".yaml");

  // Publish new map
  erl_msgs::GridMap map_msg = erl::toROS(newmap);
  private_nh.param("frame_id", map_msg.header.frame_id, std::string("map"));
  ros::Time::waitForValid();
  map_msg.header.stamp = ros::Time::now();
  map_pub.publish(map_msg);

  ros::spin();

  return EXIT_SUCCESS;
}


