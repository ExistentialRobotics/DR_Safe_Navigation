
#include <ros/ros.h>
#include <erl_map/grid_map.h>

#include <erl_rviz_markers/GridMsgGenerator.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_visualization");
  ros::NodeHandle nh("~");
  
  std::string frame_id;
  nh.param("world_frame", frame_id, std::string("map"));
  
  std::string grid_resource;
  nh.param("grid_resource", grid_resource, std::string("package://erl_models/models/fla_warehouse1/fla_warehouse1.yaml"));
  
  erl::GridMap<int8_t> mymap;
  mymap.load( grid_resource );
  erl::GridMsgGenerator gmg(frame_id, mymap.res());
  for( unsigned k = 0; k < mymap.map().size(); ++k) {
    if ( mymap.map()[k] > 0 )
    {
      // get the point coordinates in meters
      std::vector<double> p = mymap.cells2meters(mymap.ind2subv(k));
      gmg.insert_point3d(static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));
    }
  }
    
  // Read map: assumes rowmajor storage order and character datatype
  //erl::GridMap<char> mymap;
  //mymap.loadLegacy( grid_resource );
  //erl::map_nd mymap;
  //std::vector<char> cmap;
  //mymap.initFromYaml( cmap, grid_resource );

  // Generate grid message
  //erl::GridMsgGenerator gmg(frame_id, mymap.res());
  //for( unsigned int k = 0; k < mymap.map().size(); ++k) {
  //  if ( mymap.map()[k] == '1' )
  //  {
      // get the point coordinates in meters
  //    std::vector<double> p = mymap.cells2meters(mymap.ind2subv_rowmajor(k));
  //    gmg.insert_point3d(static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));
  //  }
  //}

  // Publish the grid message
  ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("grid", 1, true); 
  pub.publish(*gmg.get_msg());
  
  ROS_INFO("Published grid message.");
  ros::spin();
  
  return 0;
}


