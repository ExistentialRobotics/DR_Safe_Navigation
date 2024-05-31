/**
 * Publishes an erl::GridMap Message generated from a map file.
 */

#include <ros/ros.h>
#include <erl_map/grid_map.h>
#include <erl_msgs/GridMap.h>
#include <erl_rviz_markers/GridMsgGenerator.h>
#include <erl_conversions/erl_msg_utils.h>


std::string frame_id;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_visualization");
  ros::NodeHandle nh("~");

  nh.param("world_frame", frame_id, std::string("map"));

  std::string grid_resource;
  nh.param("grid_resource", grid_resource, std::string("package://erl_models/models/pretzel/pretzel.yaml"));

  // Read map: assumes rowmajor storage order and character datatype
  erl::GridMap<char> mymap;
  mymap.loadLegacy( grid_resource );

  // Publish the grid message
  ros::Publisher pub = nh.advertise<erl_msgs::GridMap>("grid_msg", 1, true);
  erl_msgs::GridMap msg = erl::toROS(mymap);
  msg.header.frame_id = frame_id;
  pub.publish(msg);

  ROS_INFO("Published grid message.");
  ros::spin();

  return 0;
}


