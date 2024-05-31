/**
 * Subscribes to an erl::GridMap Message and publishes a Marker message to visualize the message.
 */

#include <ros/ros.h>
#include <ros/topic.h>
#include <erl_map/grid_map.h>
#include <erl_msgs/GridMap.h>
#include <erl_conversions/erl_msg_utils.h>
#include <erl_rviz_markers/GridMsgGenerator.h>


visualization_msgs::Marker marker;
ros::Publisher pub;
std::string frame_id;


void grid_msg_callback(const erl_msgs::GridMap::ConstPtr &msg)
{
  ROS_INFO("Received Grid Message\n");
  erl::GridMap<int8_t> mymap = erl::fromROS(*msg, true); // Store the Map Bounds
  if (msg->header.frame_id != "")
    frame_id = msg->header.frame_id;
  // Generate grid message
  erl::GridMsgGenerator gmg(frame_id, mymap.res());
  for( unsigned k = 0; k < mymap.totalSize(); ++k) {
    if ( mymap.map()[k] > 0 )
    {
      // get the point coordinates in meters
      std::vector<double> p = mymap.cells2meters(mymap.ind2subv(k));
      if (p.size() > 2) // Check that there is a valid Z coordinate first.
        gmg.insert_point3d(static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));
      else
        gmg.insert_point3d(static_cast<float>(p[0]), static_cast<float>(p[1]), 0.0);
    }
  }
  // Publish the grid message
  pub.publish(*gmg.get_msg());
  ROS_INFO("Published grid message.");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_visualization");
  ros::NodeHandle nh("~");

  nh.param("world_frame", frame_id, std::string("map"));


  // Setup Subscriber
  ros::Subscriber subscriber =
      nh.subscribe<erl_msgs::GridMap>("grid_msg",
                                      10,
                                      &grid_msg_callback,
                                      ros::TransportHints().tcpNoDelay());
  // Setup Publisher
  pub = nh.advertise<visualization_msgs::MarkerArray>("grid", 1, true);

  ros::topic::waitForMessage<erl_msgs::GridMap>("grid_msg", nh);
  ros::spin();

  return 0;
}


