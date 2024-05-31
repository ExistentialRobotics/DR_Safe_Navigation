/**
 * Subscribes to an erl::LandMarkMap Message and publishes a Marker message to visualize the message.
 */

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <ros/ros.h>
#include <ros/topic.h>
#include <erl_rviz_markers/GridMsgGenerator.h>
#include <erl_msgs/LandmarkMap.h>

erl_msgs::LandmarkMap data;
visualization_msgs::Marker marker;
ros::Publisher pub;

void grid_msg_callback(const erl_msgs::LandmarkMap::ConstPtr &msg) {
//  ROS_INFO("Received LandmarkMap Message with %u landmarks\n", msg->n_landmarks);
  data = *msg;
}

void default_pub() {
  marker.header.stamp = ros::Time();
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  pub.publish(marker);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "landmark_visualization");
  ros::NodeHandle nh("~");

  std::string frame_id;
  nh.param("world_frame", frame_id, std::string("map"));


  // Setup Subscriber
  ros::Subscriber subscriber =
      nh.subscribe<erl_msgs::LandmarkMap>("landmark_msg",
                                          10,
                                          &grid_msg_callback,
                                          ros::TransportHints().tcpNoDelay());

  ros::topic::waitForMessage<erl_msgs::LandmarkMap>("landmark_msg", nh);
  ros::spinOnce(); // Wait for Message to Roll in


  std::vector<double> res = {1.0, 1.0, 1.0};    // Set Scale:
  // Make an RGB(1, 0, 0, 1) = Red list of Spheres.
  erl::GridMsgGenerator gmg(frame_id, res, visualization_msgs::Marker::SPHERE_LIST, 1.0, 0.0, 0.0, 1.0);

  for (unsigned k = 0; k < data.n_landmarks; ++k) {
    std::vector<double> p = {data.landmark_data[k * data.landmark_dim],
                             data.landmark_data[k * data.landmark_dim + 1],
                             data.landmark_data[k * data.landmark_dim + 2]};
    // get the point coordinates in meters
    gmg.insert_point3d(static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));

  }
  // Publish the grid message
  pub = nh.advertise<visualization_msgs::MarkerArray>("landmarks", 1, true);
  pub.publish(*gmg.get_msg());

  ROS_INFO("Published grid message.");
  ros::spin();

  return 0;
}
