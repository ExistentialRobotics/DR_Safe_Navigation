/**
 * Subscribes to an shape_msgs::Mesh Message and publishes a Marker to visualize the Mesh.
 */

#include <ros/ros.h>
#include <ros/topic.h>
#include <erl_map/grid_map.h>
#include <shape_msgs/Mesh.h>
#include <erl_rviz_markers/TriangleMsgGenerator.h>


visualization_msgs::Marker marker;
ros::Publisher pub;
std::string frame_id;
erl::TriangleMsgGenerator tmg;

void mesh_msg_callback(const shape_msgs::Mesh::ConstPtr &msg)
{
  ROS_INFO("Received Mesh Message\n");

  // Generate mesh message
  //int count = 0;
  for (const auto &triangle : msg->triangles)
  {
    geometry_msgs::Point p1 = msg->vertices[triangle.vertex_indices[0]];
    geometry_msgs::Point p2 = msg->vertices[triangle.vertex_indices[1]];
    geometry_msgs::Point p3 = msg->vertices[triangle.vertex_indices[2]];
    tmg.insert_triangle(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, p3.x, p3.y, p3.z);
  }
}

void default_pub()
{
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mesh_visualization");
  ros::NodeHandle nh("~");

  nh.param("world_frame", frame_id, std::string("map"));

  tmg = erl::TriangleMsgGenerator(frame_id); // Setup Message Generator with Correct Frame ID

  // Setup Subscriber
  ros::Subscriber subscriber =
      nh.subscribe<shape_msgs::Mesh>("mesh_msg",
                                      10,
                                      &mesh_msg_callback,
                                      ros::TransportHints().tcpNoDelay());

  ros::topic::waitForMessage<shape_msgs::Mesh>("mesh_msg", nh);
  ros::spinOnce(); // Wait for Message to Roll in

  // Publish the grid message
  pub = nh.advertise<visualization_msgs::Marker>("mesh", 1, true);
  pub.publish(*tmg.get_msg());

  ROS_INFO("Published mesh message.");
  ros::spin();

  return 0;
}


