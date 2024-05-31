#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

visualization_msgs::Marker marker;
ros::Publisher pub;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  marker.header.frame_id = msg->header.frame_id;
  marker.header.stamp = ros::Time();
  marker.pose.position.x = msg->pose.pose.position.x;
  marker.pose.position.y = msg->pose.pose.position.y;
  marker.pose.position.z = msg->pose.pose.position.z;
  marker.pose.orientation.x = msg->pose.pose.orientation.x;
  marker.pose.orientation.y = msg->pose.pose.orientation.y;
  marker.pose.orientation.z = msg->pose.pose.orientation.z;
  marker.pose.orientation.w = msg->pose.pose.orientation.w;  
  pub.publish(marker);
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
  
  nh.param("world_frame", marker.header.frame_id, std::string("map"));
  marker.mesh_use_embedded_materials = true;
  marker.ns = "mesh_visualization";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  nh.param("mesh_resource", marker.mesh_resource, std::string("package://nx_models/models/hokuyo/meshes/hokuyo.dae"));
  nh.param("scale", marker.scale.x, 1.0);
  marker.scale.y = marker.scale.x;
  marker.scale.z = marker.scale.x;
  
  bool is_static;
  nh.param("is_static", is_static, true);
  ros::Subscriber odom_sub; // Declare odom_sub here to avoid going out of scope in else statement...
  if( is_static )
  {
    pub = nh.advertise<visualization_msgs::Marker>("mesh", 1, true);
    default_pub();
  }else{
    pub = nh.advertise<visualization_msgs::Marker>("mesh", 10);
    odom_sub = nh.subscribe("odom", 10, &odom_callback, ros::TransportHints().tcpNoDelay());
  }

  ros::spin();
  return 0;
}
