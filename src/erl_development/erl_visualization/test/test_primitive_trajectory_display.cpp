#include <ros/ros.h>
#include <erl_msgs/PrimitiveTrajectory.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");
  
  ros::Publisher prim_trajectory_pub = nh.advertise<erl_msgs::PrimitiveTrajectory>("primitive_trajectory", 1, true);


  erl_msgs::PrimitiveTrajectory pt;
  std_msgs::Header header;
  header.frame_id = std::string("map"); 
  pt.header = header;
  erl_msgs::Primitive prim1;
  prim1.t = 1;
  prim1.cx = {0, 1, 0, 0}; // x(t) = t
  prim1.cy = {1, 1, 1, 1}; // y(t) = 1 + t + t^2 + t^3
  prim1.cz = {0, 1, 0, 0}; // z(t) = t
  pt.primitives.push_back(prim1);
  prim_trajectory_pub.publish(pt);
  
  ROS_INFO("================ Primitive Trajectory Published! ================\n");
  
  ros::spin();
  
  return 0;
}

