#include <ros/ros.h>
#include <erl_msgs/PointTrajectory.h>
#include <geometry_msgs/Point.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");
  
  ros::Publisher point_trajectory_pub = nh.advertise<erl_msgs::PointTrajectory>("point_trajectory", 1, true);


  erl_msgs::PointTrajectory pt;
  std_msgs::Header header;
  header.frame_id = std::string("map"); 
  pt.header = header;
  pt.name = "traj1";
  geometry_msgs::Point pt1, pt2;
  pt1.x = 12.5, pt1.y = 1.4, pt1.z = 0.0;
  pt2.x = 6.4, pt2.y = 16.6, pt2.z = 0.0;  
  pt.points.push_back(pt1), pt.points.push_back(pt2); 
  point_trajectory_pub.publish(pt);
  
  printf("================ Point Trajectory Published! ================");
  
  ros::spin();
  
  return 0;
}

