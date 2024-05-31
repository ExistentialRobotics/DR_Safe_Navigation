
#include <ros/duration.h>
#include <ros/ros.h>

#include "erl_astar_ros/erl_astar_2d_online.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "erl_astar_2d_ros");
  ros::NodeHandle nh("~");

  double plan_freq = 10.0;

  // set planning node frequency by reading parameter server otherwise set it to default 10
  if (!nh.getParam("plan_freq", plan_freq)) {
    ROS_ERROR("Could not find [plan_freq] parameter, use default freq %.2f", plan_freq);
  } else {
    ROS_INFO_STREAM("get plan_freq:" << plan_freq);
  }
  ros::Rate rate(plan_freq);

  ErlAstar2dOnline astar_node(nh);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  ros::spin();
}
