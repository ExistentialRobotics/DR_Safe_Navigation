#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <topic_tools/shape_shifter.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

std::string child_frame_id;
std::string tf_prefix;
geometry_msgs::TransformStamped mapTransform;
ros::Publisher pub;

/**
This Node listens to an odometry topic in the global frame, and publishes the correct TF
 and odometry topic in the local frame.
**/

void process(const geometry_msgs::Pose &pose_in, const std_msgs::Header& header, std::string child_frame_id) {
  static tf2_ros::TransformBroadcaster odom_br;
  geometry_msgs::TransformStamped transformStamped;

  // Output the mapTransform
  ROS_WARN_ONCE("odom_to_tf: Receiving TF from %s/map to map: X: %f, Y: %f, Z: %f",
    tf_prefix.c_str(),
    mapTransform.transform.translation.x,
    mapTransform.transform.translation.y,
    mapTransform.transform.translation.z);

  // Apply Transform. (Input = pose, Output = pose_out, Transform = mapTransform)
  geometry_msgs::Pose pose_out;
  tf2::doTransform(pose_in, pose_out, mapTransform);

  // Publish TF
  transformStamped.header = header;
  transformStamped.header.frame_id = tf_prefix + "/odom";
  transformStamped.child_frame_id = tf_prefix + "/" + child_frame_id;
  transformStamped.transform.translation.x = pose_out.position.x;
  transformStamped.transform.translation.y = pose_out.position.y;
  transformStamped.transform.translation.z = pose_out.position.z;
  transformStamped.transform.rotation.x = pose_out.orientation.x;
  transformStamped.transform.rotation.y = pose_out.orientation.y;
  transformStamped.transform.rotation.z = pose_out.orientation.z;
  transformStamped.transform.rotation.w = pose_out.orientation.w;

  odom_br.sendTransform(transformStamped);
  ROS_WARN_ONCE("publish tf from [%s] to [%s]",
                transformStamped.header.frame_id.c_str(),
                transformStamped.child_frame_id.c_str());

  // Publish Ground Truth Odometry
  nav_msgs::Odometry gt_odom;
  gt_odom.header.frame_id = tf_prefix + "/odom";
  gt_odom.child_frame_id = tf_prefix + "/" + child_frame_id;
  gt_odom.pose.pose.position.x = pose_out.position.x;
  gt_odom.pose.pose.position.y = pose_out.position.y;
  gt_odom.pose.pose.position.z = pose_out.position.z;
  gt_odom.pose.pose.orientation.x = pose_out.orientation.x;
  gt_odom.pose.pose.orientation.y = pose_out.orientation.y;
  gt_odom.pose.pose.orientation.z = pose_out.orientation.z;
  gt_odom.pose.pose.orientation.w = pose_out.orientation.w;

  pub.publish(gt_odom);
}

void msgCallback(const topic_tools::ShapeShifter::ConstPtr &msg) {
  if(msg->getDataType() == "nav_msgs/Odometry") {
    auto odom = msg->instantiate<nav_msgs::Odometry>();
    process(odom->pose.pose, odom->header, odom->child_frame_id);
  }
  else if(msg->getDataType() == "geometry_msgs/PoseStamped") {
    auto pose = msg->instantiate<geometry_msgs::PoseStamped>();
    process(pose->pose, pose->header, child_frame_id);
  }
  else
    ROS_WARN_ONCE("Unrecognized msg type! [%s]", msg->getDataType().c_str());
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "odom_to_tf");
  ros::NodeHandle nh("~");

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  nh.param("child_frame_id", child_frame_id, std::string("null_frame"));
  nh.param("tf_prefix", tf_prefix, std::string(""));

  ros::Subscriber sub = nh.subscribe("input", 10, msgCallback,
                                     ros::TransportHints().tcpNoDelay());

  pub = nh.advertise<nav_msgs::Odometry>("output", 10);

  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    try {
      // Get inverse from ns/map to map
      mapTransform = tfBuffer.lookupTransform(tf_prefix+"/map","map",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};
