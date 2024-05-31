#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "matcher.hpp"

using std::string;

class LaserOdomNode {
public:
  LaserOdomNode()
    : pnh_("~"), matcher_(mrsl::ScanMatcher::Params::FromROS(pnh_)),
      have_pose_(false)
  {
    string laser_base_frame;
    pnh_.param("odom_frame", odom_frame_, string("odom_laser"));
    pnh_.param("base_frame", base_frame_, string("base_link"));
    pnh_.param("laser_base_frame", laser_base_frame, string("base_link"));
    pnh_.param("laser_frame", laser_frame_, string("laser"));
    pnh_.param("debug", debug_, false);

    sscan_ = nh_.subscribe("scan", 5, &LaserOdomNode::laserCb, this);
    sub_motor_odom_ = nh_.subscribe("odom_motor", 5, &LaserOdomNode::motorOdomCb, this);

    podom_ = nh_.advertise<nav_msgs::Odometry>("odom_laser", 5, false);
    if (debug_) {
      pmap_ = nh_.advertise<nav_msgs::OccupancyGrid>("map_local", 1, true);
    }

    // Get offset from robot base to laser
    ros::Time now = ros::Time::now();
    ros::Duration(0.1).sleep();
    try {
      tf_listen_.lookupTransform(laser_base_frame, laser_frame_, now, laser_tform_);
    } catch (const tf::LookupException &exception) {
      ROS_INFO("Couldn't find transfrom from %s to %s, using identity",
               laser_base_frame.c_str(), laser_frame_.c_str());
      laser_tform_.setRotation(tf::createQuaternionFromYaw(0.0));
    }

    odom_.header.frame_id = odom_frame_;
    odom_.child_frame_id = base_frame_;

    matcher_.map().setFrameId(odom_frame_);
    tf::Pose pose;
    tf::poseMsgToTF(matcher_.map().origin(), pose);
    tf::poseTFToMsg(pose * laser_tform_, matcher_.map().origin());
  }

  // Get 3D pose of laser in local map
  tf::Pose laserPose()
  {
    return laser_tform_ * matcher_.pose().tf();
  }

  void motorOdomCb(const nav_msgs::Odometry &msg)
  {
    motor_odom_ = msg;
  }

  void laserCb(const sensor_msgs::LaserScan &scan) {
    
    // No odom estimate
    bool map_change = matcher_.addScan(Pose2d(0.0, 0.0, 0.0), scan);
    if (debug_) {
      if (map_change) {
        pmap_.publish(matcher_.map().occGrid());
      }
    }

    odom_.header.stamp = scan.header.stamp;
    tf::poseTFToMsg(laserPose(), odom_.pose.pose);
    // Get velocity estimate
    if (!have_pose_) {
      last_pose_ = matcher_.pose();
      have_pose_ = true;
    } else {
      double dt = (scan.header.stamp - last_pose_time_).toSec();
      if (dt > 0.2) {
        Pose2d change = matcher_.pose().ominus(last_pose_);
        odom_.twist.twist.linear.x = change.x() / dt;
        odom_.twist.twist.linear.y = change.y() / dt;
        odom_.twist.twist.angular.z = change.t() / dt;
        last_pose_time_ = scan.header.stamp;
        last_pose_ = matcher_.pose();
      }
    }
    
    
    if (motor_odom_) {
      odom_.twist.twist.linear.x = motor_odom_->twist.twist.linear.x;
    } else {
      ROS_ERROR("No motor odometry");
    }

    podom_.publish(odom_);
    tf_.sendTransform(
      tf::StampedTransform(laserPose(), scan.header.stamp,
                           odom_frame_, base_frame_));
  }

private:
  ros::NodeHandle nh_, pnh_;
  std::string odom_frame_, base_frame_, laser_frame_;
  tf::StampedTransform laser_tform_;
  mrsl::ScanMatcher matcher_;
  ros::Subscriber sscan_, sub_motor_odom_;
  ros::Publisher podom_, pmap_;
  bool have_pose_;
  Pose2d last_pose_;
  ros::Time last_pose_time_;
  bool debug_;
  tf::TransformBroadcaster tf_;
  tf::TransformListener tf_listen_;
  nav_msgs::Odometry odom_;
  boost::optional<nav_msgs::Odometry> motor_odom_;
};



int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_odom");

  LaserOdomNode lon;
  
  ros::spin();
}





















