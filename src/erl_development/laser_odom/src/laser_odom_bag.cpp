#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include "matcher.hpp"

using namespace std;
using namespace mrsl;

const std::string scan_topic("/scarab44/scan");
const std::string map_topic("/scarab44/map");

const std::string map_frame("/map");
const std::string base_frame("/scarab44/base_link");

struct BagTF {
  BagTF(const rosbag::Bag &bag) {
    bool interpolate = true;
    rosbag::View view(bag, rosbag::TopicQuery(string("/tf")));

    start = view.getBeginTime();
    end = view.getEndTime();
    // Cache the whole tree
    ros::Duration cache_time = (end - start) + ros::Duration(1.0);

    transformer.reset(new tf::Transformer(interpolate, cache_time));

    tf::StampedTransform st;
    BOOST_FOREACH(const rosbag::MessageInstance &m, view) {
      tf::tfMessage::Ptr msg = m.instantiate<tf::tfMessage>();
      for (size_t t = 0; t < msg->transforms.size(); ++t) {
        tf::transformStampedMsgToTF(msg->transforms.at(t), st);
        transformer->setTransform(st);
      }
    }
  }

  boost::scoped_ptr<tf::Transformer> transformer;
  ros::Time start;
  ros::Time end;
};

ros::Publisher publishMap(const rosbag::Bag &bag) {
  rosbag::View view(bag, rosbag::TopicQuery(map_topic));
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic, 1, true);
  BOOST_FOREACH(const rosbag::MessageInstance &m, view) {
    nav_msgs::OccupancyGrid::Ptr msg = m.instantiate<nav_msgs::OccupancyGrid>();
    pub.publish(msg);
  }
  return pub;
}

void add_pose(const tf::Pose &tf_pose, nav_msgs::Path *path) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = path->header.frame_id;
  pose.header.stamp = ros::Time::now();
  tf::poseTFToMsg(tf_pose, pose.pose);
  path->poses.push_back(pose);
  path->header.stamp = ros::Time::now();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_build_bag");
  if (argc != 2) {
    ROS_ERROR("usage: map_build_bag bagfile");
    return 1;
  }

  ros::NodeHandle nh, pnh("~");
  ros::Publisher pub_path_true
    = nh.advertise<nav_msgs::Path>("/path_true", 1, false);
  ros::Publisher pub_path_est
    = nh.advertise<nav_msgs::Path>("/path_est", 1, false);
  ros::Publisher pub_laser
    = nh.advertise<sensor_msgs::LaserScan>("/scan", 1, false);
  ros::Publisher pub_laser2
    = nh.advertise<sensor_msgs::LaserScan>("/scan_mapping", 1, false);
  ros::Publisher pub_newmap =
    nh.advertise<nav_msgs::OccupancyGrid>("/newmap", 1, true);
  std::string bag_path(argv[1]);
  rosbag::Bag bag(bag_path);

  ros::Publisher pub_map = publishMap(bag);

  BagTF tf_tree(bag);

  rosbag::View view(bag, rosbag::TopicQuery(scan_topic));

  tf::StampedTransform transform;
  nav_msgs::Path path_true, path_est;
  path_true.header.frame_id = map_frame;
  path_est.header.frame_id = "/newmap";

  ros::Time start_time = tf_tree.start + ros::Duration(0.1);
  ros::Time stop_time = tf_tree.end - ros::Duration(1.0);

  tf::StampedTransform offset;
  tf_tree.transformer->lookupTransform(map_frame, base_frame, start_time,
                                       offset);

  tf::TransformBroadcaster broadcaster;

  ScanMatcher mapper(ScanMatcher::Params::FromROS(pnh));
  mapper.map().setFrameId("/newmap");
  mapper.setPose(Pose2d(offset.getOrigin().x(), offset.getOrigin().y(),
                        tf::getYaw(offset.getRotation())));
  nav_msgs::OccupancyGrid new_grid;

  ros::Time time_curr, time_prev;
  int counter = 0;
  BOOST_FOREACH(const rosbag::MessageInstance &m, view) {
    if (!ros::ok()) {
      break;
    } else if (m.getTime() < start_time) {
      continue;
    } else if (m.getTime() > stop_time) {
      break;
    }
    ++counter;
    if (counter % 1 != 0) {
      continue;
    }
    time_prev = time_prev != ros::Time(0) ? time_curr : m.getTime();
    time_curr = m.getTime();

    ROS_INFO("Timestamp = %f", m.getTime().toSec());
    sensor_msgs::LaserScan::Ptr msg = m.instantiate<sensor_msgs::LaserScan>();

    // True path
    tf_tree.transformer->lookupTransform(map_frame, base_frame, time_curr,
                                         transform);
    add_pose(transform, &path_true);
    pub_path_true.publish(path_true);

    // View laser scan in true map
    sensor_msgs::LaserScan laser_now = *msg;
    laser_now.header.frame_id = base_frame;
    laser_now.header.stamp = ros::Time::now();
    pub_laser.publish(laser_now);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                   map_frame, base_frame));

    // Incorporate into map
    // tf_tree.transformer->lookupTransform(base_frame, time_prev,
    //                                      base_frame, time_curr,
    //                                      map_frame, transform);
    // Pose2d odom_true(transform);
    // mapper.addScan(odom_true, laser_now);
    // mapper.addScan(Pose2d(odom_true.x(), odom_true.y(), 0.0), laser_now);
    laser_now.header.stamp = m.getTime();
    bool map_change = mapper.addScan(Pose2d(0.0, 0.0, 0.0), laser_now);

    if (map_change) {
      pub_newmap.publish(mapper.map().occGrid());
    }

    // Offset for occ grid
    transform.setIdentity();
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                   map_frame, "/newmap"));

    // Pose of robot in occ grid
    broadcaster.sendTransform(tf::StampedTransform(mapper.pose().tf(),
                                                   ros::Time::now(),
                                                   "/newmap", "/scarabpose"));

    // Laser in occ grid
    laser_now = *msg;
    laser_now.header.frame_id = "/scarabpose";
    laser_now.header.stamp = ros::Time::now();
    pub_laser2.publish(laser_now);

    add_pose(mapper.pose().tf(), &path_est);
    pub_path_est.publish(path_est);
  }
}
