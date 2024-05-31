/**
 * Subscribes to an erl::LandMarkMap Message and publishes a Marker message to visualize the message.
 */

#include <ros/ros.h>
#include <ros/topic.h>
#include <erl_conversions/erl_msg_utils.h>
#include <erl_msgs/MultiTargetBelief.h>
#include <visualization_msgs/MarkerArray.h>
#include <erl_utilities/erl_geom_utils.h>
#include <Eigen/Eigenvalues>

erl_msgs::MultiTargetBelief data;
visualization_msgs::Marker marker;
std::string frame_id;
ros::Publisher pub;

void belief_msg_callback(const erl_msgs::MultiTargetBelief::ConstPtr &msg) {
  data = *msg;
}

/**
 * Construct a basic marker at the origin with identity orientation.
 * @param r Red value.
 * @param g Green value.
 * @param b Blue value.
 * @param a Alpha value
 * @return The basic marker.
 */
visualization_msgs::Marker defaultMarker(double r= 0.0, double g=1.0, double b=1.0, double a=1) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  marker.type = 2;
  return marker;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "multi_target_belief_visualization");
  ros::NodeHandle nh("~");
  ros::Rate r(10);

  nh.param("world_frame", frame_id, std::string("map"));


  // Setup Subscriber
  ros::Subscriber subscriber =
      nh.subscribe<erl_msgs::MultiTargetBelief>("belief_msg",
                                          10,
                                          &belief_msg_callback,
                                          ros::TransportHints().tcpNoDelay());
  pub = nh.advertise<visualization_msgs::MarkerArray>("belief_vis", 1, true);

  ros::topic::waitForMessage<erl_msgs::MultiTargetBelief>("belief_msg", nh);
  ros::spinOnce(); // Wait for Message to Roll in


  while (ros::ok()) {
    visualization_msgs::MarkerArray msg;

    int id = 0;
    for (const auto &pose : data.poses) {
      visualization_msgs::Marker marker = defaultMarker();
      marker.id = id++;
      // Set Pose
      marker.pose.position.x = pose.pose.position.x;
      marker.pose.position.y = pose.pose.position.y;
      marker.pose.position.z = pose.pose.position.z;

      // Now Compute Scale and Orientation via the Covariance Matrix
      // Generate Covariance Matrix
      Eigen::Matrix3d mat;
      mat << pose.covariance[0], pose.covariance[1], pose.covariance[2],
          pose.covariance[6], pose.covariance[7], pose.covariance[8],
          pose.covariance[12], pose.covariance[13], pose.covariance[14];
      Eigen::EigenSolver<Eigen::MatrixXd> es(mat);

      Eigen::Matrix3d rot = es.eigenvectors().real().transpose(); // TODO Verify this is correct.

      auto quat = erl::rot2quat(rot);

      marker.pose.orientation.w = quat[0];
      marker.pose.orientation.x = quat[1];
      marker.pose.orientation.y = quat[2];
      marker.pose.orientation.z = quat[3];

      marker.scale.x = std::max(0.1, std::sqrt(es.eigenvalues()(0).real()));
      marker.scale.y = std::max(0.1, std::sqrt(es.eigenvalues()(1).real()));
      marker.scale.z = std::max(0.1, std::sqrt(es.eigenvalues()(2).real()));

      msg.markers.push_back(marker);
    }
    // Publish the grid message
    pub.publish(msg);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}


