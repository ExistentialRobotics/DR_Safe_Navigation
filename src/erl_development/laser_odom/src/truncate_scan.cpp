#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pubScan;

void laser_callback(const sensor_msgs::LaserScan &scan) {
  sensor_msgs::LaserScan scan1(scan);
  for (int k = 0; k < scan1.ranges.size(); ++k)
    if (scan1.ranges[k] > scan.range_max) scan1.ranges[k] = scan.range_max;
  pubScan.publish(scan1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "truncate_scan");
  ros::NodeHandle n("~");
  ros::Subscriber sub1 = n.subscribe("scan_in", 10, laser_callback, ros::TransportHints().tcpNoDelay());
  pubScan = n.advertise<sensor_msgs::LaserScan>("scan_out", 10);
  ros::spin();
  return 0;
}
