// test_depth_sim cube.yaml cube.off

#include <depth_sim.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_depth_sim");
  ros::Time::init();
  erl::LaserScannerSim lss( 0.1, 30.0, -2.35619, 2.35619, 1081, "laser", 40.0 );
  erl::CloudScannerSim css( 0.1, 30.0, -2.35619, 2.35619, -0.2618, 0.2618, 1081, 121, "cloud");
  erl::DepthScannerSim dss( 480, 640, 0.1, 30.0, 1.0, 1.0, 240.0, 320.0, "camera" );
  
  
  erl::GridMap<int8_t> gridmap;
  if( argc < 1 || !gridmap.load(argv[1]) )
    return EXIT_FAILURE;
  
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  double noise_sd = 0.0;
  
  sensor_msgs::LaserScan ls = lss.getScan(gridmap, R, p, noise_sd );
  sensor_msgs::PointCloud2 pc = css.getCloud(gridmap, R, p, noise_sd );
  sensor_msgs::Image im = dss.getDepth(gridmap, R, p, noise_sd );
  
  std::cout << "Generated laser scan with " << ls.ranges.size() << " beams." << std::endl;
  std::cout << "Generated cloud of size " << pc.width << " x " << pc.height << "." << std::endl;
  std::cout << "Generated image of size " << im.width << " x " << im.height << "." << std::endl;
  
  erl::MeshMap meshmap;
  if( argc < 2 || !meshmap.loadOFF(argv[2]) )
    return EXIT_FAILURE;
  
  sensor_msgs::LaserScan ls2 = lss.getScan(meshmap, R, p, noise_sd );
  sensor_msgs::PointCloud2 pc2 = css.getCloud(meshmap, R, p, noise_sd );
  sensor_msgs::Image im2 = dss.getDepth(meshmap, R, p, noise_sd );
  
  std::cout << "Generated laser scan with " << ls2.ranges.size() << " beams." << std::endl;
  std::cout << "Generated cloud of size " << pc2.width << " x " << pc2.height << "." << std::endl;
  std::cout << "Generated image of size " << im2.width << " x " << im2.height << "." << std::endl;

  // Publish Messages
  ros::NodeHandle nh;
  ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 10, 1);
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 10, 1);
  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("image", 10, 1);
  ls.header.frame_id = "map";
  pc.header.frame_id = "map";
  im.header.frame_id = "map";
  laser_pub.publish(ls);
  cloud_pub.publish(pc);
  image_pub.publish(im);

  ros::spin();
  return EXIT_SUCCESS;  
}


