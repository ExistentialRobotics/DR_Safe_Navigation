#include "matcher.hpp"

#include <queue>

#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace Eigen;
using namespace mrsl;

GridMap::GridMap(double ox, double width, double oy, double height,
                 double meters_per_pixel)
  : origin_x_(ox), origin_y_(oy), meters_per_pixel_(meters_per_pixel) {
  width_ = ceil(width / meters_per_pixel_);
  height_ = ceil(height / meters_per_pixel_);
  ros_grid_.reset(new nav_msgs::OccupancyGrid);
  ros_grid_->header.frame_id = "/map";
  ros_grid_->data.resize(width_ * height_);
  ros_grid_->info.resolution = meters_per_pixel_;
  ros_grid_->info.width = width_;
  ros_grid_->info.height = height_;
  ros_grid_->info.origin.position.x = origin_x_;
  ros_grid_->info.origin.position.y = origin_y_;
  ros_grid_->info.origin.orientation.w = 1.0;

  grid_ = new uint8_t[width_ * height_];
}

void GridMap::scores3D(const Pose2d &pose,
                       const RowMatrix2d &points,
                       int delta_xi, int num_x,
                       int delta_yi, int num_y,
                       double delta_t, int num_t, double inc_t,
                       vector<ArrayXXi> *scores) const {
  // Iterate over theta and get scores for each point
  scores->resize(num_t);
  for (int t = 0; t < num_t; ++t) {
    double theta = delta_t + t * inc_t;
    scores2D(pose, points, delta_xi, num_x, delta_yi, num_y, theta,
             &scores->at(t));
  }
}

void GridMap::scores2D(const Pose2d &pose,
                       const RowMatrix2d &points,
                       int delta_xi, int num_x,
                       int delta_yi, int num_y,
                       double theta, ArrayXXi *scores_ptr) const {
  ArrayXXi &scores = *scores_ptr;
  scores.resize(num_x, num_y);
  scores.setZero();

  double ct = cos(theta + pose.t()), st = sin(theta + pose.t());
  for (int i = 0; i < points.cols(); ++i) {
    // Rotate point, convert it to pixel coordinates and *then* translate it.
    // If you translate in world coordinates rounding can do weird things.
    const Eigen::Vector2d &point = points.col(i);
    double x = ct * point(0) - st * point(1) + pose.x();
    double y = st * point(0) + ct * point(1) + pose.y();

    int xi0, yi0;
    // Get grid coordinates of point
    getSubscript(x, y, &xi0, &yi0);

    // Shift by initial offset
    xi0 += delta_xi;
    yi0 += delta_yi;

    // Iterate over translation of point and update scores
    for (int dyi = 0; dyi < num_y; ++dyi) {
      int yi = yi0 + dyi;
      for (int dxi = 0; dxi < num_x; ++dxi) {
        int xi = xi0 + dxi;
        int ll = static_cast<int>(get(xi, yi));
        scores(dxi, dyi) += ll;
      }
    }
  }
}

void mrsl::projectScan(const Pose2d &pose, const sensor_msgs::LaserScan &scan,
                       int subsample, RowMatrix2d *points) {
  vector<Point2d> point_vec;
  double theta = scan.angle_min;
  int added = 0;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];
    if (scan.range_min <= range && range <= scan.range_max) {
      ++added;
      if (added % subsample == 0) {
        double x = cos(theta) * scan.ranges[i];
        double y = sin(theta) * scan.ranges[i];
        point_vec.push_back(pose.transform_from(Point2d(x, y)));
      }
    }
    theta += scan.angle_increment;
  }
  if (abs(theta - scan.angle_increment - scan.angle_max) > 1e-5) {
    ROS_ERROR("Bad angle: %f %f", theta-scan.angle_increment, scan.angle_max);
    ROS_BREAK();
  }

  // Copy to output
  points->resize(2, point_vec.size());
  for (size_t i = 0; i < point_vec.size(); ++i) {
    points->col(i) = point_vec.at(i).vector();
  }
}

void mrsl::transformPoints(const Pose2d& pose, const RowMatrix2d &local,
                           RowMatrix2d *points) {
  double ct = cos(pose.t()), st = sin(pose.t());
  points->resize(2, local.cols());
  for (int i = 0; i < local.cols(); ++i) {
    Point2d local_point = Point2d(local.col(i));
    points->col(i) = pose.transform_from(local_point).vector();
  }
}

ScanMatcher::ScanMatcher(const Params &p)
  : p_(p), map_(NULL), have_scan_(false) {
  p_.align();
  map_.reset(new GridMap(-40, 80.0, -40, 80.0, p_.grid_res));
  map_->fill(0);
  pub_scan_ = nh_.advertise<sensor_msgs::PointCloud2>("laser_cloud", 1, false);
}

ScanMatcher::~ScanMatcher() {
}

bool ScanMatcher::addScan(const Pose2d &odom,
                          const sensor_msgs::LaserScan &scan)
{
  // Add odometry
  pose_ = pose_.oplus(odom);
  // ROS_INFO_STREAM("Pose: " << pose_);

  // Correct pose
  if (have_scan_)
  {  
    //ros::Time start = ros::Time::now();
    Gaussian3d update_est = matchScan(pose_, scan);
    //ROS_INFO("  Match time: %.3f", (ros::Time::now() - start).toSec());

    Eigen::Vector3d update = update_est.mean();
    //ROS_INFO_STREAM("Update: " << update.transpose());
     
    if (fabs(update(0)) == p_.range_x || fabs(update(1)) == p_.range_y ||
        fabs(update(2)) == p_.range_t) {
      ROS_WARN("Update (%.5f, %.5f, %.5f) is at max of search window;\n"
               "drive slower or make window bigger!",
               update(0), update(1), update(2));
    }

    pose_.setX(pose_.x() +  update(0));
    pose_.setY(pose_.y() +  update(1));
    pose_.setT(pose_.t() +  update(2));
  } else {
    last_decay_ = scan.header.stamp;
  }
  

  bool changed = false;
  if (scan.header.stamp - last_decay_ > ros::Duration(p_.decay_duration)) {
    map_->decay(p_.decay_step);
    last_decay_ = scan.header.stamp;
    changed = true;
  }


  // If map has no scans, robot has moved, or nothing has been incorporated in
  // a while, update the map
  Pose2d traveled = pose_.ominus(last_scan_pose_);
  bool moved_linear = hypot(traveled.x(), traveled.y()) > p_.travel_distance;
  bool moved_angular = traveled.t() > p_.travel_angle;
  bool add = scan.header.stamp - last_add_ > ros::Duration(0.2);
  if (!have_scan_ || moved_linear || moved_angular || add) {
    RowMatrix2d points;
    projectScan(pose_, scan, 1, &points);

    // ros::Time start = ros::Time::now();
    updateMap(points);
    // ROS_INFO("  Update time: %.3f", (ros::Time::now() - start).toSec());
    last_scan_pose_ = pose_;
    have_scan_ = true;
    last_add_ = scan.header.stamp;
    return true;
  } else {
    return changed;
  }

}

// update map; points are in map frame
void ScanMatcher::updateMap(const RowMatrix2d &points) {
  // TODO: Lookup table
  // Update likelihood of neighboring points in 3 sigma radius around point
  const int max_offset = ceil(p_.sensor_sd / p_.grid_res) * 3;
  // Don't normalize by standard deviation; all values are scaled equally and
  // this results in a better spread
  const double normalizer = 1.0;
  const double exp_factor = (p_.grid_res * p_.grid_res) /
    (2 * p_.sensor_sd * p_.sensor_sd);

  for (int i = 0; i < points.cols(); ++i) {
    int xi, yi;
    map_->getSubscript(points(0, i), points(1, i), &xi, &yi);
    for (int delta_y = -max_offset; delta_y <= max_offset; ++delta_y) {
      for (int delta_x = -max_offset; delta_x <= max_offset; ++delta_x) {
        int dist = delta_x * delta_x + delta_y * delta_y;
        double prob = normalizer * exp(-dist * exp_factor);
        uint8_t val = static_cast<uint8_t>(prob * 255.0);
        map_->setMax(xi + delta_x, yi + delta_y, val);
      }
    }
  }
}

Gaussian3d ScanMatcher::matchScan(const Pose2d &pose,
                                  const sensor_msgs::LaserScan &scan)
{
  RowMatrix2d points;
  // Project laser scan points into local frame
  Pose2d identity(0.0, 0.0, 0.0);
  projectScan(identity, scan, p_.subsample, &points);
  
  // Filter nearby points
  double resolution = 0.01;
  sensor_msgs::PointCloud2 ros_input;
  laser_geometry::LaserProjection projector;
  projector.projectLaser(scan, ros_input);
  
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(ros_input, pcl_pc);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc, *cloud);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(resolution, resolution, 1.0);
  pcl::PointCloud<pcl::PointXYZ> pcl_filtered;
  sor.filter(pcl_filtered);
  pub_scan_.publish(*cloud);
  
  RowMatrix2d points_filt;
  points_filt.resize(2, pcl_filtered.points.size());
  for (int i = 0; i < points_filt.cols(); ++i) {
    points_filt(0, i) = pcl_filtered.points.at(i).x;
    points_filt(1, i) = pcl_filtered.points.at(i).y;
  }
  
  return match(points_filt);
}

Gaussian3d ScanMatcher::match(const RowMatrix2d &points)
{
  int sx = round(p_.range_x / p_.grid_res);
  int sy = round(p_.range_y / p_.grid_res);
  int num_x = 2 * sx + 1;
  int num_y = 2 * sy + 1;
  int num_t = 2 * round(p_.range_t / p_.inc_t) + 1;

  vector<ArrayXXi> scores;
  map_->scores3D(pose_,
                 points, -sx, num_x, -sy, num_y,
                 -p_.range_t, num_t, p_.inc_t, &scores);

  // Process scores to get best guess
  Vector3i inds = Vector3i::Zero();
  int max_likeli = std::numeric_limits<int>::min();
  for (size_t ti = 0; ti < scores.size(); ++ti) {
    ArrayXXi &dxdy_scores = scores.at(ti);
    for (int xi = 0; xi < dxdy_scores.rows(); ++xi) {
      for (int yi = 0; yi < dxdy_scores.cols(); ++yi) {
        int score = dxdy_scores(xi, yi);
        if (score > max_likeli) {
          inds(0) = xi;
          inds(1) = yi;
          inds(2) = ti;
          max_likeli = score;
        }
      }
    }
  }

  Vector3d transform(-p_.range_x + inds(0) * p_.grid_res,
                     -p_.range_y + inds(1) * p_.grid_res,
                     -p_.range_t + inds(2) * p_.inc_t);

  return Gaussian3d(transform, Matrix3d::Identity());
}
