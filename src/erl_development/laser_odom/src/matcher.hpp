#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>

#include "Pose2d.hpp"

namespace mrsl {

template<int D>
class Gaussian {
public:
  typedef Eigen::Matrix<double, D, 1> VectorD;
  typedef Eigen::Matrix<double, D, D> MatrixD;

  Gaussian(){}
  
  Gaussian(const VectorD &mean, const MatrixD &cov) : mean_(mean), cov_(cov) {
  }

  const MatrixD& cov() const { return cov_; }
  const VectorD& mean() const { return mean_; }

private:
  Eigen::Matrix<double, D, 1> mean_;
  Eigen::Matrix<double, D, D> cov_;
};

typedef Gaussian<3> Gaussian3d;

typedef Eigen::Matrix<double, 2, Eigen::Dynamic> RowMatrix2d;

class GridMap {
public:
  GridMap(double origin_x, double width, double origin_y, double height,
          double meters_per_pixel);

  ~GridMap() {
    delete[] grid_;
  }

  double metersPerPixel() const { return meters_per_pixel_; }
  int width() const { return width_; }
  int height() const { return height_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }

  void getCoord(int xi, int yi, double *x, double *y) const {
    *x = (xi + 0.5) * meters_per_pixel_ + origin_x_;
    *y = (yi + 0.5) * meters_per_pixel_ + origin_y_;
  }

  void getSubscript(double x, double y, int *xi, int *yi) const {
    *xi = (x - origin_x_) / meters_per_pixel_;
    *yi = (y - origin_y_) / meters_per_pixel_;
  }

  bool valid(int xi, int yi) const {
    return !(xi >= width_ || yi >= height_ || xi < 0 || yi < 0);
  }

  uint8_t get(int xi, int yi) const {
    if (valid(xi, yi)) {
      int ind = yi * width_ + xi;
      return grid_[ind];
    } else {
      return 0;
    }
  }

  void decay(int val) {
    for (int i = 0; i < width_ * height_; ++i) {
      if (grid_[i] > 0) {
        if (grid_[i] >= val) {
          grid_[i] -= val;
        } else {
          grid_[i] = 0;
        }
        ros_grid_->data[i] = 100 - static_cast<int>(grid_[i] / 2.55);
      }
    }
  }


  void setMax(int xi, int yi, uint8_t val) {
    if (valid(xi, yi)) {
      int ind = yi * width_ + xi;
      grid_[ind] = std::max(val, grid_[ind]);
      ros_grid_->data[ind] = 100 - static_cast<int>(grid_[ind] / 2.55);
    } else {
      ROS_WARN("Setting coordinates outside map");
    }
  }

  void fill(uint8_t val) {
    for (int i = 0; i < width_ * height_; ++i) {
      grid_[i] = val;
      ros_grid_->data[i] = 100 - static_cast<int>(grid_[i] / 2.55);
    }
  }

  void setFrameId(const std::string &frame) {
    ros_grid_->header.frame_id = frame;
  }

  geometry_msgs::Pose& origin() {
    return ros_grid_->info.origin;
  }

  const nav_msgs::OccupancyGrid& occGrid() const {
    return *ros_grid_;
  }

  // Get likelihood of points for various translations after a rotation.
  //
  // Points are x-shifted in pixel coordinates by (delta_xi + i) where i is
  // in the half open interval [0, num_x).  Similar for y.
  //
  // scores(xi, yi) = Likelihood points after x-shifted by (delta_xi + xi) *
  // metersPerPixel() and y-shifted by (delta_yi + yi) * metersPerPixel()
  void scores2D(const Pose2d &pose, const RowMatrix2d &points,
                int delta_xi, int num_x,
                int delta_yi, int num_y,
                double theta, Eigen::ArrayXXi *scores) const;

  // Evaluate scores2D with several different angles.  'points' should be in
  // robot's local frame, 'pose' should be an initial estimate of the robot's
  // pose in the map frame.
  //
  // scores[i] scores2d() with theta = delta_t + i * inc_t
  void scores3D(const Pose2d &pose, const RowMatrix2d &points,
                int delta_xi, int num_x, int int_y, int num_yi,
                double delta_t, int num_t, double inc_t,
                std::vector<Eigen::ArrayXXi> *scores) const;

private:
  GridMap() {};
  GridMap(const GridMap &map);
  void operator=(const GridMap&);

  // Lower left coordinate of map in meters
  double origin_x_, origin_y_;
  double meters_per_pixel_;
  // Size of grid_ in pixels
  int width_, height_;
  // Probability of occupancy; 0 means 0 probability, 255 means 1.0
  uint8_t *grid_;
  boost::scoped_ptr<nav_msgs::OccupancyGrid> ros_grid_;
};

void projectScan(const Pose2d &pose, const sensor_msgs::LaserScan &scan,
                 int subsample, RowMatrix2d *points);

void transformPoints(const Pose2d &pose, const RowMatrix2d &local,
                     RowMatrix2d *points);

// Estimate robot's position using scans + building local map
class ScanMatcher {
public:
  struct Params {
    // 0.14 rad ~= 8 deg
    // 0.0035 rad ~= 0.2 deg
    Params()
      : range_x(0.1), range_y(0.1), range_t(0.14), inc_t(0.0035),
        grid_res(0.02), sensor_sd(0.02), subsample(3),
        travel_distance(0.2), travel_angle(angles::from_degrees(2.0)),
        decay_duration(15.0), decay_step(40) {}

    static Params FromROS(ros::NodeHandle &nh) {
      Params p;
      nh.param("range_x", p.range_x, p.range_x);
      nh.param("range_y", p.range_y, p.range_y);
      nh.param("range_t", p.range_t, p.range_t);
      nh.param("inc_t", p.inc_t, p.inc_t);
      nh.param("grid_resolution", p.grid_res, p.grid_res);
      nh.param("sensor_sd", p.sensor_sd, p.sensor_sd);
      nh.param("subsample", p.subsample, p.subsample);
      nh.param("travel_distance", p.travel_distance, p.travel_distance);
      nh.param("travel_angle", p.travel_angle, p.travel_angle);
      nh.param("decay_duration", p.decay_duration, p.decay_duration);
      nh.param("decay_step", p.decay_step, p.decay_step);
      p.align();
      ROS_INFO("%s", p.string().c_str());
      return p;
    }

    std::string string() {
      char s[400];
      sprintf(s,
              "range_x: %.3f range_y: %.3f range_tt: %.3f inc_t: %.3f\n"
              "grid_resolution: %.3f sensor_sd: %0.3f subsample: %i\n"
              "travel_distance: %.3f travel_angle: %0.3f\n"
              "decay_duration: %.3f decay_step: %i",
              range_x, range_y, range_t, inc_t,
              grid_res, sensor_sd, subsample,
              travel_distance, travel_angle, decay_duration, decay_step);
      return std::string(s);
    }

    double range_x;
    double range_y;
    double range_t;
    double inc_t;
    double grid_res;
    double sensor_sd;
    int subsample;
    double travel_distance, travel_angle;
    double decay_duration;
    int decay_step;

    void align() {
      range_x = round(range_x / grid_res) * grid_res;
      range_y = round(range_y / grid_res) * grid_res;
      range_t = round(range_t / inc_t) * inc_t;
    }
  };

  ScanMatcher(const Params &p);
  ~ScanMatcher();

  const Pose2d& pose() { return pose_; }
  void setPose(const Pose2d &pose) { pose_ = pose; }

  bool addScan(const Pose2d &odom, const sensor_msgs::LaserScan &scan);
  void updateMap(const RowMatrix2d &points);

  const GridMap& map() const { return *map_; }
  GridMap& map() { return *map_; }

  Gaussian3d matchScan(const Pose2d &pose, const sensor_msgs::LaserScan &scan);
  Gaussian3d match(const RowMatrix2d &points);
private:
  Params p_;
  Pose2d last_scan_pose_; // pose of the last incorporated scan
  Pose2d pose_; // current pose of the robot
  ros::Time last_decay_, last_add_;
  bool have_scan_;
  boost::scoped_ptr<GridMap> map_;
  ros::NodeHandle nh_;
  ros::Publisher pub_scan_;
};

};
