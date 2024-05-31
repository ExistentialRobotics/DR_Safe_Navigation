
#include <dd_control/DDControl.h>
#include <ros/ros.h>

DDControl::DDControl() {}


void DDControl::setPosition(const Eigen::Vector3d &position)
{
  pos_ = position;
}

void DDControl::setVelocity(const Eigen::Vector3d &velocity)
{
  vel_ = velocity;
}

double clamp(double x, double lower, double upper)
{
  return std::min(upper, std::max(x, lower));
}

double wrap_angle(double theta)
{
  theta = fmod(theta + M_PI, 2 * M_PI);

  return theta - M_PI;
}
void DDControl::calculateControl(double des_x, double des_y, double des_th,
                                  double kp, double ka, double kb, double vmax, double wmax)
{
  // Do Math here.
  // See: https://cs.gmu.edu/~kosecka/cs485/lec04-control.pdf
  // Compute alternate coordinates, rho, alpha, beta.
  double dx = des_x - pos_[0];
  double dy = des_y - pos_[1];
  double rho = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
  double alpha = wrap_angle(std::atan2(dy, dx) - yaw_);
  double beta = wrap_angle(-des_th- alpha);

  // Apply Gains and Compute Control Inputs

  // ROS_INFO_THROTTLE(2.0, "Initial state: %f %f %f", pos_[0], pos_[1], yaw_);
  // ROS_INFO_THROTTLE(2.0, "Alt state: %f %f%f %f %f", dx, dy, rho, alpha, beta);
  v = kp * rho;
  w = ka * alpha + kb * beta;
  ROS_INFO_THROTTLE(2, "Preclip v: %f w: %f", v, w);
  // Clip Inputs
  v = clamp(v, 0, vmax);
  w = clamp(w, -wmax, wmax);
}

bool DDControl::checkReachedGoal(const Eigen::Vector3d &des_pos, double des_yaw) {
  bool result = false;
  double dx = pos_[0] - des_pos[0];
  double dy = pos_[1] - des_pos[1];
  double dyaw = yaw_ - des_yaw;
  // ROS_INFO("DX %f DY %f Dyaw %f Rho %f", dx, dy, dyaw, rho);
  if (std::abs(dyaw) < M_PI / 6 && std::hypot(dx, dy) < 0.3)
    result= true;
  return result;

}

void DDControl::setHeading(double heading) {
  yaw_ = heading;
}
double DDControl::getLinearVelocity() const {
  return v;
}
double DDControl::getAngularVelocity() const {
  return w;
}


