
#include <dd_control/DDControl.h>
#include <ros/ros.h>

/*

PowerDiagramControl::PowerDiagramControl() {}

void PowerDiagramControl::setPosition(const Eigen::Vector3d &position)
{
  pos_ = position;
}

void PowerDiagramControl::setVelocity(const Eigen::Vector3d &velocity)
{
  vel_ = velocity;
}
*/
double clamp(double x, double lower, double upper)
{
  return std::min(upper, std::max(x, lower));
}

double wrap_angle(double theta)
{
  theta = fmod(theta + M_PI, 2 * M_PI);

  return theta - M_PI;
}
void PowerDiagramControl::calculateControl(double des_x, double des_y, double ka,
                                 double kl, double vmax, double wmax)
{
  // Power Diagram controller
  double dx = des_x - pos_[0];
  double dy = des_y - pos_[1];
  // Apply Gains and Compute Control Inputs
  v = kl*(cos(yaw_)*dx + sin(yaw_)*dy);
  w = ka*atan2(-sin(yaw_)*dx + cos(yaw_)*dy, cos(yaw_)*dx + sin(yaw_)*dy);

  ROS_INFO_THROTTLE(2, "Preclip v: %f w: %f", v, w);
  // Clip Inputs
  v = clamp(v, -vmax, vmax);
  w = clamp(w, -wmax, wmax);
}

bool PowerDiagramControl::checkReachedGoal(const Eigen::Vector3d &des_pos, double des_yaw) {
  bool result = false;
  double dx = pos_[0] - des_pos[0];
  double dy = pos_[1] - des_pos[1];
  //double dyaw = yaw_ - des_yaw;
  // ROS_INFO("DX %f DY %f Dyaw %f Rho %f", dx, dy, dyaw, rho);
  if (std::hypot(dx, dy) < 0.3)
    result= true;
  return result;

}





