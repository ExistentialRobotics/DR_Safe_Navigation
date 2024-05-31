#ifndef DD_CONTROL_H
#define DD_CONTROL_H

#include <Eigen/Geometry>

class DDControl
{
 public:
  DDControl();

  void setPosition(const Eigen::Vector3d &position);

  void setVelocity(const Eigen::Vector3d &velocity);

  void setHeading(double heading);

  void calculateControl(double des_x, double des_y, double des_th,
                        double kp, double ka, double kb, double vmax, double wmax);

  bool checkReachedGoal(const Eigen::Vector3d &des_pos, double des_yaw);

  double getLinearVelocity() const;

  double getAngularVelocity() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 protected:

  // Inputs for the controller
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  double yaw_;

  // Output
  double v; // Linear Velocity
  double w; // Angular Velocity

};

class PowerDiagramControl: public DDControl
{
 public:
    void calculateControl(double des_x, double des_y, double ka,
                          double kl, double vmax, double wmax);
    bool checkReachedGoal(const Eigen::Vector3d &des_pos, double des_yaw);
};

#endif
