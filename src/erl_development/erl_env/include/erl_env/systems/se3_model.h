//
// Created by brent on 10/16/20.
//


/**
 * Define the SE(3) geometry transition functions, given an input SE(3) pose, and a control input.

 Design:
 Control inputs are u in R^6. Vector6d.

 */

#ifndef SRC_ERL_DEVELOPMENT_ERL_ENV_INCLUDE_ERL_ENV_SYSTEMS_SE3_MODEL_H_
#define SRC_ERL_DEVELOPMENT_ERL_ENV_INCLUDE_ERL_ENV_SYSTEMS_SE3_MODEL_H_
#include <erl_utilities/se3_pose.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

namespace erl {

/**
 * Compute the hatMap transformation from a 3-d control input w to a 3x3 skew-symmetric matrix.
 * @param w The angular velocity input.
 * @return The resulting skew symmetric matrix.
 */
Eigen::Matrix3d hatMap(const Eigen::Vector3d &w) {
  Eigen::Matrix3d w_hat; w_hat << 0, -w[2], w[1], w[2], 0, -w[0], -w[1], w[0], 0;
  return w_hat;
}

/**
 * Compute the hatMap transformation from a 6-d control input u to a 4x4 twist matrix.
 * @param u The control input vector of linear and angular velocity.
 * @return The resulting twist matrix.
 */
Eigen::Matrix4d hatMap(const Eigen::Vector6d &u) {
  // Split u into velocity, and angular velocity.
  Eigen::Vector3d v = u.head(3);
  Eigen::Vector3d w = u.tail(3);
  Eigen::Matrix3d w_hat = hatMap(w);

  Eigen::Matrix4d u_hat = Eigen::Matrix4d::Zero(4, 4);
  u_hat.block(0, 0, 3, 3) = w_hat;
  u_hat.block(0, 3, 3, 1) = v;
  return u_hat;
}

/**
 * Compute the adjoint representation mapping a 6-d control input to a 6x6 adjoint matrix.
 * @param u The control input vector of linear and angular velocity.
 * @return The adjoint matrix.
 */
Eigen::Matrix6d adjointMap(const Eigen::Vector6d &u) {
  Eigen::Matrix6d result = Eigen::Matrix6d::Zero();

  Eigen::Vector3d v = u.head(3);
  Eigen::Vector3d w = u.tail(3);
  Eigen::Matrix3d w_hat = hatMap(w);
  Eigen::Matrix3d v_hat = hatMap(v);
  result.block(0, 0, 3, 3) = w_hat;
  result.block(0, 3, 3, 3) = v_hat;
  result.block(3, 3, 3, 3) = w_hat;
  return result;
}

/**
 * Defines the SE(3) transition function.
 * @param pose The current pose.
 * @param u The 6-D control input.
 * @return The resulting pose.
 */
SE3Pose transition(const SE3Pose& pose, const Eigen::Vector6d &u, double dt) {

  Eigen::Matrix4d u_hat = hatMap(u);
  Eigen::Matrix4d result = pose.getMatrix() * (dt*u_hat).exp();
  return SE3Pose(result);
}

} // end namespace


#endif //SRC_ERL_DEVELOPMENT_ERL_ENV_INCLUDE_ERL_ENV_SYSTEMS_SE3_MODEL_H_
