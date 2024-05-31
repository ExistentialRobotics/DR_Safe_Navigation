//
// Created by brent on 11/9/20.
//

#ifndef __ENV_SE3_H_
#define __ENV_SE3_H_

#include <memory>   // std::unique_ptr
#include <utility>      // std::pair
#include <erl_env/environments/environment_metric.h>
#include <erl_utilities/se3_pose.h>
#include <erl_map/grid_map.h>
#include <erl_map/grid_map_utils.h>
#include <iostream>
#include <erl_env/systems/se3_model.h>

namespace erl {

/**
 * @brief The EnvironmentSE3 class implements the Environment specification for an SE(3) environment
 * (X, Y, Z, Roll, Pitch, Yaw) with motion primitives.
 */
template <class U>
class EnvironmentSE3 : public EnvironmentMetric<SE3Pose, U> {
 public:
  using EnvironmentMetric<SE3Pose, U>::map; // Bring Parent variables into the namespace.

  typedef MotionPrimitive<SE3Pose, Eigen::Vector6d> MPrim;
  std::vector<MPrim> mprim_;


  /*
   * @brief Constructs the EnvironmentSE3 from map parameters and list motion primitive values.
   * @param map The map resolution.
   * @param primitives The list of valid linear velocity inputs.
   */
  EnvironmentSE3(const std::shared_ptr<erl::GridMap<U>> &map,
                 const std::vector<Eigen::Vector6d> &primitives,
                 const std::vector<double> &tlist)
      : EnvironmentMetric<SE3Pose, U>(map) {

    // Init MPrim
    Eigen::Vector6d origin = Eigen::Vector6d::Zero();
    for (size_t i = 0; i < primitives.size(); i++) {
      Eigen::Vector6d state = transition(origin, primitives[i], tlist[i]).getVectorForm();
      double cost = (state.head(3)).norm();
      mprim_.push_back({{primitives[i]}, {tlist[i]}, {cost}});
    }
    std::cout <<"Initialized Primitives for SE(3) environment.\n";
  }

  /**
   * @brief Computes the successors of the state curr.
   * @param curr The current state to compute successors of.
   * @param succ The vector of successor states.
   * @param succ_cost The vector of costs of each successor.
   * @param action_idx The vector of indices of the actions leading to the successor.
   */
  inline void getSuccessors(const SE3Pose &curr,
                            std::vector<SE3Pose> &succ,
                            std::vector<double> &succ_cost,
                            std::vector<int> &action_idx) const override {
    size_t num_traj = mprim_.size();

    // Reserve space for successors
    succ.reserve(num_traj);
    succ_cost.reserve(num_traj);
    action_idx.reserve(num_traj);

    for (size_t tr = 0; tr < num_traj; ++tr) {
      // Check Micro-States
      Eigen::Vector6d u = mprim_[tr].uVec[0];
      double tau = mprim_[tr].tVec[0];
      for (double t_ = 0.05; t_ <= tau; t_ += .05) {
        // check for collisions along the microstates
        Eigen::Vector6d state = transition(curr, u, t_).getVectorForm();
        // Discard any primitives that collide.
        if (!map->inMap(state.head(3))) break;
      }
      // If we make it through collision checking, add the successor.
      succ.push_back(transition(curr, u, tau));
      action_idx.push_back(tr);
      succ_cost.push_back(mprim_[tr].cVec[0]);
    }
  }

  /**
   * @brief Computes the vector of micro-states from the current state if following the action given by action_id.
   * @param curr The current state.
   * @param action_id The index of the action to be evaluated.
   * @return The vector of micro-states.
   */
  std::vector<SE3Pose> forwardAction(const SE3Pose &curr, int action_id) const override {
    // Output
    std::vector<SE3Pose> next_micro;
    Eigen::Vector6d u = mprim_[action_id].uVec[0];
    double tau = mprim_[action_id].tVec[0];
    for (double t_ = 0.05; t_ < tau; t_ += .05) {
      next_micro.push_back(transition(curr, u, t_));
    }
    next_micro.push_back(transition(curr, u, tau));
    return next_micro;
  }

  /**
   * Returns the state representation of the metric state.
   * @param metric The metric state.
   * @return The state.
   */
  SE3Pose toState(const std::vector<double> &metric) const override {
    Eigen::Vector6d state; state << metric[0], metric[1], metric[2], metric[3], metric[4], metric[5];
    return SE3Pose((Eigen::Vector6d) state);
  }

  /**
   * Returns the metric representation of the state.
   * @param cell The state.
   * @return The metric state.
   */
  std::vector<double> toMetric(const SE3Pose &cell) const override {
    Eigen::Vector6d state = cell.getVectorForm();
    return {state[0], state[1], state[2], state[3], state[4], state[5]};
  }

  /**
 * Returns a manifold distance metric on SE(2) for the cell coordinate states s1 and s2.
 * @param s1 The first state.
 * @param s2 The second state.
 * @return The resulting metric.
 */
  double stateMetric(const SE3Pose &s1, const SE3Pose &s2) const override {
    double result {0.0};
    auto s1_vec = toMetric(s1);
    auto s2_vec = toMetric(s2);

    std::vector<double> diff = {s1_vec[0] - s2_vec[0],
                                s1_vec[1] - s2_vec[1],
                                s1_vec[2] - s2_vec[2]};

    for (unsigned k = 0; k < s1_vec.size(); k++)
    {
      result += std::pow(s1_vec[k] - s2_vec[k], 2);
    }

    return std::sqrt(result);
  }

 private:

};

}
#endif //__ENV_SE3_H_
