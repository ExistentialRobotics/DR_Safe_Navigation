#ifndef __ENV_2D_H_
#define __ENV_2D_H_

//#include <boost/multi_array.hpp>
#include <erl_env/environments/environment_metric.h>
#include <erl_env/systems/motion_primitive.h>
#include <erl_map/grid_map.h>
#include <erl_utilities/trajectories/piecewise_polynomial.h>

#include <memory>   // std::unique_ptr
#include <utility>  // std::pair
#include <vector>
namespace erl {

/**
 * @brief The Environment2D class implements the Environment specification for a 2D Grid environment.
 */
template <class U>
class Environment2D : public EnvironmentMetric<std::vector<int>, U> {
 public:
  using EnvironmentMetric<std::vector<int>, U>::map;  // Bring Parent variables into the namespace.

  std::vector<MotionPrimitive<std::pair<double, double>, std::string>> mprim_;  // Motion Primitives
  /**
   * @brief Constructs the Environment2D via the GridMap and costmaps provided. Use this constructor when there are
   * multiple environments to avoid storing multiple copies of the maps.
   * @param map The map of the environment.
   * @param cmap The cost map of the environment.
   */
  Environment2D(const std::shared_ptr<erl::GridMap<U>> &map) : EnvironmentMetric<std::vector<int>, U>(map), mprim_(8) {
    // Initialize 2-D Motion Primitives.
    initMprim();
  }

  /**
   * @brief Constructs the Environment2D via regular GridMap. Internally they are converted to shared pointer.
   * @param map The map of the environment.
   * @param cmap The cost map of the environment.
   */
  Environment2D(const erl::GridMap<U> &map)
      : EnvironmentMetric<std::vector<int>, U>(std::make_shared<erl::GridMap<U>>(map)), mprim_(8) {
    initMprim();
  }

  /**
   * @brief Constructs the Environment2D from map parameters and a costmap directly. Use this constructor for
   * convenience.
   * @param mapmin The minimum map boundaries.
   * @param mapmax The maximum map boundaries.
   * @param mapres The map resolution.
   * @param cmap The cost map.
   */
  Environment2D(const std::vector<double> &mapmin, const std::vector<double> &mapmax, const std::vector<double> &mapres,
                const std::vector<U> cmap = std::vector<U>())
      : Environment2D(std::make_shared<erl::GridMap<U>>(erl::GridMap<U>(mapmin, mapmax, mapres))){};

  /**
   * @brief Computes the successors of the state curr.
   * @param curr The current state to compute successors of.
   * @param succ The vector of successor states.
   * @param succ_cost The vector of costs of each successor.
   * @param action_idx The vector of indices of the actions leading to the successor.
   */
  void getSuccessors(const std::vector<int> &curr, std::vector<std::vector<int>> &succ, std::vector<double> &succ_cost,
                     std::vector<int> &action_idx) const override {
    size_t num_traj = mprim_.size();
    size_t len_traj = mprim_[0].xVecVec.size();

    double min_x = map->min()[0];
    double res_x = map->res()[0];
    int sz_x = map->size()[0];
    double min_y = map->min()[1];
    double res_y = map->res()[1];
    int sz_y = map->size()[1];

    // Reserve space for successors
    succ.reserve(num_traj * len_traj);
    succ_cost.reserve(num_traj * len_traj);
    action_idx.reserve(num_traj * len_traj);

    for (unsigned tr = 0; tr < num_traj; ++tr)
      for (unsigned len = 0; len < len_traj; ++len) {
        // check for collisions along the microstates
        bool collided = false;
        unsigned int len_fine_sz = mprim_[tr].xVecVec[len].size();
        for (unsigned int len_fine = 0; len_fine < len_fine_sz; ++len_fine) {
          int x_val = erl::meters2cells(
              erl::cells2meters(curr[0], min_x, res_x) + mprim_[tr].xVecVec[len][len_fine].first, min_x, res_x);
          int y_val = erl::meters2cells(
              erl::cells2meters(curr[1], min_y, res_y) + mprim_[tr].xVecVec[len][len_fine].second, min_y, res_y);

          // Discard motion primitives that go outside of the map
          if (x_val < 0 || x_val >= sz_x || y_val < 0 || y_val >= sz_y) {
            collided = true;
            break;
          }
          // Discard motion primitives that collide
          size_t lindix = map->subv2ind({x_val, y_val});
          if (map->map().at(lindix) > 0) {
            collided = true;
            break;
          }
          // If End of trajectory, we arrive at the successor.
          if (len_fine == len_fine_sz - 1) {
            succ.push_back({x_val, y_val});
            succ_cost.push_back(mprim_[tr].cVec[len]);
            action_idx.push_back(tr * len_traj + len);  // action_id
          }
        }
        // No need to check the rest of the trajectory length if we collided already
        if (collided) break;
      }
  }

  /**
   * @brief Computes the vector of micro-states from the current state if following the action given by action_id.
   * @param curr The current state.
   * @param action_id The index of the action to be evaluated.
   * @return The vector of micro-states.
   */
  std::vector<std::vector<int>> forwardAction(const std::vector<int> &curr, int action_id) const override {
    // Output.
    std::vector<std::vector<int>> next_micro;
    size_t len_traj = mprim_[0].xVecVec.size();

    // convert action_id to specific control input
    unsigned int tr = action_id / len_traj;  // find which trajectory it is
    unsigned int len = action_id % len_traj;
    unsigned int len_fine_sz = mprim_[tr].xVecVec[len].size();

    for (unsigned int len_fine = 0; len_fine < len_fine_sz; ++len_fine) {
      int x_val = erl::meters2cells(
          erl::cells2meters(curr[0], map->min()[0], map->res()[0]) + mprim_[tr].xVecVec[len][len_fine].first,
          map->min()[0], map->res()[0]);
      int y_val = erl::meters2cells(
          erl::cells2meters(curr[1], map->min()[1], map->res()[1]) + mprim_[tr].xVecVec[len][len_fine].second,
          map->min()[1], map->res()[1]);
      next_micro.push_back({x_val, y_val});
    }
    return next_micro;
  }

  /**
   * @brief Computes the state representation of the metric state.
   * @param metric The metric state.
   * @return The state representation.
   */
  virtual std::vector<int> toState(const std::vector<double> &metric) const override {
    return map->meters2cells(metric);
  }

  /**
   * @brief Returns the metric representation of the state form.
   * @param cell The state.
   * @return The metric state.
   */
  virtual std::vector<double> toMetric(const std::vector<int> &cell) const override { return map->cells2meters(cell); }

  /**
   * @brief Generate PiecewisePolynomial trajectories from a 2D path.
   * @param path The path given.
   * @param tau The uniform time scaling parameter to constrain the velocity profile.
   * @param th_init The initial heading in radians. Set to zero if this is not needed.
   * @param th_end The final heading in radians. Set to zero if this is not needed.
   * @param vel_init The initial velocity. Default to 1 m/s.
   * @return The resulting PiecewisePolynomial trajectory.
   */
  trajectories::PiecewisePolynomial<double> toTrajectory(const std::vector<std::vector<double>> &path, double tau = 0.1,
                                                         double th_init = 0.0, double th_end = 0.0,
                                                         double vel_init = 1.0) const {
    // Compute the segment times
    std::vector<double> segment_times(path.size());
    segment_times[0] = 0.0;
    for (int k = 0; k < path.size() - 1; ++k) segment_times[k + 1] = segment_times[k] + tau;
    std::vector<Eigen::MatrixXd> knots(path.size(), Eigen::MatrixXd(2, 1));
    for (int i = 0; i < path.size(); i++) {
      Eigen::MatrixXd vec(2, 1);
      vec << path[i][0], path[i][1];
      knots[i] = (vec);
    }

    // Start and End Velocity. This may optionally be used to allow for terminal yaw constraints.
    Eigen::MatrixXd v_start(2, 1);
    v_start << vel_init * std::cos(th_init), vel_init * std::sin(th_init);
    Eigen::MatrixXd v_end(2, 1);
    v_end << vel_init * std::cos(th_end), vel_init * std::sin(th_end);

    return trajectories::PiecewisePolynomial<double>::Cubic(segment_times, knots, v_start, v_end);
  }

 private:
  /**
   * @brief Initialize Motion Primitives.
   */
  void initMprim() {
    // Setup 2-D Grid Motion Primitives
    double x_res = map->res()[0];
    double y_res = map->res()[1];
    mprim_[0].uVec.push_back("Forward");
    mprim_[0].xVecVec.push_back({std::make_pair(x_res, 0)});
    mprim_[1].uVec.push_back("Back");
    mprim_[1].xVecVec.push_back({std::make_pair(-x_res, 0)});
    mprim_[2].uVec.push_back("Right");
    mprim_[2].xVecVec.push_back({std::make_pair(0, y_res)});
    mprim_[3].uVec.push_back("Left");
    mprim_[3].xVecVec.push_back({std::make_pair(0, -y_res)});
    mprim_[4].uVec.push_back("Diag1");
    mprim_[4].xVecVec.push_back({std::make_pair(x_res, y_res)});
    mprim_[5].uVec.push_back("Diag2");
    mprim_[5].xVecVec.push_back({std::make_pair(-x_res, y_res)});
    mprim_[6].uVec.push_back("Diag3");
    mprim_[6].xVecVec.push_back({std::make_pair(x_res, -y_res)});
    mprim_[7].uVec.push_back("Diag4");
    mprim_[7].xVecVec.push_back({std::make_pair(-x_res, -y_res)});

    for (int k = 0; k < 8; ++k) {
      mprim_[k].tVec.push_back(1);
      mprim_[k].cVec.push_back(std::sqrt(mprim_[k].xVecVec[0][0].first * mprim_[k].xVecVec[0][0].first +
                                         mprim_[k].xVecVec[0][0].second * mprim_[k].xVecVec[0][0].second));
    }
  }
};

}  // namespace erl

#endif
