#ifndef __ENV_SE2_H_
#define __ENV_SE2_H_

//#include <boost/multi_array.hpp>
#include <memory>   // std::unique_ptr
#include <utility>      // std::pair
#include <erl_env/environments/environment_metric.h>
#include <erl_env/systems/motion_primitive.h>
#include <erl_env/systems/dd_motion_model.h>
#include <erl_map/grid_map.h>
#include <erl_map/grid_map_utils.h>
#include <iostream>

namespace erl {

/**
 * @brief The EnvironmentSE2 class implements the Environment specification for an SE(2) environment (X, Y, Yaw), with
 * discretized linear and angular velocity motion primitives.
 */
 template <class U>
class EnvironmentSE2 : public EnvironmentMetric<std::vector<int>, U> {
 public:
  using EnvironmentMetric<std::vector<int>, U>::map; // Bring Parent variables into the namespace.

  typedef MotionPrimitive<std::array<double, 3>, std::array<double, 2>> MPrim;
  std::vector<MPrim> mprim_;
  size_t max_len_traj_ = 0;
  unsigned int heading_disc{25}; // Yaw discretization size.
  double heading_res{2 * M_PI / heading_disc}; // Yaw Resolution.

  // for each orientation
  //   for each motion primitive
  //     for each segment
  //      we have a vector of micro states (discrete coordinates)
  std::vector<std::vector<std::vector<std::vector<std::array<int, 3> > > > > mprim_xd_;

  /**
   * @brief Constructs the EnvironmentSE2 via the GridMap and motion primitive file provided.
   * @param map The map of the environment.
   * @param cmap The cost map of the environment.
   * @param mprim_yaml The file describing the motion primitives.
   * @param heading_disc The heading angle discretization level.
   */
  EnvironmentSE2(const std::shared_ptr<erl::GridMap<U>> &map,
                 const std::string &mprim_yaml, unsigned heading_disc = 25) : EnvironmentMetric<std::vector<int>, U>(map),
                                                                              heading_disc(heading_disc) {
    // Initialize SE2 motion primitives
    erl::mprmsFromYAML<std::array<double, 3>, std::array<double, 2>>(
        mprim_yaml,
        static_cast<std::array<double, 3> (*)(const std::array<double, 3> &,
                                              const std::array<double, 2> &, double)>(erl::dd_motion_model),
        std::array<double, 3>({0, 0, 0}),
        mprim_);

    for (size_t tr = 0; tr < mprim_.size(); ++tr)
      if (mprim_[tr].uVec.size() > max_len_traj_)
        max_len_traj_ = mprim_[tr].uVec.size();
    initMprim();
  }

  /*
   * @brief Constructs the EnvironmentSE2 from map parameters, costmap and motion primitive values directly.
   * @param map The map resolution.
   * @param cmap The cost map.
   * @param vlist The list of valid linear velocity inputs.
   * @param wlist The list of valid angular velocity inputs.
   * @param tlist The list of times for each primitive.
   */
  EnvironmentSE2(const std::shared_ptr<erl::GridMap<U>> &map,
                 const std::vector<double> &vlist,
                 const std::vector<double> &wlist,
                 const std::vector<double> &tlist,
                 unsigned heading_disc = 25)
      : EnvironmentMetric<std::vector<int>, U>(map),
        heading_disc(heading_disc) {
    // Initialize SE2 Motion Primitives.
    std::array<double, 3> x0({0.0, 0.0, 0.0});
    double dt = 0.05;
    for (unsigned k = 0; k < vlist.size(); ++k) {
      mprim_.push_back(MPrim());
      mprim_.back().uVec.push_back({vlist[k], wlist[k]});
      mprim_.back().tVec.push_back(tlist[k]);
      mprim_.back().cVec.push_back(tlist[k] * vlist[k]);
      mprim_.back().xVecVec.resize(1);

      for (double t = dt; t < (tlist[k] + dt); t += dt) {
        mprim_.back().xVecVec[0].push_back(std::move(
            erl::dd_motion_model(x0, mprim_.back().uVec[0], t)));

        //std::cout << "v=" << mprim_.back().uVec[0][0] << " "
        //          << "w=" << mprim_.back().uVec[0][1] <<std::endl;
        //std::cout << mprim_.back().xVecVec[0].back()[0] << " "
        //          << mprim_.back().xVecVec[0].back()[1] << " "
        //          << mprim_.back().xVecVec[0].back()[2] << std::endl;
      }
    }
    max_len_traj_ = 1;

    // compute discrete coordinates for each orientation
    initMprim();
  }

  /**
   * @brief Constructs the EnvironmentSE2 from map parameters, costmap and motion primitive values directly.
   * @param mapmin The minimum map boundaries.
   * @param mapmax The maximum map boundaries.
   * @param mapres The map resolution.
   * @param cmap The cost map.
   * @param vlist The list of valid linear velocity inputs.
   * @param wlist The list of valid angular velocity inputs.
   * @param tlist The list of times for each primitive.
   */
  EnvironmentSE2(const erl::GridMap<U> &map,
                 const std::string &mprim_yaml,
                 unsigned heading_disc = 25)
      : EnvironmentMetric<std::vector<int>, U>(std::make_shared<erl::GridMap<U>>(map)), heading_disc(heading_disc) {
    // Initialize SE2 motion primitives
    erl::mprmsFromYAML<std::array<double, 3>, std::array<double, 2>>(
        mprim_yaml,
        static_cast<std::array<double, 3> (*)(const std::array<double, 3> &,
                                              const std::array<double, 2> &, double)>(erl::dd_motion_model),
        std::array<double, 3>({0, 0, 0}),
        mprim_);

    for (size_t tr = 0; tr < mprim_.size(); ++tr)
      if (mprim_[tr].uVec.size() > max_len_traj_)
        max_len_traj_ = mprim_[tr].uVec.size();
    initMprim();
  }

  /**
   * @brief Computes the successors of the state curr.
   * @param curr The current state to compute successors of.
   * @param succ The vector of successor states.
   * @param succ_cost The vector of costs of each successor.
   * @param action_idx The vector of indices of the actions leading to the successor.
   */
  inline void getSuccessors(const std::vector<int> &curr,
                            std::vector<std::vector<int>> &succ,
                            std::vector<double> &succ_cost,
                            std::vector<int> &action_idx) const override {
    size_t num_traj = mprim_.size();

    // Reserve space for successors
    succ.reserve(num_traj * max_len_traj_);
    succ_cost.reserve(num_traj * max_len_traj_);
    action_idx.reserve(num_traj * max_len_traj_);

    for (size_t tr = 0; tr < num_traj; ++tr) {
      size_t len_traj = mprim_xd_[curr[2]][tr].size();
      for (size_t len = 0; len < len_traj; ++len) {
        // check for collisions along the microstates
        bool collided = false;
        size_t len_fine_sz = mprim_xd_[curr[2]][tr][len].size();
        for (size_t len_fine = 0; len_fine < len_fine_sz; ++len_fine) {
          int x_val = curr[0] + mprim_xd_[curr[2]][tr][len][len_fine][0];
          int y_val = curr[1] + mprim_xd_[curr[2]][tr][len][len_fine][1];
          //int q_val = mprim_xd_[curr[2]][tr][len][len_fine][2];

          // Discard motion primitives that go outside of the map
          if (x_val < 0 || x_val >= map->size()[0] ||
              y_val < 0 || y_val >= map->size()[1]) {
            collided = true;
            break;
          }

          // Discard motion primitives that collide
          if (map->map().at(map->subv2ind({x_val, y_val}))) {
            collided = true;
            break;
          }

          if (len_fine == len_fine_sz - 1) {
            int q_val = mprim_xd_[curr[2]][tr][len][len_fine][2];
            std::vector<int> next({x_val, y_val, q_val});
            succ.push_back(next);
            succ_cost.push_back(mprim_[tr].cVec[len]);
            action_idx.push_back(static_cast<int>(tr * max_len_traj_ + len)); // action_id
          }
        }
        // No need to check the rest of the trajectory length if we collided already
        if (collided)
          break;
      }
    }
  }

  /**
   * @brief Computes the vector of micro-states from the current state if following the action given by action_id.
   * @param curr The current state.
   * @param action_id The index of the action to be evaluated.
   * @return The vector of micro-states.
   */
  std::vector<std::vector<int>> forwardAction(const std::vector<int> &curr, int action_id) const override {
    // Output
    std::vector<std::vector<int>> next_micro;
    // find which trajectory it is
    //size_t num_traj = mprim_.size();
    size_t tr = action_id / max_len_traj_;
    size_t len_sz = action_id % max_len_traj_;
    for (size_t len = 0; len <= len_sz; ++len) {
      size_t len_fine_sz = mprim_xd_[curr[2]][tr][len].size();
      for (size_t len_fine = 0; len_fine < len_fine_sz; ++len_fine) {
        int x_val = curr[0] + mprim_xd_[curr[2]][tr][len][len_fine][0];
        int y_val = curr[1] + mprim_xd_[curr[2]][tr][len][len_fine][1];
        int q_val = mprim_xd_[curr[2]][tr][len][len_fine][2];
        next_micro.push_back({x_val, y_val, q_val});
      }
    }
    return next_micro;
  }

  /**
   * Returns the state representation of the metric state.
   * @param metric The metric state.
   * @return The state.
   */
  std::vector<int> toState(const std::vector<double> &metric) const override {
    std::vector<int> cell = map->meters2cells({metric[0], metric[1]});
    int theta = erl::meters2cells(metric[2], -M_PI, heading_res);
    return {cell[0], cell[1], theta};
  }

  /**
   * Returns the metric representation of the state.
   * @param cell The state.
   * @return The metric state.
   */
  std::vector<double> toMetric(const std::vector<int> &cell) const override {
    std::vector<double> metric = map->cells2meters({cell[0], cell[1]});
    double theta = erl::cells2meters(cell[2], -M_PI, heading_res);
    return {metric[0], metric[1], theta};
  }

  /**
 * Returns a manifold distance metric on SE(2) for the cell coordinate states s1 and s2.
 * @param s1 The first state.
 * @param s2 The second state.
 * @return The resulting metric.
 */
  double stateMetric(const std::vector<int> &s1, const std::vector<int> &s2) const override {
    double result {0.0};
    auto s1_vec = toMetric(s1);
    auto s2_vec = toMetric(s2);

    std::vector<double> diff = {s1_vec[0] - s2_vec[0],
                                s1_vec[1] - s2_vec[2],
                                std::cos(s1_vec[2] - s2_vec[2]),
                                std::sin(s1_vec[2] - s2_vec[2])};

    for (unsigned k = 0; k < s1_vec.size(); k++)
    {
      result += std::pow(s1_vec[k] - s2_vec[k], 2);
    }

    return std::sqrt(result);
  }

 private:

  /**
   * Initializes the lookup table for successors generated from the specified Motion Primitives.
   */
  void initMprim() {
    /*
    for( int mp = 0; mp < mprim_.size(); ++mp)
      for( int l = 0; l < mprim_[mp].xVecVec.size(); ++l )
        for( int m = 0; m < mprim_[mp].xVecVec[l].size(); ++m )
          std::cout << mprim_[mp].xVecVec[l][m][0] << " "
                    << mprim_[mp].xVecVec[l][m][1] << " "
                    << mprim_[mp].xVecVec[l][m][2] << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;
    */
    // compute discrete coordinates for each orientation
    double x, y, q;
    int xc, yc, qc;
    mprim_xd_.resize(heading_disc);
    size_t num_prim = mprim_.size();
    for (size_t k = 0; k < heading_disc; ++k) {
      mprim_xd_[k].resize(num_prim);
      double ori = erl::cells2meters(k, -M_PI, heading_res);
      for (unsigned pr = 0; pr < num_prim; ++pr) {
        size_t num_seg = mprim_[pr].xVecVec.size();
        mprim_xd_[k][pr].resize(num_seg);
        for (size_t seg = 0; seg < num_seg; ++seg) {
          size_t num_sta = mprim_[pr].xVecVec[seg].size();
          for (size_t st = 0; st < num_sta; ++st) {
            erl::smart_plus_SE2(map->origin()[0], map->origin()[1], ori,
                               mprim_[pr].xVecVec[seg][st][0],
                               mprim_[pr].xVecVec[seg][st][1],
                               mprim_[pr].xVecVec[seg][st][2],
                               x, y, q);
            xc = erl::meters2cells(x, map->min()[0], map->res()[0])
                - map->origincells()[0];
            yc = erl::meters2cells(y, map->min()[1], map->res()[1])
                - map->origincells()[1];
            qc = erl::meters2cells(q, -M_PI, heading_res);

            // add only if unique
            if (mprim_xd_[k][pr][seg].empty() ||
                xc != mprim_xd_[k][pr][seg].back()[0] ||
                yc != mprim_xd_[k][pr][seg].back()[1] ||
                qc != mprim_xd_[k][pr][seg].back()[2])
              mprim_xd_[k][pr][seg].push_back({xc, yc, qc});
          }
        }
      }
    }
  }
};

}

#endif

