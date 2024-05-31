#ifndef __GRID_MAP_UTILS_H_
#define __GRID_MAP_UTILS_H_
#include <cmath>
#include <vector>
#include <array>
#include <limits>
#include <Eigen/Core>
#include <iostream> // for debugging
#include <erl_utilities/erl_utils.h>

// for saving the map
#include <fstream>               // std::ofstream
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>  // boost::filesystem::path
#include <typeinfo>

namespace erl {

/**
 * @file grid_map_utils.h
 * @brief Mapping utility functions, useful for discretizing n-dimensional maps, and converting metric to cell coordinates.
 */

inline int meters2cells(double datam, double min, double res) {
  return static_cast<int>(std::floor((datam - min) / res));
}

inline double cells2meters(int datac, double min, double res) { return (static_cast<double>(datac) + 0.5) * res + min; }

inline double meters2cells_cont(double datam, double dim_min, double res) { return (datam - dim_min) / res - 0.5; }

// returns the first odd integer larger than x
inline int odd_ceil(double x) {
  if (std::abs(std::floor(x) - x) <= std::numeric_limits<double>::epsilon())
    x = std::floor(x);
  int ocx = static_cast<int>(std::ceil(x));
  return (ocx % 2 == 0) ? (ocx + 1) : (ocx);
}


/******************************************************************************/
// Array Implementations  
/******************************************************************************/
template<size_t D>
inline std::array<int, D> meters2cells(std::array<double, D> const &datam,
                                       std::array<double, D> const &dim_min,
                                       std::array<double, D> const &res) {
  std::array<int, D> datac;
  for (unsigned k = 0; k < D; ++k)
    datac[k] = meters2cells(datam[k], dim_min[k], res[k]);
  return datac;
}

template<size_t D>
inline std::array<double, D> cells2meters(std::array<int, D> const &datac,
                                          std::array<double, D> const &dim_min,
                                          std::array<double, D> const &res) {
  std::array<double, D> datam;
  for (unsigned k = 0; k < D; ++k)
    datam[k] = cells2meters(datac[k], dim_min[k], res[k]);
  return datam;
}

template<size_t D>
inline std::array<double, D> meters2cells_cont(std::array<double, D> const &datam,
                                               std::array<double, D> const &dim_min,
                                               std::array<double, D> const &res) {
  std::array<double, D> datac;
  for (unsigned k = 0; k < D; ++k)
    datac[k] = meters2cells_cont(datam[k], dim_min[k], res[k]);
  return datac;
}

// Row major order as in C++
inline std::size_t subv2ind_rowmajor(int *datac_begin, int *datac_end, int *size_begin, int *size_end) {
  if (datac_end <= datac_begin || size_end <= size_begin) return -1;
  std::size_t idx = *(datac_end - 1);
  std::size_t prod = 1;
  int *it1 = datac_end - 2;
  int *it2 = size_end - 1;
  for (; it1 != (datac_begin - 1) && it2 != size_begin; --it1, --it2) {
    prod *= (*it2);
    idx += prod * (*it1);
  }
  return idx;
}

inline std::size_t subv2ind_rowmajor(int *datac_begin, int *datac_end,
                                     std::vector<int>::const_iterator const &size_begin,
                                     std::vector<int>::const_iterator const &size_end) {
  if (datac_end <= datac_begin || size_end <= size_begin) return -1;
  std::size_t idx = *(datac_end - 1);
  std::size_t prod = 1;
  int *it1 = datac_end - 2;
  std::vector<int>::const_iterator it2 = size_end - 1;
  for (; it1 != (datac_begin - 1) && it2 != size_begin; --it1, --it2) {
    prod *= (*it2);
    idx += prod * (*it1);
  }
  return idx;
}

template<std::size_t D>
inline std::array<int, D> ind2subv_rowmajor(std::size_t ind, std::array<int, D> const &sz) {
  std::array<int, D> subv;
  for (int k = D - 1; k >= 0; --k) {
    subv[k] = ind % sz[k];
    ind -= subv[k];
    ind /= sz[k];
  }
  return subv;
}

// Column major order as in MATLAB
inline std::size_t subv2ind_colmajor(int *datac_begin, int *datac_end, int *size_begin, int *size_end) {
  if (datac_end <= datac_begin || size_end <= size_begin) return -1;
  std::size_t idx = *datac_begin;
  std::size_t prod = 1;
  int *it1 = datac_begin + 1;
  int *it2 = size_begin;
  for (; it1 != datac_end && it2 != (size_end - 1); ++it1, ++it2) {
    prod *= (*it2);
    idx += (*it1) * prod;
  }
  return idx;
}

inline std::size_t subv2ind_colmajor(int *datac_begin, int *datac_end,
                                     std::vector<int>::const_iterator const &size_begin,
                                     std::vector<int>::const_iterator const &size_end) {
  if (datac_end <= datac_begin || size_end <= size_begin) return -1;
  std::size_t idx = *datac_begin;
  std::size_t prod = 1;
  int *it1 = datac_begin + 1;
  std::vector<int>::const_iterator it2 = size_begin;
  for (; it1 != datac_end && it2 != (size_end - 1); ++it1, ++it2) {
    prod *= (*it2);
    idx += (*it1) * prod;
  }
  return idx;
}

template<std::size_t D>
inline std::array<int, D> ind2subv_colmajor(std::size_t ind, std::array<int, D> const &sz) {
  std::array<int, D> subv;
  for (size_t k = 0; k < D; ++k) {
    subv[k] = ind % sz[k];
    ind -= subv[k];
    ind /= sz[k];
  }
  return subv;
}



/******************************************************************************/
// Vector Implementations
/******************************************************************************/
inline std::vector<int> meters2cells(std::vector<double>::const_iterator const &datam_begin,
                                     std::vector<double>::const_iterator const &datam_end,
                                     std::vector<double>::const_iterator const &dim_min_begin,
                                     std::vector<double>::const_iterator const &dim_min_end,
                                     std::vector<double>::const_iterator const &res_begin,
                                     std::vector<double>::const_iterator const &res_end) {
  std::vector<int> datac(std::distance(datam_begin, datam_end));
  std::vector<double>::const_iterator it1 = datam_begin;
  std::vector<double>::const_iterator it2 = dim_min_begin;
  std::vector<double>::const_iterator it3 = res_begin;
  for (unsigned k = 0; it1 != datam_end; ++it1, ++it2, ++it3, ++k)
    datac[k] = meters2cells(*it1, *it2, *it3);
  return datac;
}

inline std::vector<double> cells2meters(std::vector<int>::const_iterator const &datac_begin,
                                        std::vector<int>::const_iterator const &datac_end,
                                        std::vector<double>::const_iterator const &dim_min_begin,
                                        std::vector<double>::const_iterator const &dim_min_end,
                                        std::vector<double>::const_iterator const &res_begin,
                                        std::vector<double>::const_iterator const &res_end) {
  std::vector<double> datam(std::distance(datac_begin, datac_end));
  std::vector<int>::const_iterator it1 = datac_begin;
  std::vector<double>::const_iterator it2 = dim_min_begin;
  std::vector<double>::const_iterator it3 = res_begin;
  for (unsigned k = 0; it1 != datac_end; ++it1, ++it2, ++it3, ++k)
    datam[k] = meters2cells(*it1, *it2, *it3);
  return datam;
}

inline std::vector<double> meters2cells_cont(
    std::vector<double>::const_iterator const &datam_begin,
    std::vector<double>::const_iterator const &datam_end,
    std::vector<double>::const_iterator const &dim_min_begin,
    std::vector<double>::const_iterator const &dim_min_end,
    std::vector<double>::const_iterator const &res_begin,
    std::vector<double>::const_iterator const &res_end) {
  std::vector<double> datac(std::distance(datam_begin, datam_end));
  std::vector<double>::const_iterator it1 = datam_begin;
  std::vector<double>::const_iterator it2 = dim_min_begin;
  std::vector<double>::const_iterator it3 = res_begin;
  for (unsigned k = 0; it1 != datam_end; ++it1, ++it2, ++it3, ++k)
    datac[k] = meters2cells_cont(*it1, *it2, *it3);
  return datac;
}

inline std::vector<int> meters2cells(std::vector<double> const &datam,
                                     std::vector<double> const &dim_min,
                                     std::vector<double> const &res) {
  std::vector<int> datac(datam.size());
  for (unsigned k = 0; k < datam.size(); ++k)
    datac[k] = meters2cells(datam[k], dim_min[k], res[k]);
  return datac;
}

inline std::vector<int> meters2cells(std::vector<double> const &datam,
                                     std::vector<double> const &dim_min, double res) {
  std::vector<int> datac(datam.size());
  for (unsigned k = 0; k < datam.size(); ++k)
    datac[k] = meters2cells(datam[k], dim_min[k], res);
  return datac;
}

inline std::vector<double> cells2meters(std::vector<int> const &datac,
                                        std::vector<double> const &dim_min, std::vector<double> const &res) {
  std::vector<double> datam(datac.size());
  for (unsigned k = 0; k < datac.size(); ++k)
    datam[k] = cells2meters(datac[k], dim_min[k], res[k]);
  return datam;
}

inline std::vector<double> cells2meters(std::vector<int> const &datac,
                                        std::vector<double> const &dim_min, double res) {
  std::vector<double> datam(datac.size());
  for (unsigned k = 0; k < datac.size(); ++k)
    datam[k] = cells2meters(datac[k], dim_min[k], res);
  return datam;
}

inline std::vector<double> meters2cells_cont(const std::vector<double> &datam,
                                             const std::vector<double> &dim_min,
                                             const std::vector<double> &res) {
  std::vector<double> datac(datam.size());
  for (unsigned k = 0; k < datam.size(); ++k)
    datac[k] = meters2cells_cont(datam[k], dim_min[k], res[k]);
  return datac;
}

inline std::vector<double> meters2cells_cont(const std::vector<double> &datam,
                                             const std::vector<double> &dim_min,
                                             double res) {
  std::vector<double> datac(datam.size());
  for (unsigned k = 0; k < datam.size(); ++k)
    datac[k] = meters2cells_cont(datam[k], dim_min[k], res);
  return datac;
}

// Row major order as in C++
inline std::size_t subv2ind_rowmajor(std::vector<int>::const_iterator const &datac_begin,
                                     std::vector<int>::const_iterator const &datac_end,
                                     std::vector<int>::const_iterator const &size_begin,
                                     std::vector<int>::const_iterator const &size_end) {
  if (datac_end <= datac_begin || size_end <= size_begin) return -1;
  std::size_t idx = *(datac_end - 1);
  std::size_t prod = 1;
  std::vector<int>::const_iterator it1 = datac_end - 2;
  std::vector<int>::const_iterator it2 = size_end - 1;
  for (; it1 != (datac_begin - 1) && it2 != size_begin; --it1, --it2) {
    prod *= (*it2);
    idx += prod * (*it1);
  }
  return idx;
}

inline std::vector<int> ind2subv_rowmajor(std::size_t ind,
                                          const std::vector<int>::const_iterator &size_begin,
                                          const std::vector<int>::const_iterator &size_end) {
  const std::size_t ndims = std::distance(size_begin, size_end);
  std::vector<int> subv(ndims);
  std::vector<int>::const_iterator it = size_end - 1;
  for (int k = ndims - 1; k >= 0; --k, --it) {
    subv[k] = ind % (*it);
    ind -= subv[k];
    ind /= (*it);
  }
  return subv;
}

// Column major order as in MATLAB
inline std::size_t subv2ind_colmajor(std::vector<int>::const_iterator const &datac_begin,
                                     std::vector<int>::const_iterator const &datac_end,
                                     std::vector<int>::const_iterator const &size_begin,
                                     std::vector<int>::const_iterator const &size_end) {
  if (datac_end <= datac_begin || size_end <= size_begin) return -1;
  std::size_t idx = *datac_begin;
  std::size_t prod = 1;
  std::vector<int>::const_iterator it1 = datac_begin + 1;
  std::vector<int>::const_iterator it2 = size_begin;
  for (; it1 != datac_end && it2 != (size_end - 1); ++it1, ++it2) {
    prod *= (*it2);
    idx += (*it1) * prod;
  }
  return idx;
}

inline std::vector<int> ind2subv_colmajor(std::size_t ind,
                                          const std::vector<int>::const_iterator &size_begin,
                                          const std::vector<int>::const_iterator &size_end) {
  const std::size_t ndims = std::distance(size_begin, size_end);
  std::vector<int> subv(ndims);
  std::vector<int>::const_iterator it = size_begin;
  for (std::size_t k = 0; k < ndims; ++k, ++it) {
    subv[k] = ind % (*it);
    ind -= subv[k];
    ind /= (*it);
  }
  return subv;
}


/******************************************************************************/
// Matrix Implementations
/******************************************************************************/
inline Eigen::MatrixXi meters2cells(const Eigen::MatrixXd &datam,
                                    const Eigen::VectorXd &dim_min,
                                    const Eigen::VectorXd &res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXi datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells(datam(d, p), dim_min(d), res(d));
  return datac;
}

inline Eigen::MatrixXi meters2cells(const Eigen::MatrixXd &datam,
                                    const Eigen::VectorXd &dim_min,
                                    double res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXi datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells(datam(d, p), dim_min(d), res);
  return datac;
}

inline Eigen::MatrixXd cells2meters(const Eigen::MatrixXi &datac,
                                    const Eigen::VectorXd &dim_min,
                                    const Eigen::VectorXd &res) {
  // datac = [num_dim x num_pts]
  const size_t num_dim = datac.rows();
  const size_t num_pts = datac.cols();
  Eigen::MatrixXd datam(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datam(d, p) = cells2meters(datac(d, p), dim_min(d), res(d));
  return datam;
}

inline Eigen::MatrixXd cells2meters(const Eigen::MatrixXi &datac,
                                    const Eigen::VectorXd &dim_min,
                                    double res) {
  // datac = [num_dim x num_pts]
  const size_t num_dim = datac.rows();
  const size_t num_pts = datac.cols();
  Eigen::MatrixXd datam(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datam(d, p) = cells2meters(datac(d, p), dim_min(d), res);
  return datam;
}

inline Eigen::MatrixXd meters2cells_cont(const Eigen::MatrixXd &datam,
                                         const Eigen::VectorXd &dim_min,
                                         const Eigen::VectorXd &res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXd datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells_cont(datam(d, p), dim_min(d), res(d));
  return datac;
}

inline Eigen::MatrixXd meters2cells_cont(const Eigen::MatrixXd &datam,
                                         const Eigen::VectorXd &dim_min,
                                         double res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXd datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells_cont(datam(d, p), dim_min(d), res);
  return datac;
}

inline Eigen::MatrixXi meters2cells(const Eigen::MatrixXd &datam,
                                    const std::vector<double> &dim_min,
                                    const std::vector<double> &res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXi datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells(datam(d, p), dim_min[d], res[d]);
  return datac;
}

inline Eigen::MatrixXi meters2cells(const Eigen::MatrixXd &datam,
                                    const std::vector<double> &dim_min,
                                    double res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXi datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells(datam(d, p), dim_min[d], res);
  return datac;
}

inline Eigen::MatrixXd cells2meters(const Eigen::MatrixXi &datac,
                                    const std::vector<double> &dim_min,
                                    const std::vector<double> &res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datac.rows();
  const size_t num_pts = datac.cols();
  Eigen::MatrixXd datam(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datam(d, p) = cells2meters(datac(d, p), dim_min[d], res[d]);
  return datam;
}

inline Eigen::MatrixXd cells2meters(const Eigen::MatrixXi &datac,
                                    const std::vector<double> &dim_min,
                                    double res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datac.rows();
  const size_t num_pts = datac.cols();
  Eigen::MatrixXd datam(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datam(d, p) = cells2meters(datac(d, p), dim_min[d], res);
  return datam;
}

inline Eigen::MatrixXd meters2cells_cont(const Eigen::MatrixXd &datam,
                                         const std::vector<double> &dim_min,
                                         const std::vector<double> &res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXd datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells_cont(datam(d, p), dim_min[d], res[d]);
  return datac;
}

inline Eigen::MatrixXd meters2cells_cont(const Eigen::MatrixXd &datam,
                                         const std::vector<double> &dim_min,
                                         double res) {
  // datam = [num_dim x num_pts]
  const size_t num_dim = datam.rows();
  const size_t num_pts = datam.cols();
  Eigen::MatrixXd datac(num_dim, num_pts);
  for (unsigned d = 0; d < num_dim; ++d)
    for (unsigned p = 0; p < num_pts; ++p)
      datac(d, p) = meters2cells_cont(datam(d, p), dim_min[d], res);
  return datac;
}
/******************************************************************************/


/******************************************************************************/
// Algorithms
/******************************************************************************/


template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic>
inflate_map2d(const Eigen::MatrixBase<Derived> &cmap,
              const std::vector<double> &res, double rad) {
  Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> omap = cmap;
  if (rad < 0.00001)
    return omap; // nothing to do here

  int bbx = static_cast<int>(std::ceil(rad / res[0]));
  int bby = static_cast<int>(std::ceil(rad / res[1]));

  std::vector<std::array<int, 2>> ns;
  ns.reserve((2 * bbx + 1) * (2 * bby + 1));
  for (int nx = -bbx; nx <= bbx; ++nx)
    for (int ny = -bby; ny <= bby; ++ny) {
      double d = std::hypot(nx * res[0], ny * res[1]);
      if (0 < d && d <= rad)
        ns.push_back({nx, ny});
    }

  for (unsigned int x = 0; x < cmap.rows(); ++x)
    for (unsigned int y = 0; y < cmap.cols(); ++y) {
      if (cmap(x, y) > 0) {
        for (const auto &it : ns) {
          int nx = x + it[0], ny = y + it[1];
          if (0 <= nx && nx < cmap.rows() && 0 <= ny && ny < cmap.cols())
            omap(nx, ny) = cmap(x, y);
        }
      }
    }
  return omap;
}

template<class T>
inline std::vector<T>
inflateMap(const std::vector<T> &map_cost,
           const std::vector<int> &map_size,
           const std::vector<double> &map_resolution,
           bool is_rowmajor,
           const std::vector<double> &inflation_radius) {

  // find the number of cells to inflate per dimension
  std::vector<int> extents(map_resolution.size());
  size_t num_ns = 1;
  for (size_t i = 0; i < map_resolution.size(); ++i) {
    extents[i] = 2 * static_cast<int>(std::ceil(inflation_radius[i] / map_resolution[i])) + 1;
    num_ns *= extents[i];
  }

  // compute the neighbor coordinates
  std::vector<std::vector<int>> ns;
  ns.reserve(num_ns);
  for (size_t i = 0; i < num_ns; ++i) {
    // convert the linear index of every potential neighbor
    // to an index in [-(extents[i]-1)/2,(extents[i]-1)/2]
    // and check if this falls within the axis aligned ellipsoid
    // defined by inflation_radius
    std::vector<int> subv = ind2subv_colmajor(i, extents.begin(), extents.end());
    //std::cout << subv << std::endl;
    double d = 0.0; // ellipsoid radius
    for (size_t k = 0; k < map_size.size(); ++k) {
      subv[k] -= (extents[k] - 1) / 2; // add offset (i.e., cells2meters)
      double tmp = subv[k] * (map_resolution[k] / inflation_radius[k]);
      d += tmp * tmp;
    }

    if (0.0 < d && d <= 1.000001)
      ns.push_back(subv);
  }

  std::vector<T> inflated_cost(map_cost);
  for (size_t i = 0; i < map_cost.size(); ++i)
    if (map_cost[i] > T(0)) {
      // get coordinates of ith entry
      std::vector<int> subv = is_rowmajor ? ind2subv_rowmajor(i, map_size.begin(), map_size.end()) :
                              ind2subv_colmajor(i, map_size.begin(), map_size.end());
      // find the neighbors and make them occupied
      for (const auto &it : ns) {
        bool valid = true;
        std::vector<int> neib_coord(it);
        for (size_t k = 0; k < map_size.size(); ++k) {
          neib_coord[k] += subv[k];
          if (neib_coord[k] < 0 || neib_coord[k] >= map_size[k]) {
            valid = false;
            break;
          }
        }
        if (valid)
          inflated_cost[is_rowmajor ? subv2ind_rowmajor(neib_coord.begin(),
                                                        neib_coord.end(),
                                                        map_size.begin(),
                                                        map_size.end()) :
                        subv2ind_colmajor(neib_coord.begin(), neib_coord.end(), map_size.begin(), map_size.end())] =
              map_cost[i];
      }
    }
  return inflated_cost;
}

/******************************************************************************/
inline void bresenhamStep(double dv, double sv, int svc, double vmin, double vres,
                          int &stepV, double &tDeltaV, double &tMaxV) {
  if (dv > 0) {
    stepV = 1;
    tDeltaV = vres / dv;
    tMaxV = (vmin + (svc + 1) * vres - sv) / dv; // parametric distance until the first crossing
  } else if (dv < 0) {
    stepV = -1;
    tDeltaV = vres / -dv;
    tMaxV = (vmin + svc * vres - sv) / dv;
  } else {
    stepV = 0;
    tDeltaV = 0.0;
    tMaxV = std::numeric_limits<double>::infinity(); // the line doesn't cross the next plane
  }
}

inline void bresenham2D(double sx, double sy,
                        double ex, double ey,
                        double xmin, double ymin,
                        double xres, double yres,
                        std::vector<int> &xvec,
                        std::vector<int> &yvec) {
  xvec.clear();
  yvec.clear();
  // find start and end cells
  int sxc = meters2cells(sx, xmin, xres);
  int exc = meters2cells(ex, xmin, xres);
  int syc = meters2cells(sy, ymin, yres);
  int eyc = meters2cells(ey, ymin, yres);
  double dx = ex - sx;
  double dy = ey - sy;
  int stepX, stepY;          // direction of grid traversal
  double tDeltaX, tDeltaY; // parametric step size along different dimensions
  double tMaxX, tMaxY; // used to determine the dimension of the next step along the line
  bresenhamStep(dx, sx, sxc, xmin, xres, stepX, tDeltaX, tMaxX);
  bresenhamStep(dy, sy, syc, ymin, yres, stepY, tDeltaY, tMaxY);

  // Add initial voxel to the list
  xvec.push_back(sxc);
  yvec.push_back(syc);
  while ((sxc != exc) || (syc != eyc)) {
    // Bug Fix. When Both X, and Y reach 1 at the same time, terminate.
    if (std::abs(tMaxX - 1) < 1e-4 && std::abs(tMaxY - 1) < 1e-4) {
      sxc = exc;
      syc = eyc;
    } else if (tMaxX < tMaxY) {
      sxc += stepX;
      tMaxX += tDeltaX;
    } else {
      syc += stepY;
      tMaxY += tDeltaY;
    }
    xvec.push_back(sxc);
    yvec.push_back(syc);
  }
}

// http://www.cse.yorku.ca/~amana/research/grid.pdf
// J. Amanatides, A. Woo, "A Fast Voxel Traversal Algorithm for Ray Tracing"
// https://www.mathworks.com/matlabcentral/fileexchange/56527-fast-raytracing-through-a-3d-grid
inline void bresenham3D(double sx, double sy, double sz,
                        double ex, double ey, double ez,
                        double xmin, double ymin, double zmin,
                        double xres, double yres, double zres,
                        std::vector<int> &xvec,
                        std::vector<int> &yvec,
                        std::vector<int> &zvec) {
  xvec.clear();
  yvec.clear();
  zvec.clear();
  // find start and end cells
  int sxc = meters2cells(sx, xmin, xres);
  int exc = meters2cells(ex, xmin, xres);
  int syc = meters2cells(sy, ymin, yres);
  int eyc = meters2cells(ey, ymin, yres);
  int szc = meters2cells(sz, zmin, zres);
  int ezc = meters2cells(ez, zmin, zres);
  double dx = ex - sx;
  double dy = ey - sy;
  double dz = ez - sz;
  int stepX, stepY, stepZ;          // direction of grid traversal
  double tDeltaX, tDeltaY, tDeltaZ; // parametric step size along different dimensions
  double tMaxX, tMaxY, tMaxZ; // used to determine the dim of the next step along the line
  bresenhamStep(dx, sx, sxc, xmin, xres, stepX, tDeltaX, tMaxX);
  bresenhamStep(dy, sy, syc, ymin, yres, stepY, tDeltaY, tMaxY);
  bresenhamStep(dz, sz, szc, zmin, zres, stepZ, tDeltaZ, tMaxZ);

  // Add initial voxel to the list
  xvec.push_back(sxc);
  yvec.push_back(syc);
  zvec.push_back(szc);
  while ((sxc != exc) || (syc != eyc) || (szc != ezc)) {
    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        sxc += stepX;
        tMaxX += tDeltaX;
      } else {
        szc += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        syc += stepY;
        tMaxY += tDeltaY;
      } else {
        szc += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
    xvec.push_back(sxc);
    yvec.push_back(syc);
    zvec.push_back(szc);
  }
}

inline std::vector<std::vector<int>>
bresenham(const std::vector<double> &start,
          const std::vector<double> &end,
          const std::vector<double> &gridmin,
          const std::vector<double> &gridres) {
  // find start and end cells
  std::vector<int> start_cell =
      meters2cells(start.begin(), start.end(), gridmin.begin(), gridmin.end(), gridres.begin(), gridres.end());
  std::vector<int>
      end_cell = meters2cells(end.begin(), end.end(), gridmin.begin(), gridmin.end(), gridres.begin(), gridres.end());
  std::vector<double> diff = end - start;
  //std::vector<double> diff(start.size());
  //std::transform(end.begin(), end.end(), start.begin(), diff.begin(), std::minus<double>());
  std::vector<int> step(start_cell.size()); // direction of grid traversal
  std::vector<double> tDelta(start_cell.size()); // parametric step size along different dimensions
  std::vector<double> tMax(start_cell.size()); // used to determine the dim of the next step along the line
  for (size_t k = 0; k < diff.size(); ++k)
    bresenhamStep(diff[k], start[k], start_cell[k], gridmin[k], gridres[k], step[k], tDelta[k], tMax[k]);

  // Add initial voxel to the list
  std::vector<std::vector<int>> cellidx;
  cellidx.push_back(start_cell);
  while (start_cell != end_cell) {
    auto min_idx = std::distance(tMax.begin(), std::min_element(tMax.begin(), tMax.end())); // TODO: need to increment all min elements at the same time
    start_cell[min_idx] += step[min_idx];
    tMax[min_idx] += tDelta[min_idx];
    cellidx.push_back(start_cell);
  }
  return cellidx;
}

}
#endif




/*
class map_nd {
 public:
  std::vector<double> min_;
  std::vector<double> max_;
  std::vector<double> res_;
  std::vector<int> size_;
  std::vector<double> origin_;
  std::vector<int> origincells_;
  bool row_major_{false}; // Indicates that the map is stored in row major order. Default to column-major.
  std::string mappath; // Path of the cost-map to load.

 public:
  map_nd() {} // default constructor

  // Constructs a map from an Input YAML file.
  // @param input_yaml The input YAML describing the map.
  map_nd(const std::string &input_yaml) {
    // Set map size
    YAML::Node map_spec = YAML::LoadFile(input_yaml);
    if (map_spec["mapdim"] && map_spec["origin"] && map_spec["mapres"] && map_spec["mappath"]) {
      size_ = map_spec["mapdim"].as<std::vector<int>>();
      origin_ = map_spec["origin"].as<std::vector<double>>();
      res_ = map_spec["mapres"].as<std::vector<double>>();
      double dev;
      for (unsigned k = 0; k < size_.size(); ++k) {
        if (size_[k] % 2 == 0)
          size_[k] += 1; // ensure the map size is odd!
        dev = size_[k] * res_[k] / 2;
        min_.push_back(origin_[k] - dev);
        max_.push_back(origin_[k] + dev);
        origincells_.push_back((size_[k] - 1) / 2);
      }
    } else if (map_spec["mapmin"] && map_spec["mapmax"] && map_spec["mapres"] && map_spec["mappath"]) {

      const std::vector<double> &devmin = map_spec["mapmin"].as<std::vector<double>>();
      const std::vector<double> &devmax = map_spec["mapmax"].as<std::vector<double>>();
      res_ = map_spec["mapres"].as<std::vector<double>>();
      double dev;

      for (unsigned k = 0; k < devmin.size(); ++k) {
        origin_.push_back((devmin[k] + devmax[k]) / 2);
        size_.push_back(odd_ceil((devmax[k] - devmin[k]) / res_[k]));
        dev = size_[k] * res_[k] / 2;
        min_.push_back(origin_[k] - dev);
        max_.push_back(origin_[k] + dev);
        origincells_.push_back((size_[k] - 1) / 2);
      }
    }
  }

  map_nd(std::vector<double> const &devmin,
         std::vector<double> const &devmax,
         std::vector<double> const &resolution,
         bool rowmajor = false)
      : map_nd(devmin.size(), devmin.data(), devmax.data(), resolution.data(), rowmajor) {}

  map_nd(std::vector<double> const &devmin, std::vector<double> const &devmax, double resolution)
      : map_nd(devmin.size(), devmin.data(), devmax.data(), std::vector<double>(devmin.size(), resolution).data()) {}

  map_nd(size_t num_dim, const double *devmin, const double *devmax, double resolution)
      : map_nd(num_dim, devmin, devmax, std::vector<double>(num_dim, resolution).data()) {}

  map_nd(size_t num_dim, const double *devmin, const double *devmax, const double *resolution, bool rowmajor = false)
      : res_(resolution, resolution + num_dim) {
    double dev;
    for (size_t k = 0; k < num_dim; ++k) {
      origin_.push_back((devmin[k] + devmax[k]) / 2);
      size_.push_back(odd_ceil((devmax[k] - devmin[k]) / resolution[k]));
      dev = size_[k] * resolution[k] / 2;
      min_.push_back(origin_[k] - dev);
      max_.push_back(origin_[k] + dev);
      origincells_.push_back((size_[k] - 1) / 2);
    }
    row_major_ = rowmajor;
  }

  map_nd(const std::vector<int> &oddsize, const std::vector<double> &origin,
         const std::vector<double> &resolution)
      : res_(resolution), size_(oddsize), origin_(origin) {
    double dev;
    for (size_t k = 0; k < size_.size(); ++k) {
      if (size_[k] % 2 == 0)
        size_[k] += 1; // ensure the map size is odd!
      dev = size_[k] * res_[k] / 2;
      min_.push_back(origin_[k] - dev);
      max_.push_back(origin_[k] + dev);
      origincells_.push_back((size_[k] - 1) / 2);
    }
  }

  ///////////////////////////////////////////////////////////////
  // Methods
  ///////////////////////////////////////////////////////////////
  // Compute Total map size;
  std::size_t totalSize() const {
    std::size_t sz = 1;
    for (unsigned k = 0; k < size_.size(); ++k)
      sz *= size_[k];
    return sz;
  }

  size_t getLinearIndex(const std::vector<int> &cell) const {
    return row_major_ ? subv2ind_rowmajor(cell) : subv2ind_colmajor(cell);
  }

  std::vector<int> getCellFromIndex(size_t lindix) const {
    return row_major_ ? ind2subv_rowmajor(lindix) : ind2subv_colmajor(lindix);
  }

  std::vector<double> const &min(void) const { return min_; }
  std::vector<double> const &max(void) const { return max_; }
  std::vector<double> const &res(void) const { return res_; }
  std::vector<int> const &size(void) const { return size_; }
  std::vector<double> const &origin(void) const { return origin_; }
  std::vector<int> const &origincells(void) const { return origincells_; }

  bool inMap(const std::vector<double> &datam) const {
    for (size_t k = 0; k < datam.size(); ++k)
      if (datam[k] < min_[k] || datam[k] > max_[k])
        return false;
    return true;
  }

  std::vector<int> meters2cells(const std::vector<double> &datam) const { return meters2cells(datam, min_, res_); }

  std::vector<double> cells2meters(const std::vector<int> &datac) const { return cells2meters(datac, min_, res_); }

  std::vector<double> meters2cells_cont(const std::vector<double> &datam) const {
    return meters2cells_cont(datam,
                                 min_,
                                 res_);
  }

  std::vector<int> meters2cells(const std::vector<double>::const_iterator &datam_begin,
                                const std::vector<double>::const_iterator &datam_end) const {
    return meters2cells(datam_begin,
                            datam_end,
                            min_.begin(),
                            min_.end(),
                            res_.begin(),
                            res_.end());
  }

  std::vector<double> cells2meters(const std::vector<int>::const_iterator &datac_begin,
                                   const std::vector<int>::const_iterator &datac_end) const {
    return cells2meters(datac_begin,
                            datac_end,
                            min_.begin(),
                            min_.end(),
                            res_.begin(),
                            res_.end());
  }

  std::vector<double> meters2cells_cont(const std::vector<double>::const_iterator &datam_begin,
                                        const std::vector<double>::const_iterator &datam_end) const {
    return meters2cells_cont(datam_begin,
                                 datam_end,
                                 min_.begin(),
                                 min_.end(),
                                 res_.begin(),
                                 res_.end());
  }

  // Row major order as in C++
  size_t subv2ind_rowmajor(std::vector<int> const &datac) const {
    const size_t ndims = datac.size();
    if (ndims == 0) return -1;

    size_t idx = datac.back();
    size_t prod = 1;
    for (int k = ndims - 1; k > 0; --k) {
      prod = prod * size_[k];
      idx += datac[k - 1] * prod;
    }
    return idx;
  }

  std::vector<int> ind2subv_rowmajor(size_t ind) const {
    const size_t ndims = size_.size();
    std::vector<int> subv(ndims);
    for (int k = ndims - 1; k >= 0; --k) {
      subv[k] = ind % size_[k];
      ind -= subv[k];
      ind /= size_[k];
    }
    return subv;
  }

  // Column major order as in MATLAB
  std::size_t subv2ind_colmajor(std::vector<int> const &datac) const {
    const size_t ndims = datac.size();
    if (ndims == 0) return -1;

    size_t idx = datac[0];
    size_t prod = 1;
    for (size_t k = 1; k < ndims; ++k) {
      prod *= size_[k - 1];
      idx += datac[k] * prod;
    }
    return idx;
  }

  std::vector<int> ind2subv_colmajor(std::size_t ind) const {
    const size_t ndims = size_.size();
    std::vector<int> subv(ndims);
    for (size_t k = 0; k < ndims; ++k) {
      subv[k] = ind % size_[k];
      ind -= subv[k];
      ind /= size_[k];
    }
    return subv;
  }

  // Slices a 3-Dimensional map into 2-Dimensional map at a fixed index.
  // @tparam T The type of cmap being stored.
  // @param cmap The cmap to be sliced.
  // @param slice The slice index.
  // @return A pair of the 2-dimensional map and the sliced costmap.
  template<class T>
  std::pair<map_nd, std::vector<T>> get2DSlice(const std::vector<T> &cmap, size_t z_dim) {

    std::pair<map_nd, std::vector<T>> result;
    // Copy the 3D map, but truncate the third dimension.
    map_nd map_2d;
    map_2d.min_ = {min_[0], min_[1]};
    map_2d.max_ = {max_[0], max_[1]};
    map_2d.res_ = {res_[0], res_[1]};
    map_2d.size_ = {size_[0], size_[1]};
    map_2d.origin_ = {origin_[0], origin_[1]};
    map_2d.origincells_ = {origincells_[0], origincells_[1]};
    map_2d.row_major_ = row_major_;
    map_2d.mappath = mappath;

    std::vector<T> slice_map(size()[0] * size()[1]);       // Create the new map.

    if (!row_major_) {     // For Column-major order
      std::copy(cmap.begin() + z_dim * slice_map.size(),
                cmap.begin() + (z_dim + 1) * slice_map.size(),
                slice_map.begin());
    } else { // For Row-major order
      for (size_t k = 0; k < slice_map.size(); k++) {
        auto cell = map_2d.getCellFromIndex(k);         // Get Cell coordinates of the 2D Point
        auto result = getLinearIndex({cell[0], cell[1], (int) z_dim});         // Lookup in 3D cost-map
        slice_map[k] = cmap[result];
      }
    }
    // Return result.
    result.first = map_2d;
    result.second = slice_map;
    return result;
  }

  template<class T>
  std::vector<T> rowmajor2colmajor(const std::vector<T> &map) const {
    const size_t mapsz = map.size();
    std::vector<T> newmap(mapsz);
    for (int k = 0; k < mapsz; ++k)
      newmap[subv2ind_colmajor(ind2subv_rowmajor(k))] = map[k];
    return newmap;
  }

  template<class T>
  std::vector<T> colmajor2rowmajor(const std::vector<T> &map) const {
    const size_t mapsz = map.size();
    std::vector<T> newmap(mapsz);
    for (int k = 0; k < mapsz; ++k)
      newmap[subv2ind_rowmajor(ind2subv_colmajor(k))] = map[k];
    return newmap;
  }

  template<class T>
  void saveToYaml(const std::vector<T> &map, std::string pathToYaml) const {
    saveToYaml(map, pathToYaml, "rowmajor");
  }

  // storageorder in {rowmajor, colmajor}
  template<class T>
  void saveToYaml(const std::vector<T> &map, std::string pathToYaml, std::string storageorder) const {
    // Find the file path and file name
    boost::filesystem::path bfp(pathToYaml);
    std::string yaml_parent_path(bfp.parent_path().string());
    std::string yaml_name(bfp.stem().string());

    // Generate the YAML file
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "mapmin";
    out << YAML::Value << YAML::Flow;
    out << YAML::BeginSeq;
    for (int k = 0; k < min_.size(); ++k)
      out << min_[k];
    out << YAML::EndSeq;
    out << YAML::Key << "mapmax";
    out << YAML::Value << YAML::Flow;
    out << YAML::BeginSeq;
    for (int k = 0; k < max_.size(); ++k)
      out << max_[k];
    out << YAML::EndSeq;
    out << YAML::Key << "mapres";
    out << YAML::Value << YAML::Flow;
    out << YAML::BeginSeq;
    for (int k = 0; k < res_.size(); ++k)
      out << res_[k];
    out << YAML::EndSeq;
    out << YAML::Key << "mapdim";
    out << YAML::Value << YAML::Flow;
    out << YAML::BeginSeq;
    for (int k = 0; k < size_.size(); ++k)
      out << size_[k];
    out << YAML::EndSeq;
    out << YAML::Key << "origin";
    out << YAML::Value << YAML::Flow;
    out << YAML::BeginSeq;
    for (int k = 0; k < origin_.size(); ++k)
      out << origin_[k];
    out << YAML::EndSeq;
    out << YAML::Key << "origincells";
    out << YAML::Value << YAML::Flow;
    out << YAML::BeginSeq;
    for (int k = 0; k < origincells_.size(); ++k)
      out << origincells_[k];
    out << YAML::EndSeq;
    out << YAML::Key << "datatype";
    out << YAML::Value << typeid(T).name();
    out << YAML::Key << "storage";
    out << YAML::Value << storageorder;
    out << YAML::Key << "mappath";
    out << YAML::Value << yaml_name + ".cfg";
    out << YAML::EndMap;

    // Save yaml file
    std::ofstream ofs;
    ofs.open(yaml_parent_path + "/" + yaml_name + ".yaml", std::ofstream::out | std::ofstream::trunc);
    ofs << out.c_str();
    ofs.close();

    // Save cfg file
    ofs.open(yaml_parent_path + "/" + yaml_name + ".cfg", std::ofstream::out | std::ofstream::trunc);
    for (int k = 0; k < map.size(); ++k)
      ofs << map[k] << " ";
    ofs.close();
  }

  template<class T>
  void LoadCMapFromYaml(std::vector<T> &cmap, const std::string &pathToYaml) {
    YAML::Node map_spec = YAML::LoadFile(pathToYaml);
    // Read the map
    if (size_.size() > 0) {
      std::ifstream fsr(mappath, std::ifstream::in);
      cmap.resize(totalSize());
      for (unsigned k = 0; k < cmap.size(); ++k)
        fsr >> cmap[k];
      fsr.close();
    }
  }

  template<class T>
  void loadCMapFromFile(std::vector<T> &cmap) {
    LoadCMapFromYaml(cmap, mappath);
  }

  void loadFromYaml(std::string path_to_yaml)
  {
    //std::cout << "Path to yaml: " << path_to_yaml << std::endl;
    // Set map size
    YAML::Node map_spec = YAML::LoadFile(path_to_yaml);
    if (map_spec["mapdim"] && map_spec["origin"] && map_spec["mapres"] && map_spec["mappath"]) {
      size_ = map_spec["mapdim"].as<std::vector<int>>();
      origin_ = map_spec["origin"].as<std::vector<double>>();
      res_ = map_spec["mapres"].as<std::vector<double>>();
      double dev;
      for (unsigned k = 0; k < size_.size(); ++k) {
        if (size_[k] % 2 == 0)
          size_[k] += 1; // ensure the map size is odd!
        dev = size_[k] * res_[k] / 2;
        min_.push_back(origin_[k] - dev);
        max_.push_back(origin_[k] + dev);
        origincells_.push_back((size_[k] - 1) / 2);
      }
    } else if (map_spec["mapmin"] && map_spec["mapmax"] && map_spec["mapres"] && map_spec["mappath"]) {
      const std::vector<double> &devmin = map_spec["mapmin"].as<std::vector<double>>();
      const std::vector<double> &devmax = map_spec["mapmax"].as<std::vector<double>>();
      res_ = map_spec["mapres"].as<std::vector<double>>();
      double dev;
      for (unsigned k = 0; k < devmin.size(); ++k) {
        origin_.push_back((devmin[k] + devmax[k]) / 2);
        size_.push_back(odd_ceil((devmax[k] - devmin[k]) / res_[k]));
        dev = size_[k] * res_[k] / 2;
        min_.push_back(origin_[k] - dev);
        max_.push_back(origin_[k] + dev);
        origincells_.push_back((size_[k] - 1) / 2);
      }
    }

    // Assign storage order of map.
    if (map_spec["storage"])
      row_major_ = map_spec["storage"].as<std::string>() == "rowmajor" ? true : false;

    // Get Map Path
    std::string yaml_parent_path(boost::filesystem::path(path_to_yaml).parent_path().string());
    mappath = yaml_parent_path + "/" + map_spec["mappath"].as<std::string>();
  }

  template<class T>
  void initFromYaml(std::vector<T> &map, std::string path_to_yaml) {
    loadFromYaml(path_to_yaml);
    LoadCMapFromYaml(map, mappath);
  }

};
*/



