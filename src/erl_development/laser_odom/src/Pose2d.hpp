/**
 * @file Pose2d.h
 * @brief Simple 2D pose class.
 * @author Michael Kaess
 * @version $Id: Pose2d.h 4133 2011-03-22 20:40:38Z kaess $
 *
 * Copyright (C) 2009-2012 Massachusetts Institute of Technology.
 * Michael Kaess, Hordur Johannsson, David Rosen and John J. Leonard
 *
 * This file is part of iSAM.
 *
 * iSAM is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * iSAM is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with iSAM.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef POSE2D_HPP
#define POSE2D_HPP

#include <cmath>
#include <ostream>
#include <Eigen/Dense>

#include <tf/tf.h>
#include <angles/angles.h>

#include "Point2d.hpp"

class Pose2d {
public:
  Pose2d() : x_(0.0), y_(0.0), t_(0.0) {}
  Pose2d(double x, double y, double t)
    : x_(x), y_(y), t_(angles::normalize_angle(t)) {}
  Pose2d(const Eigen::Vector3d& vec)
    : x_(vec(0)), y_(vec(1)), t_(angles::normalize_angle(vec(2))) {}
  Pose2d(const tf::Pose& pose)
    : x_(pose.getOrigin().x()), y_(pose.getOrigin().y()),
      t_(angles::normalize_angle(tf::getYaw(pose.getRotation()))) {}

  double x() const { return x_; }
  double y() const { return y_; }
  double t() const { return t_; }

  Eigen::Vector3d vector() const {
    Eigen::Vector3d v(x_, y_, t_);
    return v;
  }

  tf::Pose tf() const {
    tf::Pose tform;
    tform.setOrigin(tf::Vector3(x(), y(), 0.0));
    tform.setRotation(tf::createQuaternionFromYaw(t()));
    return tform;
  }

  void setX(double x) { x_ = x; }
  void setY(double y) { y_ = y; }
  void setT(double t) { t_ = angles::normalize_angle(t); }

  void set(double x, double y, double t) {
    setY(y);
    setX(x);
    setT(t);
  }

  void set(const Eigen::Vector3d& v) {
    set(v(0), v(1), v(2));
  }

  /**
   * Calculate new pose b composed from this pose (a) and the odometry d.
   * Follows notation of Lu&Milios 1997.
   * \f$ b = a \oplus d \f$
   * @param d Pose difference to add.
   * @return d transformed from being local in this frame (a) to the global frame.
   */
  Pose2d oplus(const Pose2d& d) const {
    double c = cos(t());
    double s = sin(t());
    double px = x() + c*d.x() - s*d.y();
    double py = y() + s*d.x() + c*d.y();
    double pt = t() + d.t();
    return Pose2d(px,py,pt);
  }

  /**
   * Odometry d from b to this pose (a). Follows notation of
   * Lu&Milios 1997.
   * \f$ d = a \ominus b \f$
   * @param b Base frame.
   * @return Global this (a) expressed in base frame b.
   */
  Pose2d ominus(const Pose2d& b) const {
    double c = cos(b.t());
    double s = sin(b.t());
    double dx = x() - b.x();
    double dy = y() - b.y();
    double ox =  c*dx + s*dy;
    double oy = -s*dx + c*dy;
    double ot = t() - b.t();
    return Pose2d(ox,oy,ot);
  }

  /**
   * Project point into this coordinate frame.
   * @param p Point to project
   * @return Point p locally expressed in this frame.
   */
  Point2d transform_to(const Point2d& p) const {
    double c = cos(t());
    double s = sin(t());
    double dx = p.x() - x();
    double dy = p.y() - y();
    double x =  c*dx + s*dy;
    double y = -s*dx + c*dy;
    return Point2d(x,y);
  }

  Point2d transform_from(const Point2d& p) const {
    double c = cos(t());
    double s = sin(t());
    double px = x() + c*p.x() - s*p.y();
    double py = y() + s*p.x() + c*p.y();
    return Point2d(px, py);
  }

private:
  double x_;
  double y_;
  double t_;
};

std::ostream& operator<<(std::ostream& out, const Pose2d& p) {
  out << "(" << p.x() << ", " << p.y() << ", " << p.t() << ")";
  return out;
}

#endif
