/**
 * @file Point2d.h
 * @brief Simple 2D point class.
 * @author Michael Kaess
 * @version $Id: Point2d.h 4133 2011-03-22 20:40:38Z kaess $
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

#ifndef POINT2D_HPP
#define POINT2D_HPP

#include <Eigen/Dense>

class Point2d {
public:
  Point2d() : x_(0), y_(0) {}
  Point2d(double x, double y) : x_(x), y_(y) {}
  Point2d(const Eigen::Vector2d& vec) : x_(vec(0)), y_(vec(1)) {}

  double x() const {return x_;}
  double y() const {return y_;}
  Eigen::Vector2d vector() const { return Eigen::Vector2d(x_, y_); }

  void setX(double x) {x_ = x;}
  void setY(double y) {y_ = y;}
  void set(double x, double y) {
    setX(x);
    setY(y);
  }

private:
  double x_;
  double y_;
};

std::ostream& operator<<(std::ostream& out, const Point2d& p) {
  out << "(" <<  p.x() << ", " << p.y() << ")";
  return out;
}

#endif
