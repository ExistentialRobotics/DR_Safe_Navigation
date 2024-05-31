#pragma once

#include "erl_common/eigen.hpp"

namespace erl::geometry {

    /**
     * find the nearest point on the segment (x_1, y_1) -- (x_2, y_2) to point (x_0, y_0)
     * line1: (x_1, y_1), (x_2, y_2)
     * line2: (x_0, y_0) of direction d=(y_2-y_1, x_1-x_2)
     * lam * (x_1, y_1) + (1-lam) * (x_2, y_2) == (x_0, y_0) + m_tree_ * d
     * if intersected, lam in (0, 1); |m_tree_| is the distance
     * if lam < 0, (x_2, y_2) is the closest point; if lam > 1, (x_1, y_1) is the closest point.
     *
     * @param x_0: point x coordinate
     * @param y_0: point y coordinate
     * @param x_1: x coordinate of line vertex 1
     * @param y_1: y coordinate of line vertex 1
     * @param x_2: x coordinate of line vertex 2
     * @param y_2: y coordinate of line vertex 2
     * @return
     */
    inline double
    ComputeNearestDistanceFromPointToLineSegment2D(
        const double &x_0,
        const double &y_0,
        const double &x_1,
        const double &y_1,
        const double &x_2,
        const double &y_2) {

        const double dx_20 = x_2 - x_0;
        const double dx_21 = x_2 - x_1;
        const double dy_20 = y_2 - y_0;
        const double dy_21 = y_2 - y_1;
        const double d = dx_21 * dx_21 + dy_21 * dy_21;
        const double lam = (dx_20 * dx_21 + dy_20 * dy_21) / d;

        double dist = (dy_21 * dx_20 - dy_20 * dx_21) / std::sqrt(d);
        dist = std::abs(dist);

        if (lam > 1.) {
            const double dx_10 = x_1 - x_0;
            const double dy_10 = y_1 - y_0;
            dist = std::sqrt(dx_10 * dx_10 + dy_10 * dy_10);
        } else if (lam < 0.) {
            dist = std::sqrt(dx_20 * dx_20 + dy_20 * dy_20);
        }

        return dist;
    }

    /**
     * find the intersection between ray [p_0, d] and segment [p_1, p_2]
     * @param p_0: ray start point
     * @param d: ray direction, assumed normalized
     * @param p_1: point 1 on the line
     * @param p_2: point 2 on the line
     * @param lam: the intersection point is lam * p_1 + (1 - lam) * p_2
     * @param dist: distance from p_0 to the line along direction d
     */
    inline void
    ComputeIntersectionBetweenRayAndSegment2D(
        const Eigen::Ref<const Eigen::Vector2d> &p_0,
        const Eigen::Ref<const Eigen::Vector2d> &d,
        const Eigen::Ref<const Eigen::Vector2d> &p_1,
        const Eigen::Ref<const Eigen::Vector2d> &p_2,
        double &lam,
        double &dist) {

        Eigen::Vector2d v_21 = p_2 - p_1;
        Eigen::Vector2d v_20 = p_2 - p_0;

        const double tmp = v_21.x() * d.y() - v_21.y() * d.x();  // tmp = (p_2 - p_1).cross(d)
        if (std::abs(tmp) < 1.e-10) {
            lam = std::numeric_limits<double>::infinity();
            dist = std::numeric_limits<double>::infinity();
            return;
        }
        lam = (v_20.x() * d.y() - v_20.y() * d.x()) / tmp;         // (p_2 - p_0).cross(d) / tmp
        dist = (v_21.x() * v_20.y() - v_21.y() * v_20.x()) / tmp;  // dist = (p_2 - p_1).cross(p_2 - p_0) / tmp
    }

    /**
     * find the intersection between ray [p, r] and axis-aligned bounding box [box_min, box_max]
     * @param p            ray start point
     * @param r_inv        1 / r considering performance of multiple computations with the same ray direction
     * @param box_min      box min point
     * @param box_max      box max point
     * @param d1           output distance from p to the first intersection point (closest one if p is outside the box, forward one if p is inside the box)
     * @param d2           output distance from p to the second intersection point (farthest one if p is outside the box, backward one if p is inside the box)
     * @param intersected  output whether the ray intersects the box
     *
     * @refitem https://tavianator.com/fast-branchless-raybounding-box-intersections/
     * @refitem https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
     * @refitem https://en.wikipedia.org/wiki/Cohen%E2%80%93Sutherland_algorithm
     * @refitem https://en.wikipedia.org/wiki/Liang%E2%80%93Barsky_algorithm
     * @refitem https://en.wikipedia.org/wiki/Cyrus%E2%80%93Beck_algorithm
     * @refitem https://en.wikipedia.org/wiki/Nicholl%E2%80%93Lee%E2%80%93Nicholl_algorithm
     * @refitem https://en.wikipedia.org/wiki/Line_clipping#Fast_clipping
     */
    inline void
    ComputeIntersectionBetweenRayAndAabb2D(
        const Eigen::Ref<const Eigen::Vector2d> &p,
        const Eigen::Ref<const Eigen::Vector2d> &r_inv,
        const Eigen::Ref<const Eigen::Vector2d> &box_min,
        const Eigen::Ref<const Eigen::Vector2d> &box_max,
        double &d1,
        double &d2,
        bool &intersected) {

        double tx_1, tx_2, ty_1, ty_2;
        if (p[0] == box_min[0]) {
            tx_1 = 0;
        } else {
            tx_1 = (box_min[0] - p[0]) * r_inv[0];
        }
        if (p[0] == box_max[0]) {
            tx_2 = 0;
        } else {
            tx_2 = (box_max[0] - p[0]) * r_inv[0];
        }
        double t_min = std::min(tx_1, tx_2);
        double t_max = std::max(tx_1, tx_2);

        if (p[1] == box_min[1]) {
            ty_1 = 0;
        } else {
            ty_1 = (box_min[1] - p[1]) * r_inv[1];
        }
        if (p[1] == box_max[1]) {
            ty_2 = 0;
        } else {
            ty_2 = (box_max[1] - p[1]) * r_inv[1];
        }
        t_min = std::max(t_min, std::min(ty_1, ty_2));
        t_max = std::min(t_max, std::max(ty_1, ty_2));

        intersected = t_max >= t_min;
        d1 = std::numeric_limits<double>::infinity();
        d2 = std::numeric_limits<double>::infinity();
        if (intersected) {
            if (p[0] < box_min[0] || p[0] > box_max[0] || p[1] < box_min[1] || p[1] > box_max[1]) {  // ray start point is outside the box
                if (t_min >= 0) {                                                                    // forward intersection
                    d1 = t_min;                                                                      // first intersection point
                    d2 = t_max;                                                                      // second intersection point
                } else {                                                                             // backward intersection
                    d1 = t_max;                                                                      // first intersection point
                    d2 = t_min;                                                                      // second intersection point
                }
            } else {         // ray start point is inside the box
                d1 = t_max;  // forward intersection
                d2 = t_min;  // backward intersection
            }
        }
    }

    /**
     * find the intersection between ray [p, r] and axis-aligned bounding box [box_min, box_max]
     * @param p            ray start point
     * @param r_inv        1 / r considering performance of multiple computations with the same ray direction
     * @param box_min      box min point
     * @param box_max      box max point
     * @param d1           output distance from p to the first intersection point (closest one if p is outside the box, forward one if p is inside the box)
     * @param d2           output distance from p to the second intersection point (farthest one if p is outside the box, backward one if p is inside the box)
     * @param intersected  output whether the ray intersects the box
     */
    inline void
    ComputeIntersectionBetweenRayAndAabb3D(
        const Eigen::Ref<const Eigen::Vector3d> &p,
        const Eigen::Ref<const Eigen::Vector3d> &r_inv,
        const Eigen::Ref<const Eigen::Vector3d> &box_min,
        const Eigen::Ref<const Eigen::Vector3d> &box_max,
        double &d1,
        double &d2,
        bool &intersected) {

        double tx_1, tx_2, ty_1, ty_2, tz_1, tz_2;
        if (p[0] == box_min[0]) {
            tx_1 = 0;
        } else {
            tx_1 = (box_min[0] - p[0]) * r_inv[0];
        }
        if (p[0] == box_max[0]) {
            tx_2 = 0;
        } else {
            tx_2 = (box_max[0] - p[0]) * r_inv[0];
        }
        double t_min = std::min(tx_1, tx_2);
        double t_max = std::max(tx_1, tx_2);

        if (p[1] == box_min[1]) {
            ty_1 = 0;
        } else {
            ty_1 = (box_min[1] - p[1]) * r_inv[1];
        }
        if (p[1] == box_max[1]) {
            ty_2 = 0;
        } else {
            ty_2 = (box_max[1] - p[1]) * r_inv[1];
        }
        t_min = std::max(t_min, std::min(ty_1, ty_2));
        t_max = std::min(t_max, std::max(ty_1, ty_2));

        if (p[2] == box_min[2]) {
            tz_1 = 0;
        } else {
            tz_1 = (box_min[2] - p[2]) * r_inv[2];
        }
        if (p[2] == box_max[2]) {
            tz_2 = 0;
        } else {
            tz_2 = (box_max[2] - p[2]) * r_inv[2];
        }
        t_min = std::max(t_min, std::min(tz_1, tz_2));
        t_max = std::min(t_max, std::max(tz_1, tz_2));

        intersected = t_max >= t_min;
        d1 = std::numeric_limits<double>::infinity();
        d2 = std::numeric_limits<double>::infinity();
        if (intersected) {
            if (p[0] < box_min[0] || p[0] > box_max[0] ||  // check x
                p[1] < box_min[1] || p[1] > box_max[1] ||  // check y
                p[2] < box_min[2] || p[2] > box_max[2]) {  // ray start point is outside the box
                if (t_min >= 0) {                          // forward intersection
                    d1 = t_min;                            // first intersection point
                    d2 = t_max;                            // second intersection point
                } else {                                   // backward intersection
                    d1 = t_max;                            // first intersection point
                    d2 = t_min;                            // second intersection point
                }
            } else {         // ray start point is inside the box
                d1 = t_max;  // forward intersection
                d2 = t_min;  // backward intersection
            }
        }
    }

    inline std::vector<Eigen::Matrix4d>
    ConvertPath2dTo3d(const Eigen::Ref<const Eigen::Matrix3Xd> &path_2d, const double &z) {
        std::vector<Eigen::Matrix4d> path_3d(path_2d.cols(), Eigen::Matrix4d::Identity());
        for (long i = 0; i < path_2d.cols(); ++i) {
            Eigen::Matrix4d &mat = path_3d[i];
            mat.coeffRef(0, 0) = std::cos(path_2d(2, i));
            mat.coeffRef(0, 1) = -std::sin(path_2d(2, i));
            mat.coeffRef(1, 0) = -mat(0, 1);
            mat.coeffRef(1, 1) = mat(0, 0);
            mat.coeffRef(0, 3) = path_2d(0, i);
            mat.coeffRef(1, 3) = path_2d(1, i);
            mat.coeffRef(2, 3) = z;
        }
        return path_3d;
    }

}  // namespace erl::geometry
