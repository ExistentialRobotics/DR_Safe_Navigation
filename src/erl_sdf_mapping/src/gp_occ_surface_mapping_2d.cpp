#include "erl_sdf_mapping/gp_occ_surface_mapping_2d.hpp"

#include "erl_common/clip.hpp"

namespace erl::sdf_mapping {

    static void
    Cartesian2Polar(const Eigen::Ref<const Eigen::Vector2d> &xy, double &r, double &angle) {
        r = xy.norm();
        angle = std::atan2(xy.y(), xy.x());
    }

    bool
    GpOccSurfaceMapping2D::Update(
        const Eigen::Ref<const Eigen::VectorXd> &angles,
        const Eigen::Ref<const Eigen::VectorXd> &distances,
        const Eigen::Ref<const Eigen::Matrix23d> &pose) {

        m_changed_keys_.clear();
        auto t0 = std::chrono::high_resolution_clock::now();
        m_gp_theta_->Train(angles, distances, pose);
        auto t1 = std::chrono::high_resolution_clock::now();
        auto dt = std::chrono::duration<double, std::milli>(t1 - t0).count();
        ERL_INFO("GP theta training time: {:f} ms.", dt);
        if (m_gp_theta_->IsTrained()) {

            // clang-format off
            m_xy_perturb_ << m_setting_->perturb_delta, -m_setting_->perturb_delta, 0., 0.,
                             0., 0., m_setting_->perturb_delta, -m_setting_->perturb_delta;
            // clang-format on

            if (m_setting_->update_occupancy) {
                t0 = std::chrono::high_resolution_clock::now();
                const auto train_buffer = m_gp_theta_->GetTrainBuffer();
                UpdateOccupancy(train_buffer.vec_angles, train_buffer.vec_ranges, pose);
                t1 = std::chrono::high_resolution_clock::now();
                dt = std::chrono::duration<double, std::milli>(t1 - t0).count();
                ERL_INFO("Update occupancy time: {:f} ms.", dt);
            }

            // perform surface mapping
            t0 = std::chrono::high_resolution_clock::now();
            UpdateMapPoints();
            t1 = std::chrono::high_resolution_clock::now();
            dt = std::chrono::duration<double, std::milli>(t1 - t0).count();
            ERL_INFO("Update map points time: {:f} ms.", dt);

            t0 = std::chrono::high_resolution_clock::now();
            AddNewMeasurement();
            t1 = std::chrono::high_resolution_clock::now();
            dt = std::chrono::duration<double, std::micro>(t1 - t0).count();
            ERL_INFO("Add new measurement time: {:f} us.", dt);

            return true;
        }
        return false;
    }

    void
    GpOccSurfaceMapping2D::UpdateMapPoints() {
        if (m_quadtree_ == nullptr || !m_gp_theta_->IsTrained()) { return; }

        auto &train_buffer = m_gp_theta_->GetTrainBuffer();
        geometry::Aabb2D observed_area(train_buffer.position, train_buffer.max_distance + 3.0);
        Eigen::Vector2d &min_corner = observed_area.min();
        Eigen::Vector2d &max_corner = observed_area.max();

        auto it = m_quadtree_->BeginLeafInAabb(min_corner.x(), min_corner.y(), max_corner.x(), max_corner.y());
        auto end = m_quadtree_->EndLeafInAabb();
        const Eigen::Vector2d &sensor_position = train_buffer.position;
        double cluster_half_size = m_setting_->quadtree->resolution * std::pow(2, m_setting_->cluster_level - 1);
        double squared_dist_max = train_buffer.max_distance * train_buffer.max_distance + cluster_half_size * cluster_half_size * 2;
        for (; it != end; ++it) {
            Eigen::Vector2d cluster_position;
            m_quadtree_->KeyToCoord(it.GetKey(), m_quadtree_->GetTreeDepth() - m_setting_->cluster_level, cluster_position.x(), cluster_position.y());
            if ((cluster_position - sensor_position).squaredNorm() > squared_dist_max) { continue; }  // out of range

            std::shared_ptr<SurfaceMappingQuadtreeNode::SurfaceData> surface_data = it->GetSurfaceData();
            if (surface_data == nullptr) { continue; }  // no surface data

            Eigen::Vector2d &xy_global_old = surface_data->position;

            double distance_old;
            Eigen::Scalard angle;
            Eigen::Vector2d xy_local_old = m_gp_theta_->GlobalToLocalSe2(xy_global_old);
            Cartesian2Polar(xy_local_old, distance_old, angle[0]);
            if (angle[0] < m_setting_->gp_theta->train_buffer->valid_angle_min ||      // out of range
                angle[0] > m_setting_->gp_theta->train_buffer->valid_angle_max ||      // out of range
                distance_old < m_setting_->gp_theta->train_buffer->valid_range_min ||  // out of range
                distance_old > m_setting_->gp_theta->train_buffer->valid_range_max) {
                continue;
            }

            double occ;
            Eigen::Scalard distance_pred, distance_pred_var;
            if (!m_gp_theta_->ComputeOcc(angle, distance_old, distance_pred, distance_pred_var, occ)) { continue; }
            if (occ < m_setting_->update_map_points->min_observable_occ) { continue; }

            Eigen::Vector2d &grad_global_old = surface_data->normal;
            Eigen::Vector2d grad_local_old = m_gp_theta_->GlobalToLocalSo2(grad_global_old);

            // compute a new position for the point
            double delta = m_setting_->perturb_delta;
            Eigen::Vector2d xy_local_new = xy_local_old;
            int num_adjust_tries = 0;
            double occ_abs = std::fabs(occ);
            double distance_new = distance_old;
            while (num_adjust_tries < m_setting_->update_map_points->max_adjust_tries && occ_abs > m_setting_->update_map_points->max_surface_abs_occ) {
                // move one step
                // the direction is determined by the occupancy sign, the step GetSize is heuristically determined according to iteration.
                if (occ < 0.) {
                    // point is inside the obstacle
                    xy_local_new += grad_local_old * delta;
                } else if (occ > 0.) {
                    // point is outside the obstacle
                    xy_local_new -= grad_local_old * delta;
                }

                // test the new point
                Cartesian2Polar(xy_local_new, distance_pred[0], angle[0]);
                if (double occ_new; m_gp_theta_->ComputeOcc(angle, distance_pred[0], distance_pred, distance_pred_var, occ_new)) {
                    occ_abs = std::fabs(occ_new);
                    distance_new = distance_pred[0];
                    if (occ_abs < m_setting_->update_map_points->max_surface_abs_occ) { break; }
                    if (occ * occ_new < 0.) {
                        delta *= 0.5;  // too big, make it smaller
                    } else {
                        delta *= 1.1;
                    }
                    occ = occ_new;
                } else {
                    break;  // fail to estimate occ
                }
                ++num_adjust_tries;
            }

            // compute new gradient and uncertainty
            double occ_mean, var_distance;
            Eigen::Vector2d grad_local_new;
            if (!ComputeGradient1(xy_local_new, grad_local_new, occ_mean, var_distance)) { continue; }

            Eigen::Vector2d grad_global_new = m_gp_theta_->LocalToGlobalSo2(grad_local_new);
            double var_position_new, var_gradient_new;
            ComputeVariance(xy_local_new, grad_local_new, distance_new, var_distance, std::fabs(occ_mean), occ_abs, false, var_position_new, var_gradient_new);

            Eigen::Vector2d xy_global_new = m_gp_theta_->LocalToGlobalSe2(xy_local_new);
            if (const double var_position_old = surface_data->var_position, var_gradient_old = surface_data->var_normal;
                var_gradient_old <= m_setting_->update_map_points->max_valid_gradient_var) {
                // do bayes Update only when the old result is not too bad, otherwise, just replace it
                double var_position_sum = var_position_new + var_position_old;
                double var_gradient_sum = var_gradient_new + var_gradient_old;

                // position Update
                xy_global_new = (xy_global_new * var_position_old + xy_global_old * var_position_new) / var_position_sum;

                // gradient Update
                const double &old_x = surface_data->normal.x();
                const double &old_y = surface_data->normal.y();
                const double &new_x = grad_global_new.x();
                const double &new_y = grad_global_new.y();
                double angle_dist = std::atan2(old_x * new_y - old_y * new_x, old_x * new_x + old_y * new_y) * var_position_new / var_position_sum;
                double sin = std::sin(angle_dist);
                double cos = std::cos(angle_dist);
                // rotate grad_global_old by angle_dist
                grad_global_new.x() = cos * old_x - sin * old_y;
                grad_global_new.y() = sin * old_x + cos * old_y;

                // variance Update
                double distance = (xy_global_new - xy_global_old).norm() * 0.5;
                var_position_new = std::max((var_position_new * var_position_old) / var_position_sum + distance, m_setting_->gp_theta->sensor_range_var);
                var_gradient_new = common::ClipRange(
                    (var_gradient_new * var_gradient_old) / var_gradient_sum + distance,
                    m_setting_->compute_variance->min_gradient_var,
                    m_setting_->compute_variance->max_gradient_var);
            }
            var_position_new = std::max(var_position_new, m_setting_->update_map_points->min_position_var);
            var_gradient_new = std::max(var_gradient_new, m_setting_->update_map_points->min_gradient_var);

            // Update the surface data
            if ((var_position_new > m_setting_->update_map_points->max_bayes_position_var) &&
                (var_gradient_new > m_setting_->update_map_points->max_bayes_gradient_var)) {
                it->ResetSurfaceData();
                RecordChangedKey(it.GetKey());
                continue;  // too bad, skip
            }
            if (geometry::QuadtreeKey new_key = m_quadtree_->CoordToKey(xy_global_new.x(), xy_global_new.y()); new_key != it.GetKey()) {
                it->ResetSurfaceData();
                RecordChangedKey(it.GetKey());
                SurfaceMappingQuadtreeNode *new_node = m_quadtree_->InsertNode(new_key);
                ERL_ASSERTM(new_node != nullptr, "Failed to get the node");
                if (new_node->GetSurfaceData() != nullptr) { continue; }  // the new node is already occupied
                new_node->SetSurfaceData(surface_data);
                RecordChangedKey(new_key);
            }
            surface_data->position = xy_global_new;
            surface_data->normal = grad_global_new;
            surface_data->var_position = var_position_new;
            surface_data->var_normal = var_gradient_new;
        }
    }

    void
    GpOccSurfaceMapping2D::UpdateOccupancy(
        const Eigen::Ref<const Eigen::VectorXd> &angles,
        const Eigen::Ref<const Eigen::VectorXd> &distances,
        const Eigen::Ref<const Eigen::Matrix23d> &pose) {

        if (m_quadtree_ == nullptr) { m_quadtree_ = std::make_shared<SurfaceMappingQuadtree>(m_setting_->quadtree); }

        const Eigen::Index n = angles.size();
        const auto rotation = pose.topLeftCorner<2, 2>();
        const auto translation = pose.col(2);
        Eigen::Matrix2Xd points(2, n);
        for (long i = 0; i < n; ++i) {
            const double x = distances[i] * std::cos(angles[i]);
            const double y = distances[i] * std::sin(angles[i]);
            points(0, i) = rotation(0, 0) * x + rotation(0, 1) * y + translation[0];
            points(1, i) = rotation(1, 0) * x + rotation(1, 1) * y + translation[1];
        }
        constexpr bool parallel = false;   // no improvement
        constexpr bool lazy_eval = false;  // no improvement
        constexpr bool discrete = false;
        m_quadtree_->InsertPointCloud(points, translation, m_setting_->gp_theta->train_buffer->valid_range_max, parallel, lazy_eval, discrete);
    }

    void
    GpOccSurfaceMapping2D::AddNewMeasurement() {

        if (m_quadtree_ == nullptr) { m_quadtree_ = std::make_shared<SurfaceMappingQuadtree>(m_setting_->quadtree); }

        auto &train_buffer = m_gp_theta_->GetTrainBuffer();

        const auto n = train_buffer.Size();
        Eigen::Scalard angle, predicted_range, predicted_range_var;
        for (ssize_t i = 0; i < n; ++i) {
            angle[0] = train_buffer.vec_angles[i];
            m_gp_theta_->Test(angle, predicted_range, predicted_range_var, true);
            if (!IsValidRangeEstimation(predicted_range[0], predicted_range_var[0])) { continue; }  // uncertain point, drop it

            geometry::QuadtreeKey key = m_quadtree_->CoordToKey(train_buffer.mat_xy_global(0, i), train_buffer.mat_xy_global(1, i));
            SurfaceMappingQuadtreeNode *leaf = m_quadtree_->InsertNode(key);                       // insert the point to the tree
            if (leaf == nullptr) { continue; }                                                     // failed to insert the point, skip it
            if (m_setting_->update_occupancy && !m_quadtree_->IsNodeOccupied(leaf)) { continue; }  // the leaf is not marked as occupied, skip it
            if (leaf->GetSurfaceData() != nullptr) { continue; }                                   // the leaf already has surface data, skip it

            double occ_mean;
            Eigen::Vector2d grad_local;
            if (!ComputeGradient2(train_buffer.mat_xy_local.col(i), grad_local, occ_mean)) { continue; }  // uncertain point, drop it

            const Eigen::Vector2d grad_global = m_gp_theta_->LocalToGlobalSo2(grad_local);
            double var_position, var_gradient;
            ComputeVariance(
                train_buffer.mat_xy_local.col(i),
                grad_local,
                train_buffer.vec_ranges[i],  // distance
                0.,                          // var_distance is not used for a new point
                std::fabs(occ_mean),         // occ_mean_abs
                0.,                          // occ_abs is not used for a new point
                true,                        // new point
                var_position,
                var_gradient);

            var_position = std::max(var_position, m_setting_->update_map_points->min_position_var);
            var_gradient = std::max(var_gradient, m_setting_->update_map_points->min_gradient_var);

            // Insert the point to the tree and mark the key as changed
            leaf->SetSurfaceData(Eigen::Vector2d(train_buffer.mat_xy_global.col(i)), grad_global, var_position, var_gradient);
            RecordChangedKey(key);
        }
    }

    bool
    GpOccSurfaceMapping2D::ComputeGradient1(
        const Eigen::Ref<const Eigen::Vector2d> &xy_local,
        Eigen::Ref<Eigen::Vector2d> gradient,
        double &occ_mean,
        double &distance_var) {

        Eigen::Scalard distance_pred, var;
        double occ[4];
        occ_mean = 0.;
        double distance_sum = 0.;
        double distance_square_mean = 0.;
        gradient.x() = 0.;
        gradient.y() = 0.;

        double distance;
        Eigen::Scalard angle;
        for (int j = 0; j < 4; ++j) {
            Cartesian2Polar(xy_local + m_xy_perturb_.col(j), distance, angle[0]);
            if (m_gp_theta_->ComputeOcc(angle, distance, distance_pred, var, occ[j])) {
                occ_mean += occ[j];
            } else {
                return false;
            }
            distance_sum += distance_pred[0];
            distance_square_mean += distance_pred[0] * distance_pred[0];
        }

        occ_mean *= 0.25;
        // 4 samples in total, to calculate the unbiased variance
        // var(r) = sum((r_i - mean(r))^2) / 3. = (mean(r^2) - mean(r) * mean(r)) * 4. / 3. = (sum(r^2) - sum(r) * sum(r) * 0.25) / 3.
        // to Remove the numerical approximation's influence, let var(r) = var(r) / delta
        distance_var = (distance_square_mean - distance_sum * distance_sum * 0.25) / (3. * m_setting_->perturb_delta);

        gradient.x() = (occ[0] - occ[1]) / m_setting_->perturb_delta;
        gradient.y() = (occ[2] - occ[3]) / m_setting_->perturb_delta;

        const double gradient_norm = gradient.norm();
        if (gradient_norm <= m_setting_->zero_gradient_threshold) { return false; }  // uncertain point, drop it
        gradient.x() /= gradient_norm;
        gradient.y() /= gradient_norm;

        return true;
    }

    bool
    GpOccSurfaceMapping2D::ComputeGradient2(const Eigen::Ref<const Eigen::Vector2d> &xy_local, Eigen::Ref<Eigen::Vector2d> gradient, double &occ_mean) {
        Eigen::Scalard distance_pred, var;
        double occ[4];
        occ_mean = 0.;
        gradient.x() = 0.;
        gradient.y() = 0.;

        double distance;
        Eigen::Scalard angle;
        for (int j = 0; j < 4; ++j) {
            Cartesian2Polar(xy_local + m_xy_perturb_.col(j), distance, angle[0]);
            if (m_gp_theta_->ComputeOcc(angle, distance, distance_pred, var, occ[j])) {
                occ_mean += occ[j];
            } else {
                return false;
            }
        }

        occ_mean *= 0.25;
        gradient.x() = (occ[0] - occ[1]) / m_setting_->perturb_delta;
        gradient.y() = (occ[2] - occ[3]) / m_setting_->perturb_delta;

        const double gradient_norm = gradient.norm();
        if (gradient_norm <= m_setting_->zero_gradient_threshold) { return false; }  // uncertain point, drop it
        gradient.x() /= gradient_norm;
        gradient.y() /= gradient_norm;

        return true;
    }

    void
    GpOccSurfaceMapping2D::ComputeVariance(
        const Eigen::Ref<const Eigen::Vector2d> &xy_local,
        const Eigen::Ref<const Eigen::Vector2d> &grad_local,
        const double &distance,
        const double &distance_var,
        const double &occ_mean_abs,
        const double &occ_abs,
        const bool new_point,
        double &var_position,
        double &var_gradient) const {

        const double min_distance_var = m_setting_->compute_variance->min_distance_var;
        const double max_distance_var = m_setting_->compute_variance->max_distance_var;
        const double min_gradient_var = m_setting_->compute_variance->min_gradient_var;
        const double max_gradient_var = m_setting_->compute_variance->max_gradient_var;

        const double var_distance = common::ClipRange(distance * distance, min_distance_var, max_distance_var);
        const double cos_view_angle = -xy_local.dot(grad_local) / xy_local.norm();
        const double cos2_view_angle = std::max(cos_view_angle * cos_view_angle, 1.e-2);  // avoid zero division
        const double var_direction = (1. - cos2_view_angle) / cos2_view_angle;

        if (new_point) {
            var_position = m_setting_->compute_variance->position_var_alpha * (var_distance + var_direction);
            var_gradient = common::ClipRange(occ_mean_abs, min_gradient_var, max_gradient_var);
        } else {  // compute variance for update_map_points
            var_position = m_setting_->compute_variance->position_var_alpha * (var_distance + var_direction) + occ_abs;
            var_gradient = common::ClipRange(occ_mean_abs + distance_var, min_gradient_var, max_gradient_var) + 0.1 * var_direction;
        }
    }
}  // namespace erl::sdf_mapping
