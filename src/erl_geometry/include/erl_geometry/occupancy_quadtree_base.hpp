#pragma once

#include "aabb.hpp"
#include "abstract_occupancy_quadtree.hpp"
#include "occupancy_nd_tree_setting.hpp"
#include "quadtree_impl.hpp"

#include "erl_common/random.hpp"

#include <omp.h>

#include <list>

namespace erl::geometry {

    struct OccupancyQuadtreeBaseSetting : public common::OverrideYamlable<OccupancyNdTreeSetting, OccupancyQuadtreeBaseSetting> {
        bool use_change_detection = false;
        bool use_aabb_limit = false;
        Aabb2D aabb = {};

        bool
        operator==(const NdTreeSetting& rhs) const override {
            if (OccupancyNdTreeSetting::operator==(rhs)) {
                const auto that = reinterpret_cast<const OccupancyQuadtreeBaseSetting&>(rhs);
                return use_change_detection == that.use_change_detection &&  //
                       use_aabb_limit == that.use_aabb_limit &&              //
                       aabb == that.aabb;
            }
            return false;
        }
    };

    template<class Node, class Setting>
    class OccupancyQuadtreeBase : public QuadtreeImpl<Node, AbstractOccupancyQuadtree, Setting> {
        static_assert(std::is_base_of_v<OccupancyQuadtreeNode, Node>);

    protected:
        QuadtreeKeyBoolMap m_changed_keys_ = {};
        QuadtreeKeyVectorMap m_discrete_end_point_mapping_ = {};  // buffer used for inserting point cloud to track the end points
        QuadtreeKeyVectorMap m_end_point_mapping_ = {};           // buffer used for inserting point cloud to track the end points

    public:
        OccupancyQuadtreeBase() = delete;  // no default constructor

        explicit OccupancyQuadtreeBase(const std::shared_ptr<Setting>& setting)
            : QuadtreeImpl<Node, AbstractOccupancyQuadtree, Setting>(setting) {}

        OccupancyQuadtreeBase(const OccupancyQuadtreeBase& other) = default;
        OccupancyQuadtreeBase&
        operator=(const OccupancyQuadtreeBase& other) = default;
        OccupancyQuadtreeBase(OccupancyQuadtreeBase&& other) noexcept = default;
        OccupancyQuadtreeBase&
        operator=(OccupancyQuadtreeBase&& other) noexcept = default;

        [[nodiscard]] std::shared_ptr<AbstractQuadtree>
        Clone() const override {
            std::shared_ptr<AbstractQuadtree> tree = QuadtreeImpl<Node, AbstractOccupancyQuadtree, Setting>::Clone();
            std::shared_ptr<OccupancyQuadtreeBase> occupancy_tree = std::dynamic_pointer_cast<OccupancyQuadtreeBase>(tree);
            occupancy_tree->m_changed_keys_ = m_changed_keys_;
            occupancy_tree->m_discrete_end_point_mapping_ = m_discrete_end_point_mapping_;
            occupancy_tree->m_end_point_mapping_ = m_end_point_mapping_;
            return tree;
        }

        //-- implement abstract methods
        void
        OnDeleteNodeChild(Node* node, Node* child, const QuadtreeKey& /*key*/) override {
            node->SetLogOdds(std::max(node->GetLogOdds(), child->GetLogOdds()));  // update log odds
        }

        //-- Sample position
        /**
         * Sample positions from the free space.
         */
        void
        SamplePositions(const std::size_t num_positions, std::vector<Eigen::Vector2d>& positions) const {
            positions.clear();
            positions.reserve(num_positions);
            double min_x, min_y, max_x, max_y;
            this->GetMetricMinMax(min_x, min_y, max_x, max_y);
            std::uniform_real_distribution<double> uniform_x(min_x, max_x);
            std::uniform_real_distribution<double> uniform_y(min_y, max_y);
            while (positions.size() < num_positions) {
                double x = uniform_x(common::g_random_engine);
                double y = uniform_y(common::g_random_engine);
                const Node* node = this->Search(x, y);
                if (node == nullptr || this->IsNodeOccupied(node)) { continue; }
                positions.emplace_back(x, y);
            }
        }

        //-- insert point cloud
        /**
         * Insert a point cloud in the world frame. Multiple points may fall into the same voxel that is updated only once, and occupied nodes are preferred
         * than free ones. This avoids holes and is more efficient than the plain ray insertion of InsertPointCloudRays().
         * @param points 2xN matrix of points in the world frame
         * @param sensor_origin 2D vector of the sensor origin in the world frame
         * @param max_range maximum range of the sensor. Points beyond this range are ignored. Non-positive value means no limit.
         * @param parallel whether to use parallel computation
         * @param lazy_eval whether to update the occupancy of the nodes later. If true, the occupancy is not updated until UpdateInnerOccupancy() is called.
         * @param discretize
         */
        virtual void
        InsertPointCloud(
            const Eigen::Ref<const Eigen::Matrix2Xd>& points,
            const Eigen::Ref<const Eigen::Vector2d>& sensor_origin,
            const double max_range,
            const bool parallel,
            const bool lazy_eval,
            const bool discretize) {
            static QuadtreeKeyVector free_cells, occupied_cells;  // static to avoid memory allocation
            // compute cells to update
            if (discretize) {
                ComputeDiscreteUpdateForPointCloud(points, sensor_origin, max_range, parallel, free_cells, occupied_cells);
            } else {
                ComputeUpdateForPointCloud(points, sensor_origin, max_range, parallel, free_cells, occupied_cells);
            }
            // insert data into tree
            for (const QuadtreeKey& kFreeCell: free_cells) { this->UpdateNode(kFreeCell, false, lazy_eval); }
            for (const QuadtreeKey& kOccupiedCell: occupied_cells) { this->UpdateNode(kOccupiedCell, true, lazy_eval); }
        }

        /**
         * Compute keys of the cells to update for a point cloud up to the resolution.
         * @param points 2xN matrix of points in the world frame, points falling into the same voxel are merged to the first appearance.
         * @param sensor_origin 2D vector of the sensor origin in the world frame
         * @param max_range maximum range of the sensor. Points beyond this range are ignored. Non-positive value means no limit.
         * @param parallel whether to use parallel computation
         * @param free_cells keys of the free cells to update
         * @param occupied_cells keys of the occupied cells to update
         */
        void
        ComputeDiscreteUpdateForPointCloud(
            const Eigen::Ref<const Eigen::Matrix2Xd>& points,
            const Eigen::Ref<const Eigen::Vector2d>& sensor_origin,
            const double max_range,
            const bool parallel,
            QuadtreeKeyVector& free_cells,
            QuadtreeKeyVector& occupied_cells) {

            const long num_points = points.cols();
            if (num_points == 0) { return; }

            Eigen::Matrix2Xd new_points(2, num_points);
            m_discrete_end_point_mapping_.clear();
            for (long i = 0; i < num_points; ++i) {
                const auto& kP = points.col(i);
                QuadtreeKey key = this->CoordToKey(kP[0], kP[1]);
                auto& indices = m_discrete_end_point_mapping_[key];
                if (indices.empty()) { new_points.col(static_cast<long>(m_discrete_end_point_mapping_.size()) - 1) << kP; }  // new end point!
                indices.push_back(i);
            }
            new_points.conservativeResize(2, static_cast<long>(m_discrete_end_point_mapping_.size()));
            this->ComputeUpdateForPointCloud(new_points, sensor_origin, max_range, parallel, free_cells, occupied_cells);
        }

        void
        ComputeUpdateForPointCloud(
            const Eigen::Ref<const Eigen::Matrix2Xd>& points,
            const Eigen::Ref<const Eigen::Vector2d>& sensor_origin,
            const double max_range,
            const bool parallel,
            QuadtreeKeyVector& free_cells,
            QuadtreeKeyVector& occupied_cells) {

            const long num_points = points.cols();
            if (num_points == 0) { return; }

            static QuadtreeKeySet free_cells_set;  // static to avoid memory allocation

            free_cells_set.clear();
            m_end_point_mapping_.clear();
            free_cells.clear();
            occupied_cells.clear();

            std::vector<double> ranges(num_points);
            std::vector<std::array<double, 2>> diffs(num_points);
            omp_set_num_threads(this->m_key_rays_.size());

            // insert occupied endpoint
            const auto setting = reinterpret_cast<OccupancyQuadtreeBaseSetting*>(this->m_setting_.get());
            const bool& aabb_limit = setting->use_aabb_limit;
            const Aabb2D& aabb = setting->aabb;
            for (long i = 0; i < num_points; ++i) {
                const auto& p = points.col(i);

                double& dx = diffs[i][0];
                double& dy = diffs[i][1];
                double& range = ranges[i];

                dx = p[0] - sensor_origin[0];
                dy = p[1] - sensor_origin[1];
                range = std::sqrt(dx * dx + dy * dy);

                QuadtreeKey key;
                if (aabb_limit) {                                                          // bounding box is specified
                    if ((aabb.contains(p) && (max_range < 0. || range <= max_range)) &&  // inside bounding box and range limit
                        this->CoordToKeyChecked(p[0], p[1], key)) {                      // key is valid
                        auto& indices = m_end_point_mapping_[key];
                        if (indices.empty()) { occupied_cells.push_back(key); }  // new key!
                        indices.push_back(i);
                    }
                } else {
                    if ((max_range < 0. || (range <= max_range)) &&    // range limit
                        this->CoordToKeyChecked(p[0], p[1], key)) {  // key is valid
                        auto& indices = m_end_point_mapping_[key];
                        if (indices.empty()) { occupied_cells.push_back(key); }  // new key!
                        indices.push_back(i);
                    }
                }
            }

            const double& sx = sensor_origin[0];
            const double& sy = sensor_origin[1];

            // insert free cells
#pragma omp parallel for if (parallel) default(none) shared(num_points, points, sensor_origin, max_range, sx, sy, ranges, diffs, free_cells, free_cells_set)
            for (long i = 0; i < num_points; ++i) {
                const auto& p = points.col(i);
                uint32_t thread_idx = omp_get_thread_num();
                QuadtreeKeyRay& key_ray = this->m_key_rays_[thread_idx];

                const double& range = ranges[i];
                double ex = p[0];
                double ey = p[1];
                if ((max_range >= 0.) && (range > max_range)) {  // crop ray at max_range
                    const double r = max_range / range;
                    ex = sx + diffs[i][0] * r;
                    ey = sy + diffs[i][1] * r;
                }

                if (!this->ComputeRayKeys(sx, sy, ex, ey, key_ray)) { continue; }  // key is invalid
#pragma omp critical(free_insert)
                {
                    for (auto& key: key_ray) {
                        if (m_end_point_mapping_.find(key) != m_end_point_mapping_.end()) { continue; }  // skip keys marked as occupied
                        const auto [_, new_key] = free_cells_set.emplace(key);
                        if (new_key) { free_cells.push_back(key); }
                    }
                }
            }
        }

        /**
         * Insert a point cloud ray by ray. Some cells may be updated multiple times.Benchmark shows that this is slower and less accurate than
         * InsertPointCloud.
         * @param points 2xN matrix of ray end points in the world frame.
         * @param sensor_origin 2D vector of the sensor origin in the world frame.
         * @param max_range maximum range of the sensor. Points beyond this range are ignored. Non-positive value means no limit.
         * @param parallel whether to use parallel computation
         * @param lazy_eval whether to update the occupancy of the nodes immediately. If true, the occupancy is not updated until UpdateInnerOccupancy() is
         * called.
         */
        virtual void
        InsertPointCloudRays(
            const Eigen::Ref<const Eigen::Matrix2Xd>& points,
            const Eigen::Ref<const Eigen::Vector2d>& sensor_origin,
            const double max_range,
            const bool parallel,
            const bool lazy_eval) {

            const long num_points = points.cols();
            if (num_points == 0) { return; }

            omp_set_num_threads(this->m_key_rays_.size());
#pragma omp parallel for if (parallel) default(none) shared(num_points, points, sensor_origin, max_range, lazy_eval) schedule(guided)
            for (long i = 0; i < num_points; ++i) {
                const auto& kP = points.col(i);
                uint32_t thread_idx = omp_get_thread_num();
                QuadtreeKeyRay& key_ray = this->m_key_rays_[thread_idx];
                if (!this->ComputeRayKeys(sensor_origin[0], sensor_origin[1], kP[0], kP[1], key_ray)) { continue; }

#pragma omp critical
                {
                    for (auto& key: key_ray) { UpdateNode(key, false, lazy_eval); }
                    const double range = (kP - sensor_origin).norm();
                    if (max_range <= 0. || range <= max_range) { UpdateNode(kP[0], kP[1], true, lazy_eval); }
                }
            }
        }

        //-- insert ray
        /**
         * Insert a ray from (sx, sy) to (ex, ey) into the tree. The ray is cut at max_range if it is positive.
         * @param sx metric x coordinate of the start point
         * @param sy metric y coordinate of the start point
         * @param ex metric x coordinate of the end point
         * @param ey metric y coordinate of the end point
         * @param max_range maximum range after which the ray is cut. Non-positive value means no limit.
         * @param lazy_eval whether to update the occupancy of the nodes immediately. If true, the occupancy is not updated until UpdateInnerOccupancy() is
         * called.
         * @return
         */
        virtual bool
        InsertRay(double sx, double sy, double ex, double ey, const double max_range, const bool lazy_eval) {
            const double dx = ex - sx;
            const double dy = ey - sy;
            const double range = std::sqrt(dx * dx + dy * dy);
            auto& key_ray = this->m_key_rays_[0];
            if (max_range > 0 && range > max_range) {  // cut ray at max_range
                const double r = max_range / range;
                ex = sx + dx * r;
                ey = sy + dy * r;
                if (!this->ComputeRayKeys(sx, sy, ex, ey, key_ray)) { return false; }
                for (auto& key: key_ray) { this->UpdateNode(key, false, lazy_eval); }
                return true;
            }

            if (!this->ComputeRayKeys(sx, sy, ex, ey, key_ray)) { return false; }
            for (auto& key: key_ray) { this->UpdateNode(key, false, lazy_eval); }
            this->UpdateNode(ex, ey, true, lazy_eval);
            return true;
        }

        //-- cast ray
        void
        CastRays(
            const Eigen::Ref<const Eigen::Vector2d>& position,
            const Eigen::Ref<const Eigen::Matrix2d>& rotation,
            const Eigen::Ref<const Eigen::VectorXd>& angles,
            const bool ignore_unknown,
            const double max_range,
            const bool prune_rays,
            const bool parallel,
            std::vector<long>& hit_ray_indices,
            std::vector<Eigen::Vector2d>& hit_positions,
            std::vector<const Node*>& hit_nodes) const {

            if (angles.size() == 0) { return; }
            long num_rays = angles.size();

            hit_ray_indices.clear();
            hit_positions.clear();
            hit_nodes.clear();

            hit_ray_indices.resize(num_rays);
            hit_positions.resize(num_rays);
            hit_nodes.resize(num_rays);

#pragma omp parallel for if (parallel) default(none) \
    shared(num_rays, position, rotation, angles, ignore_unknown, max_range, hit_ray_indices, hit_positions, hit_nodes)
            for (long i = 0; i < num_rays; ++i) {
                const double& kAngle = angles[i];
                Eigen::Vector2d direction(std::cos(kAngle), std::sin(kAngle));
                direction = rotation * direction;
                Eigen::Vector2d& hit_position = hit_positions[i];
                hit_nodes[i] = this->CastRay(position[0], position[1], direction[0], direction[1], ignore_unknown, max_range, hit_position[0], hit_position[1]);
            }

            absl::flat_hash_set<const Node*> hit_nodes_set;

            std::vector<long> filtered_hit_ray_indices;
            std::vector<Eigen::Vector2d> filtered_hit_positions;
            std::vector<const Node*> filtered_hit_nodes;

            filtered_hit_ray_indices.reserve(num_rays);
            filtered_hit_positions.reserve(num_rays);
            filtered_hit_nodes.reserve(num_rays);

            // remove rays that hit nothing or hit the same node if prune_rays is true
            for (long i = 0; i < num_rays; ++i) {
                const Node*& hit_node = hit_nodes[i];
                if (hit_node == nullptr) { continue; }
                if (!prune_rays || hit_nodes_set.insert(hit_node).second) {
                    filtered_hit_ray_indices.push_back(hit_ray_indices[i]);
                    filtered_hit_positions.push_back(hit_positions[i]);
                    filtered_hit_nodes.push_back(hit_node);
                }
            }

            filtered_hit_ray_indices.shrink_to_fit();
            filtered_hit_positions.shrink_to_fit();
            filtered_hit_nodes.shrink_to_fit();

            std::swap(hit_ray_indices, filtered_hit_ray_indices);
            std::swap(hit_positions, filtered_hit_positions);
            std::swap(hit_nodes, filtered_hit_nodes);
        }

        void
        CastRays(
            const Eigen::Ref<const Eigen::Matrix2Xd>& positions,
            const Eigen::Ref<const Eigen::Matrix2Xd>& directions,
            const bool ignore_unknown,
            const double max_range,
            const bool prune_rays,
            const bool parallel,
            std::vector<long>& hit_ray_indices,
            std::vector<Eigen::Vector2d>& hit_positions,
            std::vector<const Node*>& hit_nodes) const {
            long num_rays = 0;
            if (positions.cols() != 1 && directions.cols() != 1) {
                ERL_ASSERTM(positions.cols() == directions.cols(), "positions.cols() != directions.cols() when both are not 1.");
                num_rays = positions.cols();
                hit_positions.resize(num_rays);
                hit_nodes.resize(num_rays, nullptr);
#pragma omp parallel for if (parallel) default(none) shared(num_rays, positions, directions, ignore_unknown, max_range, hit_positions, hit_nodes)
                for (long i = 0; i < num_rays; ++i) {
                    hit_nodes[i] = CastRay(
                        positions(0, i),
                        positions(1, i),
                        directions(0, i),
                        directions(1, i),
                        ignore_unknown,
                        max_range,
                        hit_positions[i][0],
                        hit_positions[i][1]);
                }
            }
            if (positions.cols() == 1) {
                num_rays = directions.cols();
                hit_positions.resize(num_rays);
                hit_nodes.resize(num_rays, nullptr);
#pragma omp parallel for if (parallel) default(none) shared(num_rays, positions, directions, ignore_unknown, max_range, hit_positions, hit_nodes)
                for (long i = 0; i < num_rays; ++i) {
                    hit_nodes[i] = CastRay(
                        positions(0, 0),
                        positions(1, 0),
                        directions(0, i),
                        directions(1, i),
                        ignore_unknown,
                        max_range,
                        hit_positions[i][0],
                        hit_positions[i][1]);
                }
            }
            if (directions.cols() == 1) {
                num_rays = positions.cols();
                hit_positions.resize(num_rays);
                hit_nodes.resize(num_rays, nullptr);
#pragma omp parallel for if (parallel) default(none) shared(num_rays, positions, directions, ignore_unknown, max_range, hit_positions, hit_nodes)
                for (long i = 0; i < num_rays; ++i) {
                    hit_nodes[i] = CastRay(
                        positions(0, i),
                        positions(1, i),
                        directions(0, 0),
                        directions(1, 0),
                        ignore_unknown,
                        max_range,
                        hit_positions[i][0],
                        hit_positions[i][1]);
                }
            }

            if (num_rays == 0) { return; }

            absl::flat_hash_set<const Node*> hit_nodes_set;

            std::vector<long> filtered_hit_ray_indices;
            std::vector<Eigen::Vector2d> filtered_hit_positions;
            std::vector<const Node*> filtered_hit_nodes;
            filtered_hit_ray_indices.reserve(num_rays);
            filtered_hit_positions.reserve(num_rays);
            filtered_hit_nodes.reserve(num_rays);

            // remove rays that hit nothing or hit the same node if prune_rays is true
            for (long i = 0; i < num_rays; ++i) {
                const Node*& hit_node = hit_nodes[i];
                if (hit_node == nullptr) { continue; }
                if (!prune_rays || hit_nodes_set.insert(hit_node).second) {
                    filtered_hit_ray_indices.push_back(i);
                    filtered_hit_positions.push_back(hit_positions[i]);
                    filtered_hit_nodes.push_back(hit_node);
                }
            }

            filtered_hit_ray_indices.shrink_to_fit();
            filtered_hit_positions.shrink_to_fit();
            filtered_hit_nodes.shrink_to_fit();

            std::swap(hit_ray_indices, filtered_hit_ray_indices);
            std::swap(hit_positions, filtered_hit_positions);
            std::swap(hit_nodes, filtered_hit_nodes);
        }

        /**
         * Cast a ray starting from (px, py) along (vx, vy) and get the hit surface point (ex, ey) if the ray hits one.
         * @param px metric x coordinate of the start point
         * @param py metric y coordinate of the start point
         * @param vx x component of the ray direction
         * @param vy y component of the ray direction
         * @param ignore_unknown whether unknown cells are ignored, i.e. treated as free. If false, the ray casting aborts when an unknown cell is hit and
         * returns false.
         * @param max_range maximum range after which the ray casting is aborted. Non-positive value means no limit.
         * @param ex metric x coordinate of the hit leaf cell
         * @param ey metric y coordinate of the hit leaf cell
         * @return node pointer if the ray hits an occupied cell, nullptr otherwise.
         */
        const Node*
        CastRay(double px, double py, double vx, double vy, const bool ignore_unknown, const double max_range, double& ex, double& ey) const {
            // Similar to QuadtreeImpl::ComputeRayKeys, but with extra hitting checks

            QuadtreeKey current_key;
            if (!this->CoordToKeyChecked(px, py, current_key)) {
                ERL_WARN("Ray starting from ({}, {}) is out of range.\n", px, py);
                return nullptr;
            }

            // initialization
            const Node* starting_node = this->Search(current_key);
            if (starting_node != nullptr) {
                if (this->IsNodeOccupied(starting_node)) {  // (px, py) is in occupied
                    this->KeyToCoord(current_key, ex, ey);
                    return starting_node;
                }
            } else if (!ignore_unknown) {  // (px, py) is in unknown
                this->KeyToCoord(current_key, ex, ey);
                return nullptr;
            }

            const double v_norm = std::sqrt(vx * vx + vy * vy);
            vx /= v_norm;
            vy /= v_norm;
            const bool max_range_set = max_range > 0.;

            // compute step direction
            int step[2];
            if (vx > 0) {
                step[0] = 1;
            } else if (vx < 0) {
                step[0] = -1;
            } else {
                step[0] = 0;
            }
            if (vy > 0) {
                step[1] = 1;
            } else if (vy < 0) {
                step[1] = -1;
            } else {
                step[1] = 0;
            }
            if (step[0] == 0 && step[1] == 0) {
                ERL_WARN("Ray casting in direction (0, 0) is impossible!");
                return nullptr;
            }

            // compute t_max and t_delta
            const double& resolution = this->m_setting_->resolution;
            double t_max[2];
            double t_delta[2];
            if (step[0] == 0) {
                t_max[0] = std::numeric_limits<double>::infinity();
                t_delta[0] = std::numeric_limits<double>::infinity();
            } else {
                const double voxel_border = this->KeyToCoord(current_key[0]) + static_cast<double>(step[0]) * 0.5 * resolution;
                t_max[0] = (voxel_border - px) / vx;
                t_delta[0] = resolution / std::abs(vx);
            }
            if (step[1] == 0) {
                t_max[1] = std::numeric_limits<double>::infinity();
                t_delta[1] = std::numeric_limits<double>::infinity();
            } else {
                const double voxel_border = this->KeyToCoord(current_key[1]) + static_cast<double>(step[1]) * 0.5 * resolution;
                t_max[1] = (voxel_border - py) / vy;
                t_delta[1] = resolution / std::abs(vy);
            }

            // incremental phase
            const double max_range_sq = max_range * max_range;
            const long max_key_val = (this->m_tree_key_offset_ << 1) - 1;
            while (true) {
                if (t_max[0] < t_max[1]) {
                    t_max[0] += t_delta[0];
                    const long next_key_val = static_cast<long>(current_key[0]) + step[0];
                    // check overflow
                    if ((step[0] < 0 && next_key_val <= 0) || (step[0] > 0 && next_key_val >= max_key_val)) {
                        ERL_DEBUG("x coordinate hits boundary, aborting ray cast.");
                        current_key[0] = next_key_val < 0 ? 0 : max_key_val;  // set to boundary
                        this->KeyToCoord(current_key, ex, ey);
                        return nullptr;
                    }
                    current_key[0] = next_key_val;
                } else {
                    t_max[1] += t_delta[1];
                    const long next_key_val = static_cast<long>(current_key[1]) + step[1];
                    // check overflow
                    if ((step[1] < 0 && next_key_val <= 0) || (step[1] > 0 && next_key_val >= max_key_val)) {
                        ERL_DEBUG("y coordinate hits boundary, aborting ray cast.");
                        current_key[1] = next_key_val < 0 ? 0 : max_key_val;
                        this->KeyToCoord(current_key, ex, ey);
                        return nullptr;
                    }
                    current_key[1] = next_key_val;
                }

                // generate world coordinates from key
                this->KeyToCoord(current_key, ex, ey);
                // check if max_range is reached
                if (max_range_set) {
                    const double dx = ex - px;
                    const double dy = ey - py;
                    if ((dx * dx + dy * dy) > max_range_sq) { return nullptr; }
                }
                // search node of the new key
                const Node* current_node = this->Search(current_key);
                if (current_node != nullptr) {
                    if (this->IsNodeOccupied(current_node)) { return current_node; }
                } else if (!ignore_unknown) {
                    return nullptr;
                }
            }
        }

        //-- trace ray
        [[maybe_unused]] [[nodiscard]] QuadtreeKeyBoolMap::const_iterator
        BeginChangedKey() const {
            if (!reinterpret_cast<OccupancyQuadtreeBaseSetting*>(this->m_setting_.get())->use_change_detection) {
                ERL_WARN("use_change_detection is false in setting. No changes are tracked.");
                return m_changed_keys_.end();
            }
            return m_changed_keys_.begin();
        }

        [[maybe_unused]] [[nodiscard]] QuadtreeKeyBoolMap::const_iterator
        EndChangedKey() const {
            return m_changed_keys_.end();
        }

        [[maybe_unused]] void
        ClearChangedKeys() {
            m_changed_keys_.clear();
        }

        //-- update nodes' occupancy
        /**
         * Update the node at the given key with the given log-odds delta.
         * @param x
         * @param y
         * @param occupied
         * @param lazy_eval whether update of inner nodes is omitted and only leaf nodes are updated. This speeds up the intersection, but you need to call
         * UpdateInnerOccupancy() after all updates are done.
         * @return
         */
        Node*
        UpdateNode(double x, double y, const bool occupied, const bool lazy_eval) {
            QuadtreeKey key;
            if (!this->CoordToKeyChecked(x, y, key)) { return nullptr; }
            return UpdateNode(key, occupied, lazy_eval);
        }

        /**
         * Update occupancy measurement of a given node
         * @param key of the node to update
         * @param occupied whether the node is observed occupied or not
         * @param lazy_eval whether update of inner nodes is omitted and only leaf nodes are updated. This speeds up the intersection, but you need to call
         * UpdateInnerOccupancy() after all updates are done.
         */
        Node*
        UpdateNode(const QuadtreeKey& key, const bool occupied, const bool lazy_eval) {
            const auto* setting = reinterpret_cast<OccupancyQuadtreeBaseSetting*>(this->m_setting_.get());
            const float log_odds_delta = occupied ? setting->log_odd_hit : setting->log_odd_miss;
            return UpdateNode(key, log_odds_delta, lazy_eval);
        }

        Node*
        UpdateNode(double x, double y, const float log_odds_delta, const bool lazy_eval) {
            QuadtreeKey key;
            if (!this->CoordToKeyChecked(x, y, key)) { return nullptr; }
            return UpdateNode(key, log_odds_delta, lazy_eval);
        }

        Node*
        UpdateNode(const QuadtreeKey& key, const float log_odds_delta, const bool lazy_eval) {
            auto leaf = const_cast<Node*>(this->Search(key));
            auto log_odds_delta_double = static_cast<double>(log_odds_delta);
            // early abort, no change will happen: node already at threshold or its log-odds is locked.
            if (leaf) {
                if (!leaf->AllowUpdateLogOdds(log_odds_delta_double)) { return leaf; }
                const auto* setting = reinterpret_cast<OccupancyQuadtreeBaseSetting*>(this->m_setting_.get());
                if (log_odds_delta_double >= 0 && leaf->GetLogOdds() >= setting->log_odd_max) { return leaf; }
                if (log_odds_delta_double <= 0 && leaf->GetLogOdds() <= setting->log_odd_min) { return leaf; }
            }

            const bool create_root = this->m_root_ == nullptr;
            if (create_root) {
                this->m_root_ = std::make_shared<Node>();
                ++this->m_tree_size_;
                ERL_DEBUG_ASSERT(this->m_tree_size_ == 1, "tree size is not 1 after root creation.");
            }
            return static_cast<Node*>(this->UpdateNodeRecurs(this->m_root_.get(), create_root, key, log_odds_delta_double, lazy_eval));
        }

    private:
        Node*
        UpdateNodeRecurs(Node* node, const bool node_just_created, const QuadtreeKey& key, const double log_odds_delta, const bool lazy_eval) {
            ERL_DEBUG_ASSERT(node != nullptr, "node is nullptr.");

            const uint32_t depth = node->GetDepth();
            if (const uint32_t& tree_depth = this->m_setting_->tree_depth; depth < tree_depth) {  // follow down to last level
                bool created_node = false;
                int pos = QuadtreeKey::ComputeChildIndex(key, tree_depth - 1 - depth);
                if (!node->HasChild(pos)) {                            // child node does not exist
                    if (!node->HasAnyChild() && !node_just_created) {  // current node has no child and is not new
                        this->ExpandNode(node);                        // expand pruned node
                    } else {
                        this->CreateNodeChild(node, pos);
                        created_node = true;
                    }
                }

                if (lazy_eval) { return this->UpdateNodeRecurs(this->GetNodeChild(node, pos), created_node, key, log_odds_delta, lazy_eval); }
                Node* returned_node = this->UpdateNodeRecurs(this->GetNodeChild(node, pos), created_node, key, log_odds_delta, lazy_eval);
                if (this->PruneNode(node)) {
                    returned_node = node;  // returned_node is pruned, return its parent instead
                } else {
                    this->UpdateInnerNodeOccupancy(node);
                }
                return returned_node;
            }
            // last level
            if (reinterpret_cast<OccupancyQuadtreeBaseSetting*>(this->m_setting_.get())->use_change_detection) {
                bool occ_before = this->IsNodeOccupied(node);
                UpdateNodeLogOdds(node, log_odds_delta);
                if (node_just_created) {
                    m_changed_keys_.emplace(key, true);
                } else if (occ_before != this->IsNodeOccupied(node)) {  // occupancy changed, track it
                    const auto it = m_changed_keys_.find(key);
                    if (it == m_changed_keys_.end()) {  // not found
                        m_changed_keys_.emplace(key, false);
                    } else if (!it->second) {
                        m_changed_keys_.erase(it);
                    }
                }
            } else {
                UpdateNodeLogOdds(node, log_odds_delta);
            }
            return node;
        }

    protected:
        void
        UpdateNodeLogOdds(Node* node, float log_odd_delta) {
            node->AddLogOdds(log_odd_delta);
            const float l = node->GetLogOdds();
            const auto* setting = reinterpret_cast<OccupancyQuadtreeBaseSetting*>(this->m_setting_.get());
            const double& log_odd_min = setting->log_odd_min;
            const double& log_odd_max = setting->log_odd_max;
            if (l < log_odd_min) {
                node->SetLogOdds(log_odd_min);
                return;
            }

            if (l > log_odd_max) { node->SetLogOdds(log_odd_max); }
        }

    public:
        void
        UpdateInnerOccupancy() {
            if (this->m_root_ == nullptr) { return; }
            UpdateInnerOccupancyRecurs(this->m_root_.get(), 0);
        }

    protected:
        void
        UpdateInnerOccupancyRecurs(Node* node, uint32_t depth) {
            ERL_DEBUG_ASSERT(node != nullptr, "node is nullptr.");
            if (!node->HasAnyChild()) { return; }
            // only recurse and update for inner nodes
            if (depth < this->m_setting_->tree_depth) {
                for (int i = 0; i < 4; ++i) {
                    Node* child = this->GetNodeChild(node, i);
                    if (child == nullptr) { continue; }
                    UpdateInnerOccupancyRecurs(child, depth + 1);
                }
            }
            UpdateInnerNodeOccupancy(node);
        }

        void
        UpdateInnerNodeOccupancy(Node* node) {
            node->SetLogOdds(node->GetMaxChildLogOdds());
        }

    public:
        /**
         * Set all nodes' log odds according to their current max likelihood of occupancy.
         */
        void
        ToMaxLikelihood() override {
            if (this->m_root_ == nullptr) { return; }
            std::list<Node*> stack;
            stack.emplace_back(static_cast<Node*>(this->m_root_.get()));
            const auto* setting = reinterpret_cast<OccupancyQuadtreeBaseSetting*>(this->m_setting_.get());
            const double& kLogOddMin = setting->log_odd_min;
            const double& kLogOddMax = setting->log_odd_max;
            while (!stack.empty()) {
                Node* node = stack.back();
                stack.pop_back();

                if (this->IsNodeOccupied(node)) {
                    node->SetLogOdds(kLogOddMax);
                } else {
                    node->SetLogOdds(kLogOddMin);
                }

                if (node->HasAnyChild()) {
                    for (uint32_t i = 0; i < 4; ++i) {
                        auto child = this->GetNodeChild(node, i);
                        if (child == nullptr) { continue; }
                        stack.emplace_back(child);
                    }
                }
            }
        }

    protected:
        //--file IO
        std::istream&
        ReadBinaryData(std::istream& s) override {
            if (this->m_root_ != nullptr) {
                ERL_WARN("Trying to read into an existing tree.");
                return s;
            }

            this->m_root_ = std::make_shared<Node>();
            this->m_tree_size_ = 1;
            char child_record;

            std::list<std::pair<Node*, bool>> stack;  // node, is_new_node
            stack.emplace_back(this->m_root_.get(), true);

            const auto* setting = reinterpret_cast<OccupancyQuadtreeBaseSetting*>(this->m_setting_.get());
            const double& log_odd_min = setting->log_odd_min;
            const double& log_odd_max = setting->log_odd_max;

            while (!stack.empty()) {
                auto& top = stack.back();
                Node* node = top.first;
                bool& is_new_node = top.second;

                if (!is_new_node) {
                    node->SetLogOdds(node->GetMaxChildLogOdds());
                    stack.pop_back();
                    continue;
                }

                is_new_node = false;
                s.read(&child_record, sizeof(char));
                std::bitset<8> child(static_cast<unsigned long long>(child_record));
                bool has_inner_node_child = false;
                for (int i = 3; i >= 0; --i) {
                    // 0b10: free leaf
                    // 0b01: occupied leaf
                    // 0b11: inner node
                    const bool bit0 = child[i * 2];
                    const bool bit1 = child[i * 2 + 1];
                    Node* child_node = nullptr;
                    if (bit0) {
                        if (bit1) {  // 0b11, inner node
                            child_node = this->CreateNodeChild(node, i);
                            child_node->SetLogOdds(-200);
                            has_inner_node_child = true;
                            stack.emplace_back(child_node, true);
                        } else {  // 0b01, occupied leaf
                            child_node = this->CreateNodeChild(node, i);
                            child_node->SetLogOdds(log_odd_max);
                        }
                    } else if (bit1) {  // 0b10, free leaf
                        child_node = this->CreateNodeChild(node, i);
                        child_node->SetLogOdds(log_odd_min);
                    }
                    // else: 0b00, child is unknown, we leave it uninitialized
                }

                if (!has_inner_node_child) {
                    node->SetLogOdds(node->GetMaxChildLogOdds());
                    stack.pop_back();
                }
            }

            return s;
        }

        std::ostream&
        WriteBinaryData(std::ostream& s) const override {
            if (this->m_root_ == nullptr) {
                ERL_WARN("Trying to write an empty tree.");
                return s;
            }

            std::list<const Node*> nodes_stack;  // node
            nodes_stack.push_back(this->m_root_.get());

            while (!nodes_stack.empty()) {
                const Node* node = nodes_stack.back();
                nodes_stack.pop_back();

                std::bitset<8> child;
                for (int i = 3; i >= 0; --i) {
                    const Node* child_node = this->GetNodeChild(node, i);
                    if (child_node == nullptr) {  // 0b00, unknown
                        child[i * 2] = false;
                        child[i * 2 + 1] = false;
                        continue;
                    }

                    if (child_node->HasAnyChild()) {  // 0b11, inner node
                        child[i * 2] = true;
                        child[i * 2 + 1] = true;
                        nodes_stack.push_back(child_node);
                        continue;
                    }

                    if (this->IsNodeOccupied(child_node)) {  // 0b01, occupied leaf
                        child[i * 2] = true;
                        child[i * 2 + 1] = false;
                        continue;
                    }

                    // 0b10, free leaf
                    child[i * 2] = false;
                    child[i * 2 + 1] = true;
                }
                char child_record = static_cast<char>(child.to_ulong());
                s.write(&child_record, sizeof(char));
            }

            return s;
        }
    };
}  // namespace erl::geometry

template<>
struct YAML::convert<erl::geometry::OccupancyQuadtreeBaseSetting> {
    static Node
    encode(const erl::geometry::OccupancyQuadtreeBaseSetting& rhs) {
        Node node = convert<erl::geometry::OccupancyNdTreeSetting>::encode(rhs);
        node["use_change_detection"] = rhs.use_change_detection;
        node["use_aabb_limit"] = rhs.use_aabb_limit;
        node["aabb"] = rhs.aabb;
        return node;
    }

    static bool
    decode(const Node& node, erl::geometry::OccupancyQuadtreeBaseSetting& rhs) {
        if (!node.IsMap()) { return false; }
        if (!convert<erl::geometry::OccupancyNdTreeSetting>::decode(node, rhs)) { return false; }
        rhs.use_change_detection = node["use_change_detection"].as<bool>();
        rhs.use_aabb_limit = node["use_aabb_limit"].as<bool>();
        rhs.aabb = node["aabb"].as<erl::geometry::Aabb2D>();
        return true;
    }
};  // namespace YAML
