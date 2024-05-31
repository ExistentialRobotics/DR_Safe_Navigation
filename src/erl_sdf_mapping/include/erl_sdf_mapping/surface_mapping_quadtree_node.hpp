#pragma once
#include "erl_common/eigen.hpp"
#include "erl_geometry/occupancy_quadtree_node.hpp"

namespace erl::sdf_mapping {

    class SurfaceMappingQuadtreeNode : public geometry::OccupancyQuadtreeNode {

    public:
        struct SurfaceData {
            Eigen::Vector2d position = {0., 0.};
            Eigen::Vector2d normal = {0., 0.};
            double var_position = 0.;
            double var_normal = 0.;

            SurfaceData() = default;

            SurfaceData(Eigen::Vector2d position, Eigen::Vector2d normal, const double var_position, const double var_normal)
                : position(std::move(position)),
                  normal(std::move(normal)),
                  var_position(var_position),
                  var_normal(var_normal) {}

            SurfaceData(const SurfaceData &other) = default;
            SurfaceData &
            operator=(const SurfaceData &other) = default;
            SurfaceData(SurfaceData &&other) noexcept = default;
            SurfaceData &
            operator=(SurfaceData &&other) noexcept = default;

            [[nodiscard]] bool
            operator==(const SurfaceData &other) const {
                return position == other.position && normal == other.normal && var_position == other.var_position && var_normal == other.var_normal;
            }

            [[nodiscard]] bool
            operator!=(const SurfaceData &other) const {
                return !(*this == other);
            }
        };

    private:
        std::shared_ptr<SurfaceData> m_data_ = nullptr;

    public:
        explicit SurfaceMappingQuadtreeNode(const uint32_t depth = 0, const int child_index = -1, const float log_odds = 0)
            : OccupancyQuadtreeNode(depth, child_index, log_odds) {}

        SurfaceMappingQuadtreeNode(const SurfaceMappingQuadtreeNode &other)
            : OccupancyQuadtreeNode(other),
              m_data_(std::make_shared<SurfaceData>(*other.m_data_)) {}

        SurfaceMappingQuadtreeNode &
        operator=(const SurfaceMappingQuadtreeNode &other) {
            if (this == &other) { return *this; }
            OccupancyQuadtreeNode::operator=(other);
            m_data_ = std::make_shared<SurfaceData>(*other.m_data_);
            return *this;
        }

        SurfaceMappingQuadtreeNode(SurfaceMappingQuadtreeNode &&other) noexcept = default;

        SurfaceMappingQuadtreeNode &
        operator=(SurfaceMappingQuadtreeNode &&other) noexcept = default;

        [[nodiscard]] AbstractQuadtreeNode *
        Create(const uint32_t depth, const int child_index) const override {
            return new SurfaceMappingQuadtreeNode(depth, child_index, /*log_odds*/ 0);
        }

        [[nodiscard]] AbstractQuadtreeNode *
        Clone() const override {
            return new SurfaceMappingQuadtreeNode(*this);
        }

        bool
        operator==(const AbstractQuadtreeNode &other) const override {
            if (OccupancyQuadtreeNode::operator==(other)) {
                const auto &other_node = reinterpret_cast<const SurfaceMappingQuadtreeNode &>(other);
                return *m_data_ == *other_node.m_data_;
            }
            return false;
        }

        void
        SetSurfaceData(Eigen::Vector2d position, Eigen::Vector2d normal, double var_position, double var_normal) {
            m_data_ = std::make_shared<SurfaceData>(std::move(position), std::move(normal), var_position, var_normal);
        }

        void
        SetSurfaceData(const std::shared_ptr<SurfaceData> &data) {
            m_data_ = data;
        }

        std::shared_ptr<SurfaceData>
        GetSurfaceData() {
            return m_data_;
        }

        void
        ResetSurfaceData() {
            m_data_.reset();
        }

        std::istream &
        ReadData(std::istream &s) override {
            OccupancyQuadtreeNode::ReadData(s);
            char has_data;
            s.read(&has_data, sizeof(char));
            if (has_data == 0) {
                m_data_.reset();
                return s;
            }
            m_data_ = std::make_shared<SurfaceData>();
            s.read(reinterpret_cast<char *>(m_data_->position.data()), sizeof(double) * 2);
            s.read(reinterpret_cast<char *>(m_data_->normal.data()), sizeof(double) * 2);
            s.read(reinterpret_cast<char *>(&m_data_->var_position), sizeof(double));
            s.read(reinterpret_cast<char *>(&m_data_->var_normal), sizeof(double));
            return s;
        }

        std::ostream &
        WriteData(std::ostream &s) const override {
            OccupancyQuadtreeNode::WriteData(s);
            if (m_data_ == nullptr) {
                s << static_cast<char>(0);
                return s;
            }
            s << static_cast<char>(1);
            s.write(reinterpret_cast<const char *>(m_data_->position.data()), sizeof(double) * 2);
            s.write(reinterpret_cast<const char *>(m_data_->normal.data()), sizeof(double) * 2);
            s.write(reinterpret_cast<const char *>(&m_data_->var_position), sizeof(double));
            s.write(reinterpret_cast<const char *>(&m_data_->var_normal), sizeof(double));
            return s;
        }
    };

    ERL_REGISTER_QUADTREE_NODE(SurfaceMappingQuadtreeNode);
}  // namespace erl::sdf_mapping
