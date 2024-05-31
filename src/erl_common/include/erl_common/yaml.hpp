#pragma once

#include "logging.hpp"
#include "opencv.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <memory>
#include <optional>

// https://yaml.org/spec/1.2.2/
// https://www.cloudbees.com/blog/yaml-tutorial-everything-you-need-get-started

namespace erl::common {

    struct YamlableBase {
        virtual ~YamlableBase() = default;

        virtual void
        FromYamlNode(const YAML::Node& node) = 0;

        void
        FromYamlString(const std::string& yaml_string) {
            const YAML::Node node = YAML::Load(yaml_string);
            FromYamlNode(node);
        }

        [[nodiscard]] virtual YAML::Node
        AsYamlNode() const = 0;

        [[nodiscard]] virtual std::string
        AsYamlString() const = 0;

        void
        FromYamlFile(const std::string& yaml_file) {
            const auto node = YAML::LoadFile(yaml_file);
            FromYamlNode(node);
        }

        void
        AsYamlFile(const std::string& yaml_file) const {
            std::ofstream ofs(yaml_file);
            ERL_ASSERTM(ofs.is_open(), "Failed to open file: {}", yaml_file);
            YAML::Emitter emitter(ofs);
            emitter.SetIndent(4);
            emitter.SetSeqFormat(YAML::Flow);
            emitter << AsYamlNode();
        }
    };

    template<typename T>
    struct Yamlable : public YamlableBase {

        void
        FromYamlNode(const YAML::Node& node) override {
            YAML::convert<T>::decode(node, *static_cast<T*>(this));
        }

        [[nodiscard]] YAML::Node
        AsYamlNode() const override {
            return YAML::convert<T>::encode(*static_cast<const T*>(this));
        }

        [[nodiscard]] std::string
        AsYamlString() const override {
            YAML::Emitter emitter;
            emitter.SetIndent(4);
            emitter.SetSeqFormat(YAML::Flow);
            emitter << AsYamlNode();
            return emitter.c_str();
        }
    };

    /**
     * @brief Override the Yamlable interface when the class is derived from a Yamlable class.
     * @tparam Base The base class that is derived from Yamlable<Base>.
     * @tparam T The derived class that is derived from Base.
     */
    template<typename Base, typename T>
    struct OverrideYamlable : public Base {
        void
        FromYamlNode(const YAML::Node& node) override {
            YAML::convert<T>::decode(node, *static_cast<T*>(this));
        }

        [[nodiscard]] YAML::Node
        AsYamlNode() const override {
            return YAML::convert<T>::encode(*static_cast<const T*>(this));
        }
    };
}  // namespace erl::common

namespace YAML {
    template<typename T, int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic, int Order = Eigen::ColMajor>
    struct ConvertEigenMatrix {
        static Node
        encode(const Eigen::Matrix<T, Rows, Cols, Order>& rhs) {
            Node node(NodeType::Sequence);
            const int rows = Rows == Eigen::Dynamic ? rhs.rows() : Rows;
            const int cols = Cols == Eigen::Dynamic ? rhs.cols() : Cols;

            if (Order == Eigen::RowMajor) {
                for (int i = 0; i < rows; ++i) {
                    Node row_node(NodeType::Sequence);
                    for (int j = 0; j < cols; ++j) { row_node.push_back(rhs(i, j)); }
                    node.push_back(row_node);
                }
            } else {
                for (int j = 0; j < cols; ++j) {
                    Node col_node(NodeType::Sequence);
                    for (int i = 0; i < rows; ++i) { col_node.push_back(rhs(i, j)); }
                    node.push_back(col_node);
                }
            }

            return node;
        }

        static bool
        decode(const Node& node, Eigen::Matrix<T, Rows, Cols, Order>& rhs) {
            if (node.IsNull() && (Rows == Eigen::Dynamic || Cols == Eigen::Dynamic)) { return true; }
            if (!node.IsSequence()) { return false; }
            if (!node[0].IsSequence()) { return false; }

            if (Order == Eigen::RowMajor) {
                int rows = Rows == Eigen::Dynamic ? node.size() : Rows;
                int cols = Cols == Eigen::Dynamic ? node[0].size() : Cols;
                rhs.resize(rows, cols);
                ERL_DEBUG_ASSERT(rows == static_cast<int>(node.size()), "expecting rows: {}, get node.size(): {}", rows, node.size());
                for (int i = 0; i < rows; ++i) {
                    ERL_DEBUG_ASSERT(cols == static_cast<int>(node[i].size()), "expecting cols: {}, get node[0].size(): {}", cols, node[i].size());
                    auto& row_node = node[i];
                    for (int j = 0; j < cols; ++j) { rhs(i, j) = row_node[j].as<T>(); }
                }
            } else {
                int cols = Cols == Eigen::Dynamic ? node.size() : Cols;
                int rows = Rows == Eigen::Dynamic ? node[0].size() : Rows;
                rhs.resize(rows, cols);
                ERL_DEBUG_ASSERT(cols == static_cast<int>(node.size()), "expecting cols: {}, get node.size(): {}", cols, node.size());
                for (int j = 0; j < cols; ++j) {
                    ERL_DEBUG_ASSERT(rows == static_cast<int>(node[j].size()), "expecting rows: {}, get node[0].size(): {}", rows, node[j].size());
                    auto& col_node = node[j];
                    for (int i = 0; i < rows; ++i) { rhs(i, j) = col_node[i].as<T>(); }
                }
            }

            return true;
        }
    };

    template<>
    struct convert<Eigen::Matrix2i> : public ConvertEigenMatrix<int, 2, 2> {};

    template<>
    struct convert<Eigen::Matrix2f> : public ConvertEigenMatrix<float, 2, 2> {};

    template<>
    struct convert<Eigen::Matrix2d> : public ConvertEigenMatrix<double, 2, 2> {};

    template<>
    struct convert<Eigen::Matrix2Xi> : public ConvertEigenMatrix<int, 2, Eigen::Dynamic> {};

    template<>
    struct convert<Eigen::Matrix2Xf> : public ConvertEigenMatrix<float, 2, Eigen::Dynamic> {};

    template<>
    struct convert<Eigen::Matrix2Xd> : public ConvertEigenMatrix<double, 2, Eigen::Dynamic> {};

    template<>
    struct convert<Eigen::MatrixX2i> : public ConvertEigenMatrix<int, Eigen::Dynamic, 2> {};

    template<>
    struct convert<Eigen::MatrixX2f> : public ConvertEigenMatrix<float, Eigen::Dynamic, 2> {};

    template<>
    struct convert<Eigen::MatrixX2d> : public ConvertEigenMatrix<double, Eigen::Dynamic, 2> {};

    template<>
    struct convert<Eigen::Matrix3i> : public ConvertEigenMatrix<int, 3, 3> {};

    template<>
    struct convert<Eigen::Matrix3f> : public ConvertEigenMatrix<float, 3, 3> {};

    template<>
    struct convert<Eigen::Matrix3d> : public ConvertEigenMatrix<double, 3, 3> {};

    template<>
    struct convert<Eigen::Matrix3Xi> : public ConvertEigenMatrix<int, 3, Eigen::Dynamic> {};

    template<>
    struct convert<Eigen::Matrix3Xf> : public ConvertEigenMatrix<float, 3, Eigen::Dynamic> {};

    template<>
    struct convert<Eigen::Matrix3Xd> : public ConvertEigenMatrix<double, 3, Eigen::Dynamic> {};

    template<>
    struct convert<Eigen::MatrixX3i> : public ConvertEigenMatrix<int, Eigen::Dynamic, 3> {};

    template<>
    struct convert<Eigen::MatrixX3f> : public ConvertEigenMatrix<float, Eigen::Dynamic, 3> {};

    template<>
    struct convert<Eigen::MatrixX3d> : public ConvertEigenMatrix<double, Eigen::Dynamic, 3> {};

    template<>
    struct convert<Eigen::Matrix4i> : public ConvertEigenMatrix<int, 4, 4> {};

    template<>
    struct convert<Eigen::Matrix4f> : public ConvertEigenMatrix<float, 4, 4> {};

    template<>
    struct convert<Eigen::Matrix4d> : public ConvertEigenMatrix<double, 4, 4> {};

    template<>
    struct convert<Eigen::Matrix4Xi> : public ConvertEigenMatrix<int, 4, Eigen::Dynamic> {};

    template<>
    struct convert<Eigen::Matrix4Xf> : public ConvertEigenMatrix<float, 4, Eigen::Dynamic> {};

    template<>
    struct convert<Eigen::Matrix4Xd> : public ConvertEigenMatrix<double, 4, Eigen::Dynamic> {};

    template<>
    struct convert<Eigen::MatrixX4i> : public ConvertEigenMatrix<int, Eigen::Dynamic, 4> {};

    template<>
    struct convert<Eigen::MatrixX4f> : public ConvertEigenMatrix<float, Eigen::Dynamic, 4> {};

    template<>
    struct convert<Eigen::MatrixX4d> : public ConvertEigenMatrix<double, Eigen::Dynamic, 4> {};

    template<typename T, int Size = Eigen::Dynamic>
    struct ConvertEigenVector {
        static Node
        encode(const Eigen::Vector<T, Size>& rhs) {
            Node node(NodeType::Sequence);
            if (Size == Eigen::Dynamic) {
                for (int i = 0; i < rhs.size(); ++i) { node.push_back(rhs[i]); }
            } else {
                for (int i = 0; i < Size; ++i) { node.push_back(rhs[i]); }
            }
            return node;
        }

        static bool
        decode(const Node& node, Eigen::Vector<T, Size>& rhs) {
            if (!node.IsSequence()) { return false; }
            if (Size == Eigen::Dynamic) {
                rhs.resize(node.size());
                for (int i = 0; i < rhs.size(); ++i) { rhs[i] = node[i].as<T>(); }
            } else {
                for (int i = 0; i < Size; ++i) { rhs[i] = node[i].as<T>(); }
            }
            return true;
        }
    };

    template<>
    struct convert<Eigen::VectorXd> : public ConvertEigenVector<double, Eigen::Dynamic> {};

    template<>
    struct convert<Eigen::Vector2d> : public ConvertEigenVector<double, 2> {};

    template<>
    struct convert<Eigen::Vector3d> : public ConvertEigenVector<double, 3> {};

    template<>
    struct convert<Eigen::Vector4d> : public ConvertEigenVector<double, 4> {};

    template<>
    struct convert<Eigen::VectorXf> : public ConvertEigenVector<float, Eigen::Dynamic> {};

    template<>
    struct convert<Eigen::Vector2f> : public ConvertEigenVector<float, 2> {};

    template<>
    struct convert<Eigen::Vector3f> : public ConvertEigenVector<float, 3> {};

    template<>
    struct convert<Eigen::Vector4f> : public ConvertEigenVector<float, 4> {};

    template<>
    struct convert<Eigen::VectorXi> : public ConvertEigenVector<int, Eigen::Dynamic> {};

    template<>
    struct convert<Eigen::Vector2i> : public ConvertEigenVector<int, 2> {};

    template<>
    struct convert<Eigen::Vector3i> : public ConvertEigenVector<int, 3> {};

    template<>
    struct convert<Eigen::Vector4i> : public ConvertEigenVector<int, 4> {};

    template<typename... Args>
    struct convert<std::tuple<Args...>> {
        static Node
        encode(const std::tuple<Args...>& rhs) {
            Node node(NodeType::Sequence);
            std::apply([&node](const Args&... args) { (node.push_back(convert<Args>::encode(args)), ...); }, rhs);
            return node;
        }

        static bool
        decode(const Node& node, std::tuple<Args...>& rhs) {
            if (!node.IsSequence()) { return false; }
            if (node.size() != sizeof...(Args)) { return false; }
            std::apply([&node](Args&... args) { (convert<Args>::decode(node, args) && ...); }, rhs);
            return true;
        }
    };

    template<typename T>
    struct convert<std::optional<T>> {
        static Node
        encode(const std::optional<T>& rhs) {
            if (rhs) { return convert<T>::encode(*rhs); }
            return Node(NodeType::Null);
        }

        static bool
        decode(const Node& node, std::optional<T>& rhs) {
            if (node.Type() != NodeType::Null) {
                T value;
                if (convert<T>::decode(node, value)) {
                    rhs = value;
                    return true;
                }
                return false;
            }
            rhs = std::nullopt;
            return true;
        }
    };

    template<typename T>
    struct convert<std::shared_ptr<T>> {
        static Node
        encode(const std::shared_ptr<T>& rhs) {
            if (rhs == nullptr) { return Node(NodeType::Null); }
            return convert<T>::encode(*rhs);
        }

        static bool
        decode(const Node& node, std::shared_ptr<T>& rhs) {
            if (node.IsNull()) {
                rhs = nullptr;
                return true;
            }
            auto value = std::make_shared<T>();
            if (convert<T>::decode(node, *value)) {
                rhs = value;
                return true;
            }
            return false;
        }
    };

    template<typename T>
    struct convert<std::unique_ptr<T>> {
        static Node
        encode(const std::unique_ptr<T>& rhs) {
            if (rhs == nullptr) { return Node(NodeType::Null); }
            return convert<T>::encode(*rhs);
        }

        static bool
        decode(const Node& node, std::unique_ptr<T>& rhs) {
            if (node.IsNull()) {
                rhs = nullptr;
                return true;
            }
            auto value = std::make_unique<T>();
            if (convert<T>::decode(node, *value)) {
                rhs = std::move(value);
                return true;
            }
            return false;
        }
    };

    template<typename KeyType, typename ValueType>
    struct convert<std::unordered_map<KeyType, ValueType>> {
        static Node
        encode(const std::unordered_map<KeyType, ValueType>& rhs) {
            Node node(NodeType::Map);
            for (const auto& [key, value]: rhs) { node[convert<KeyType>::encode(key)] = convert<ValueType>::encode(value); }
            return node;
        }

        static bool
        decode(const Node& node, std::unordered_map<KeyType, ValueType>& rhs) {
            if (!node.IsMap()) { return false; }
            for (auto it = node.begin(); it != node.end(); ++it) {
                KeyType key;
                ValueType value;
                if (convert<KeyType>::decode(it->first, key) && convert<ValueType>::decode(it->second, value)) {
                    rhs[key] = value;
                } else {
                    return false;
                }
            }
            return true;
        }
    };

    template<typename Period>
    struct convert<std::chrono::duration<int64_t, Period>> {
        static Node
        encode(const std::chrono::duration<int64_t, Period>& rhs) {
            return Node(rhs.count());
        }

        static bool
        decode(const Node& node, std::chrono::duration<int64_t, Period>& rhs) {
            if (!node.IsScalar()) { return false; }
            rhs = std::chrono::duration<int64_t, Period>(node.as<int64_t>());
            return true;
        }
    };

    template<>
    struct convert<cv::Scalar> {
        static Node
        encode(const cv::Scalar& rhs) {
            Node node(NodeType::Sequence);
            node.push_back(rhs[0]);
            node.push_back(rhs[1]);
            node.push_back(rhs[2]);
            node.push_back(rhs[3]);
            return node;
        }

        static bool
        decode(const Node& node, cv::Scalar& rhs) {
            if (!node.IsSequence()) { return false; }
            rhs[0] = node[0].as<double>();
            rhs[1] = node[1].as<double>();
            rhs[2] = node[2].as<double>();
            rhs[3] = node[3].as<double>();
            return true;
        }
    };
}  // namespace YAML

inline std::ostream&
operator<<(std::ostream& out, const erl::common::YamlableBase& yaml) {
    out << yaml.AsYamlString();
    return out;
}
