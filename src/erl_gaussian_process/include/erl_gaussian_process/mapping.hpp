#pragma once

#include "erl_common/yaml.hpp"

#include <functional>
#include <memory>

namespace erl::gaussian_process {

    class Mapping {

    public:
        enum class Type { kIdentity = 0, kInverse = 1, kInverseSqrt = 2, kExp = 3, kLog = 4, kTanh = 5, kSigmoid = 6, kUnknown = 7 };

        struct Setting : public common::Yamlable<Setting> {
            Type type = Type::kUnknown;
            double scale = 1.0;
        };

        static const char *
        GetTypeName(const Type &type) {
            static const char *names[] = {"kIdentity", "kInverse", "kInverseSqrt", "kExp", "kLog", "kTanh", "kSigmoid", "kUnknown"};
            return names[static_cast<int>(type)];
        }

        static Type
        GetTypeFromName(const std::string &type_name) {
            if (type_name == "kIdentity") { return Type::kIdentity; }
            if (type_name == "kInverse") { return Type::kInverse; }
            if (type_name == "kInverseSqrt") { return Type::kInverseSqrt; }
            if (type_name == "kExp") { return Type::kExp; }
            if (type_name == "kLog") { return Type::kLog; }
            if (type_name == "kTanh") { return Type::kTanh; }
            if (type_name == "kSigmoid") { return Type::kSigmoid; }
            return Type::kUnknown;
        }

    protected:
        std::shared_ptr<Setting> m_setting_;

    public:
        std::function<double(double)> map;
        std::function<double(double)> inv;

        static std::shared_ptr<Mapping>
        Create();

        static std::shared_ptr<Mapping>
        Create(std::shared_ptr<Setting> setting);

        [[nodiscard]] std::shared_ptr<Setting>
        GetSetting() const {
            return m_setting_;
        }

    protected:
        Mapping();

        explicit Mapping(std::shared_ptr<Setting> setting);
    };
}  // namespace erl::gaussian_process

template<>
struct YAML::convert<erl::gaussian_process::Mapping::Setting> {
    static Node
    encode(const erl::gaussian_process::Mapping::Setting &setting) {
        Node node;
        node["type"] = erl::gaussian_process::Mapping::GetTypeName(setting.type);
        node["scale"] = setting.scale;
        return node;
    }

    static bool
    decode(const Node &node, erl::gaussian_process::Mapping::Setting &setting) {
        if (!node.IsMap()) { return false; }
        setting.type = erl::gaussian_process::Mapping::GetTypeFromName(node["type"].as<std::string>());
        setting.scale = node["scale"].as<double>();
        return true;
    }
};  // namespace YAML
